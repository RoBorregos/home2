"""
Read-only client for the Candidates Dashboard Robot API (GET requests only),
an offline backup with the same interface, and a source that falls back to it.

Shared by the HRI dialogue agent and the task manager experiments.
Config: docker/robot_api.env (gitignored) with ROBOT_API_URL, ROBOT_API_KEY
and optionally ROBOT_API_BACKUP.
"""

import difflib
import json
import os
import re
import unicodedata
import urllib.error
import urllib.parse
import urllib.request

DEFAULT_EDITION = 2026
DEFAULT_TIMEZONE = "America/Monterrey"
MIN_SCORE = 0.6
ENV_FILE = os.getenv("ROBOT_API_ENV", "/workspace/src/docker/robot_api.env")
DATA_DIR = "/workspace/src/task_manager/benchmarks/data"
BACKUP_FILE = os.path.join(DATA_DIR, "schedule_backup.json")
EXAMPLE_BACKUP_FILE = os.path.join(DATA_DIR, "schedule_backup.example.json")


class ApiError(Exception):
    def __init__(self, status: int, message: str):
        super().__init__(f"{status}: {message}")
        self.status = status
        self.message = message

    @property
    def unreachable(self) -> bool:
        """Network failure or server error: worth retrying against the backup."""
        return self.status == 0 or self.status >= 500


def load_env_file(path: str) -> dict:
    """Parse KEY=VALUE lines. Missing file -> empty dict."""
    values = {}
    if not os.path.isfile(path):
        return values
    with open(path, "r") as file:
        for line in file:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            values[key.strip()] = value.strip().strip('"').strip("'")
    return values


class RobotAPI:
    """HTTP client. Only GET requests; never writes to the dashboard."""

    def __init__(self, base_url: str, api_key: str, timeout: float = 5.0):
        if not base_url or not api_key:
            raise ValueError("ROBOT_API_URL and ROBOT_API_KEY are required")
        self.base_url = base_url.rstrip("/")
        self.api_key = api_key
        self.timeout = timeout

    def _get(self, path: str, params: dict | None = None) -> dict:
        params = {k: v for k, v in (params or {}).items() if v is not None}
        query = urllib.parse.urlencode(params)
        url = f"{self.base_url}{path}" + (f"?{query}" if query else "")
        request = urllib.request.Request(
            url,
            method="GET",
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Accept": "application/json",
            },
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                return json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as e:
            try:
                message = json.loads(e.read().decode("utf-8")).get("error", e.reason)
            except Exception:
                message = str(e.reason)
            raise ApiError(e.code, message) from e
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            raise ApiError(0, f"Connection error: {e}") from e

    def ping(self) -> dict:
        return self._get("/ping")

    def search_candidates(
        self, name: str, team=None, track=None, limit=3, edition=DEFAULT_EDITION
    ) -> dict:
        return self._get(
            "/candidates/search",
            {
                "name": name,
                "team": team,
                "track": track,
                "limit": limit,
                "edition": edition,
            },
        )

    def search_teams(self, name: str, limit=3, edition=DEFAULT_EDITION) -> dict:
        return self._get(
            "/teams/search", {"name": name, "limit": limit, "edition": edition}
        )

    def candidate(self, candidate_id: str, edition=DEFAULT_EDITION) -> dict:
        return self._get(
            f"/candidates/{urllib.parse.quote(candidate_id)}",
            {"edition": edition, "lang": "en"},
        )

    def team(self, team_id: str, edition=DEFAULT_EDITION) -> dict:
        return self._get(
            f"/teams/{urllib.parse.quote(team_id)}", {"edition": edition, "lang": "en"}
        )


_PARTICLES = {"de", "del", "la", "las", "los", "y", "team", "the"}


def normalize_name(text: str) -> list[str]:
    """Rough version of the API matching: no accents/case/particles, Spanish sound-alikes."""
    text = unicodedata.normalize("NFKD", text or "").encode("ascii", "ignore").decode()
    text = re.sub(r"[^a-z0-9\s]", " ", text.lower())
    text = text.replace("v", "b").replace("z", "s").replace("x", "j")
    text = re.sub(r"c([ei])", r"s\1", text)
    text = re.sub(r"g([ei])", r"j\1", text)
    return [w for w in text.split() if w not in _PARTICLES]


def name_score(query: str, name: str) -> tuple[float, bool]:
    """(similarity 0-1, every query word appears in name)."""
    query_words, name_words = normalize_name(query), normalize_name(name)
    if not query_words or not name_words:
        return 0.0, False
    joined = "".join(name_words)
    best = [
        max(
            [difflib.SequenceMatcher(None, q, n).ratio() for n in name_words]
            + [1.0 if q in joined and len(q) > 3 else 0.0]
        )
        for q in query_words
    ]
    contains_all = all(
        q in name_words or (len(q) > 3 and q in joined) for q in query_words
    )
    return sum(best) / len(best), contains_all


def rank(query: str, items: list[dict], key="name", limit=3) -> tuple[bool, list]:
    """Score items against query like the API: drop < MIN_SCORE, exact if one contains all."""
    scored = []
    for item in items:
        score, contains_all = name_score(query, item[key])
        scored.append((1.0 if contains_all else score, contains_all, item))
    scored.sort(key=lambda s: s[0], reverse=True)
    containing = [s for s in scored if s[1]]
    if len(containing) == 1:
        return True, [(1.0, containing[0][2])]
    kept = [(round(s, 2), item) for s, _, item in scored if s >= MIN_SCORE]
    return False, kept[: max(1, min(10, int(limit)))]


class BackupAPI:
    """Offline copy of the schedule with the same responses as RobotAPI (lang=en).

    File format: see task_manager/benchmarks/data/schedule_backup.template.json
    """

    def __init__(self, path: str):
        with open(path, "r") as file:
            data = json.load(file)
        self.path = path
        self.timezone = data.get("timezone", DEFAULT_TIMEZONE)
        self.teams = data.get("teams", [])
        self.solo = data.get("candidates_without_team", [])

    def _candidates(self):
        for team in self.teams:
            for member in team.get("members", []):
                yield member, team
        for candidate in self.solo:
            yield candidate, None

    def team_names(self) -> list[str]:
        return [t["name"] for t in self.teams]

    def candidate_names(self) -> list[str]:
        return [c["name"] for c, _ in self._candidates()]

    def ping(self) -> dict:
        return {"ok": True, "edition": DEFAULT_EDITION, "backup": True}

    def search_teams(self, name: str, limit=3, edition=DEFAULT_EDITION) -> dict:
        exact, ranked = rank(name, self.teams, limit=limit)
        return {
            "query": name.strip(),
            "exact": exact,
            "matches": [
                {
                    "id": t["id"],
                    "name": t["name"],
                    "score": score,
                    "memberCount": len(t.get("members", [])),
                }
                for score, t in ranked
            ],
        }

    def search_candidates(
        self, name: str, team=None, track=None, limit=3, edition=DEFAULT_EDITION
    ) -> dict:
        result = {"query": name.strip()}
        pool = list(self._candidates())
        if team:
            _, ranked_teams = rank(team, self.teams, limit=1)
            if not ranked_teams:
                best = max(
                    ((name_score(team, t["name"])[0], t) for t in self.teams),
                    key=lambda s: s[0],
                    default=(0.0, None),
                )
                if best[1] is not None:
                    result["team"] = {
                        "id": best[1]["id"],
                        "name": best[1]["name"],
                        "score": round(best[0], 2),
                    }
                result.update(exact=False, matches=[])
                return result
            score, best_team = ranked_teams[0]
            result["team"] = {
                "id": best_team["id"],
                "name": best_team["name"],
                "score": score,
            }
            pool = [(c, t) for c, t in pool if t is best_team]
        if track:
            pool = [(c, t) for c, t in pool if (t or c).get("track") == track.upper()]

        items = [{"name": c["name"], "_c": c, "_t": t} for c, t in pool]
        exact, ranked = rank(name, items, limit=limit)
        result["exact"] = exact
        result["matches"] = [
            {
                "id": item["_c"]["id"],
                "name": item["_c"]["name"],
                "team": item["_t"]["name"] if item["_t"] else None,
                "score": score,
            }
            for score, item in ranked
        ]
        return result

    def candidate(self, candidate_id: str, edition=DEFAULT_EDITION) -> dict:
        for candidate, team in self._candidates():
            if candidate["id"] == candidate_id:
                source = team or candidate
                return {
                    "timezone": self.timezone,
                    "name": candidate["name"],
                    "track": source.get("track"),
                    "challenge": source.get("challenge"),
                    "teamId": team["id"] if team else None,
                    "team": team["name"] if team else None,
                    "interview": candidate.get("interview"),
                    "rounds": team.get("rounds", []) if team else [],
                }
        raise ApiError(404, "Candidate not found")

    def team(self, team_id: str, edition=DEFAULT_EDITION) -> dict:
        for team in self.teams:
            if team["id"] == team_id:
                return {
                    "timezone": self.timezone,
                    "name": team["name"],
                    "track": team.get("track"),
                    "challenge": team.get("challenge"),
                    "members": [
                        {
                            "id": m["id"],
                            "name": m["name"],
                            "interview": m.get("interview"),
                        }
                        for m in team.get("members", [])
                    ],
                    "rounds": team.get("rounds", []),
                }
        raise ApiError(404, "Team not found")


class ScheduleSource:
    """Online API first; on network/server errors falls back to the backup.

    `last_source` says which one answered the last call ("online" / "backup").
    """

    METHODS = ("ping", "search_candidates", "search_teams", "candidate", "team")

    def __init__(self, online: RobotAPI | None, backup: BackupAPI | None):
        if online is None and backup is None:
            raise ValueError("Need an online API or a backup file")
        self.online = online
        self.backup = backup
        self.last_source = None
        for name in self.METHODS:
            setattr(self, name, self._dispatch(name))

    def _dispatch(self, name):
        def call(*args, **kwargs):
            if self.online is not None:
                try:
                    result = getattr(self.online, name)(*args, **kwargs)
                    self.last_source = "online"
                    return result
                except ApiError as e:
                    if not (e.unreachable and self.backup is not None):
                        raise
            self.last_source = "backup"
            return getattr(self.backup, name)(*args, **kwargs)

        return call

    def team_names(self) -> list[str]:
        return self.backup.team_names() if self.backup else []

    def candidate_names(self) -> list[str]:
        return self.backup.candidate_names() if self.backup else []


def make_schedule_source(
    offline: bool = False, env_file: str = ENV_FILE, timeout: float = 5.0
) -> ScheduleSource:
    """Build the source from docker/robot_api.env (env vars override the file)."""
    values = load_env_file(env_file)

    def setting(key, default=""):
        return os.getenv(key, values.get(key, default))

    online = None
    if not offline and setting("ROBOT_API_URL") and setting("ROBOT_API_KEY"):
        online = RobotAPI(setting("ROBOT_API_URL"), setting("ROBOT_API_KEY"), timeout)

    backup_path = setting("ROBOT_API_BACKUP", BACKUP_FILE)
    if not os.path.isfile(backup_path):
        backup_path = EXAMPLE_BACKUP_FILE
    backup = BackupAPI(backup_path) if os.path.isfile(backup_path) else None
    return ScheduleSource(online, backup)
