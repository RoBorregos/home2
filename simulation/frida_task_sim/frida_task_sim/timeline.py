"""Human-readable view of referee events: one line per event with the running score."""

from frida_task_sim import scoring

RESET, DIM, BOLD = "\033[0m", "\033[2m", "\033[1m"
RED, GREEN, YELLOW, BLUE, CYAN = (
    "\033[31m",
    "\033[32m",
    "\033[33m",
    "\033[34m",
    "\033[36m",
)


def clock(t: float) -> str:
    return f"{int(t) // 60:02d}:{int(t) % 60:02d}"


def describe(event: dict) -> tuple[str, str] | None:
    """(colour, text) for an event, or None when it is not worth a line."""
    kind = event["type"]
    if kind == "speech":
        who = "FRIDA" if event["who"] == "robot" else event["who"].upper()
        return (
            CYAN if event["who"] == "robot" else YELLOW
        ), f'{who}: "{event["text"]}"'
    if kind == "step":
        return DIM, f"step  {event['name']}"
    if kind == "start":
        return BOLD, "START  button pressed"
    if kind == "door_open":
        return BOLD, "DOOR  the arena door opens"
    if kind == "at_table":
        return GREEN, "REACHED the dining table"
    if kind == "recognize":
        mark = (GREEN, "RECOGNIZED") if event["correct"] else (RED, "WRONG RECOGNITION")
        return mark[0], f"{mark[1]}  {event['name']} at {event['at'] or 'nowhere'}"
    if kind == "shelf_indicated":
        colour = GREEN if event["correct"] else RED
        return (
            colour,
            f"SHELF  {event['object']} -> shelf {event['level'] + 1} ({'right' if event['correct'] else 'wrong'})",
        )
    if kind == "pick":
        return GREEN, f"PICKED  {event['object']} from {event['source']}"
    if kind == "place":
        level = "" if event["level"] is None else f" shelf {event['level'] + 1}"
        colour = RED if event["location"] in ("floor", "air") else BLUE
        return (
            colour,
            f"PLACED  {event['object']} on {event['location']}{level} (released {event['drop']:.2f} m above)",
        )
    if kind == "knocked":
        return RED, f"KNOCKED  {event['object']} off {event['source']}"
    if kind == "finish":
        return BOLD, "FINISH"
    return None


class Timeline:
    """Accumulates events and reports score changes."""

    def __init__(self, scenario: dict):
        self.scenario = scenario
        self.events: list[dict] = []
        self.total = 0
        self.rows: dict[str, int] = {}

    def add(self, event: dict) -> list[str]:
        """Plain-text lines for this event: its description and any points it earned."""
        self.events.append(event)
        lines = []
        described = describe(event)
        if described:
            lines.append((described[0], f"[{clock(event['t'])}] {described[1]}"))
        result = scoring.score(self.scenario, self.events, {})
        for row in result["rows"]:
            delta = row["total"] - self.rows.get(row["id"], 0)
            if delta:
                self.rows[row["id"]] = row["total"]
                colour = GREEN if delta > 0 else RED
                lines.append(
                    (
                        colour,
                        f"          {delta:+d} pts  {row['label']}   (score {result['total']})",
                    )
                )
        self.total = result["total"]
        return lines
