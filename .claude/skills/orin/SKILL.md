---
name: orin
description: How to work on FRIDA's two Jetson Orins (HRI Orin and main Orin) — connecting over SSH, which checkout to use (~/home2 vs ~/dev/<area>), what you may and may not change there, how to test, and how to bring changes back to git. Use whenever a task involves running, testing, debugging or syncing code on the Orin/robot.
---

# Working on the Orin

FRIDA has **two Orins**, both shared robot computers. Treat them as deploy
targets, not dev machines:

| Orin | Runs |
| --- | --- |
| HRI Orin | Only `hri` (listed in `ORIN_SERVER_AREAS` in `lib.sh`) |
| Main Orin | Every other area: vision, navigation, manipulation, integration, zed, display |

Make sure you are on the right one for the area you are working on.

## Golden rules

1. **The Orin is pull-only.** Edit and commit on your own machine, push, then
   pull on the Orin. Never commit from the Orin and never leave uncommitted
   edits there.
2. **Only touch your own area.** Don't stop, rebuild or recreate another
   area's containers, and don't change another area's checkout.
3. **Host config is off-limits** (`/etc/*`, sysctl, udev, `/etc/pip.conf`,
   JetPack/drivers, docker daemon) unless the user explicitly asks. If a host
   change is made, document it in `docs/` (see
   `docs/jetpack7-jazzy-migration.md` → "Host configuration").
4. **Ask before anything destructive or global**: `./run.sh --stop|--down|--build|--clean`
   act on **all** areas and kill every `screen` session; `docker system prune`,
   `docker rmi`, `git reset --hard`, `git clean`, reboots.

## Connecting

Connect over SSH from the robot's Wi-Fi network. Hosts and IPs are still
changing, so **ask the user for the host** of the Orin you need instead of
guessing or hardcoding one.

`lib.sh` reaches the HRI Orin from the main one through `ORIN_SSH_USER/PASS/HOST`
in the repo-root `.env` (see `.env.example`) for areas in `ORIN_SERVER_AREAS`.
If SSH times out, you are not on the robot's network — tell the user instead of
retrying.

## Which checkout: `~/home2` vs `~/dev/<area>`

| Path | Use it for | Branch |
| --- | --- | --- |
| `~/home2` | Integration and competition runs (`./run.sh --gpsr`, `--restaurant`, ...) | `main` / the agreed integration branch — don't switch it |
| `~/dev/<area>` (e.g. `~/dev/nav`) | Testing your area's feature branch without disturbing others | Your feature branch |

Containers have **fixed names** (`home2-navigation`, `home2-manipulation`, ...)
and mount the checkout that created them (`../../:/workspace/src`). Before
testing, check which checkout a running container is using:

```bash
docker inspect home2-navigation --format '{{range .Mounts}}{{.Source}} {{end}}'
```

When switching between `~/home2` and `~/dev/<area>`, recreate **only your**
container: `./run.sh <area> --recreate`.

## Syncing code to the Orin

```bash
# laptop: commit + push first
cd ~/dev/<area>            # or ~/home2 if that's what is being run
git status                 # must be clean; if not, see "Changes made on the Orin"
git fetch origin
git checkout <branch>
git pull --ff-only origin <branch>   # explicit remote: the branch may have no upstream
git submodule update --init --recursive
```

A plain `git pull` with no upstream can silently do nothing — always check
`git log -1` matches the commit you pushed.

## Testing

```bash
./run.sh <area>                      # starts/enters the area container (l4t auto-detected)
docker exec -it home2-<area> bash    # extra shell in a running container
./run.sh <area> --build              # rebuild ROS packages after pulling C++/interface changes
./scripts/status.sh <area> <task>    # check required nodes are up
docker ps; screen -ls                # see what else is running before you start
tegrastats                           # CPU/GPU/RAM load on the Jetson
```

- Check `docker ps` / `screen -ls` first: someone else may be using the robot.
- Hardware tests (base, arm, cameras) move a real robot — confirm with the user
  that it is safe before launching anything that actuates.
- Report results with the actual command output; don't claim it works from a
  build alone.

## Changes made on the Orin (hotfixes)

If something was edited on the Orin (e.g. a fix during a competition), bring it
back to git instead of leaving it there:

```bash
# Orin
cd ~/dev/<area>
git diff > /tmp/orin-hotfix.patch          # include new files: git add -N <file> first
# laptop
scp <orin-user>@<orin-host>:/tmp/orin-hotfix.patch .
git apply orin-hotfix.patch && git commit -am "<area>: <what the hotfix does>" && git push
# Orin, once the commit is pushed (ask the user before discarding)
git checkout -- . && git pull --ff-only origin <branch>
```
