#!/usr/bin/env python3
"""Offline self-test for the semantic navigation core (no ROS, no robot).

Exercises nav_main.semantic against the real arena map in map_context, so the
patrol logic can be verified on a laptop before anything is flashed:

    python3 navigation/packages/nav_main/scripts/semantic_nav_selftest.py
    python3 ... --map robocup_c1          # another map
    ros2 run nav_main semantic_nav_selftest.py

Exits non-zero if any check fails, so it can be wired into CI later.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
import time

# Run from a source checkout without installing: nav_main/ lives two levels up.
_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from nav_main.semantic.areas import area_for_point, areas_containing  # noqa: E402
from nav_main.semantic.route import (  # noqa: E402
    order_by_staleness,
    order_route,
    route_length,
)
from nav_main.semantic.staleness import ScanLog  # noqa: E402
from nav_main.semantic.surfaces import SURFACE_TYPES, classify, load_viewpoints  # noqa: E402
from nav_main.semantic.viewpoints import (  # noqa: E402
    Grid,
    grid_from_pgm,
    is_area_free,
    relocate,
    target_of,
    validate_viewpoints,
)

PASSED = 0
FAILED = 0


def warn(name: str, detail: str = "") -> None:
    """A map-data problem worth surfacing that is not a code failure."""
    print(f"  \033[93mWARN\033[0m {name}" + (f" — {detail}" if detail else ""))


def check(name: str, condition: bool, detail: str = "") -> bool:
    global PASSED, FAILED
    if condition:
        PASSED += 1
        print(f"  \033[92mPASS\033[0m {name}" + (f" — {detail}" if detail else ""))
    else:
        FAILED += 1
        print(f"  \033[91mFAIL\033[0m {name}" + (f" — {detail}" if detail else ""))
    return condition


def load_map_yaml(path: str) -> dict:
    """Minimal YAML reader for a nav2 map file (avoids a PyYAML dependency)."""
    out: dict = {}
    with open(path) as handle:
        for line in handle:
            line = line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, value = (part.strip() for part in line.split(":", 1))
            if value.startswith("["):
                out[key] = json.loads(value)
            else:
                try:
                    out[key] = float(value) if "." in value or value.isdigit() else value
                except ValueError:
                    out[key] = value
    return out


# --------------------------------------------------------------------- checks


def test_areas(areas_data: dict) -> None:
    print("\nareas — punto -> cuarto + mueble")
    rooms = [a for a, d in areas_data.items() if isinstance(d, dict) and d.get("polygon")]
    check("hay cuartos con polígono", bool(rooms), f"{len(rooms)}: {rooms}")

    misplaced = []
    total = 0
    for room in rooms:
        for name, pose in areas_data[room].items():
            if name in ("polygon", "safe_place") or name.endswith("_meta"):
                continue
            if not isinstance(pose, list) or len(pose) < 2:
                continue
            total += 1
            area, subs, dists, in_house = area_for_point(areas_data, pose[0], pose[1])
            if area != room:
                misplaced.append(f"{room}/{name} -> '{area}'")
            elif name not in subs:
                misplaced.append(f"{room}/{name} no se encuentra a sí mismo: {subs[:2]}")
    check(
        "cada mueble cae en su propio cuarto y se encuentra a sí mismo",
        not misplaced,
        f"{total - len(misplaced)}/{total}" + (f"; fallas: {misplaced}" if misplaced else ""),
    )

    # Two pieces of furniture tagged at the same spot make the patrol stop twice
    # in one place and stare at the wrong thing half the time. The code handles
    # it; the map should still be fixed with the tagger.
    seen: dict[tuple[float, float], str] = {}
    for room in rooms:
        for name, pose in areas_data[room].items():
            if name in ("polygon", "safe_place") or name.endswith("_meta"):
                continue
            if not isinstance(pose, list) or len(pose) < 2:
                continue
            key = (round(pose[0], 3), round(pose[1], 3))
            if key in seen:
                warn(
                    "dos muebles comparten pose etiquetada",
                    f"{seen[key]} y {room}/{name} en ({key[0]}, {key[1]}) — reetiquetar con el tagger",
                )
            else:
                seen[key] = f"{room}/{name}"

    area, subs, dists, in_house = area_for_point(areas_data, 999.0, 999.0)
    check("punto fuera del mapa no inventa cuarto", area == "" and not in_house and not subs)

    waypoints = [a for a, d in areas_data.items() if isinstance(d, dict) and not d.get("polygon")]
    leaked = [w for w in waypoints if w in areas_containing(areas_data, 0.0, 0.0)]
    check(
        "las áreas sin polígono nunca se devuelven",
        not leaked,
        f"{len(waypoints)} waypoints ignorados: {waypoints}",
    )

    room = rooms[0]
    subs_all = [
        n
        for n, v in areas_data[room].items()
        if n not in ("polygon", "safe_place") and not n.endswith("_meta") and isinstance(v, list)
    ]
    if subs_all:
        pose = areas_data[room][subs_all[0]]
        _, near, dists, _ = area_for_point(areas_data, pose[0], pose[1], 0.1)
        check("el radio de sublocations se respeta", all(d <= 0.1 for d in dists), f"{near}")


def test_surfaces(areas_data: dict) -> None:
    print("\nsurfaces — clasificación de muebles")
    vps = load_viewpoints(areas_data)
    check("se cargaron viewpoints", bool(vps), f"{len(vps)}")

    unknown = [v.key for v in vps if v.surface_type == "unknown"]
    check("todos los muebles tienen tipo", not unknown, f"sin clasificar: {unknown}" if unknown else "")

    with_waypoints = load_viewpoints(areas_data, include_waypoints=True)
    skipped = [v.key for v in with_waypoints if v.surface_type == "waypoint"]
    check(
        "los waypoints (house_entry y similares) no se patrullan",
        all(v.surface_type != "waypoint" for v in vps),
        f"omitidos: {skipped}" if skipped else "ninguno en este mapa",
    )

    bad_pose = [v.key for v in vps if v.arm_pose not in {t[1] for t in SURFACE_TYPES.values()}]
    check("cada viewpoint tiene pose de brazo conocida", not bad_pose, str(bad_pose))

    check("clasificación por nombre", classify("dinner_table") == "surface", "dinner_table -> surface")
    check("los nombres específicos ganan", classify("bedside_table") == "low_surface")
    check("electrodomésticos", classify("washing_machine") == "appliance")
    check("contenedores de piso", classify("waste_basket") == "floor")

    # A _meta entry must override the name heuristic.
    candidates = [
        (a, n)
        for a, d in areas_data.items()
        if isinstance(d, dict) and d.get("polygon")
        for n, v in d.items()
        if n not in ("polygon", "safe_place")
        and not n.endswith("_meta")
        and isinstance(v, list)
        and len(v) >= 7
    ]
    if not candidates:
        warn("sin muebles etiquetados con pose completa", "se omite la prueba de _meta")
        return
    room, name = candidates[0]
    patched = json.loads(json.dumps(areas_data))
    patched[room][f"{name}_meta"] = {"type": "shelf", "dwell_s": 9.0, "arm_pose": "flat_stare"}
    override = next(v for v in load_viewpoints(patched) if v.key == f"{room}/{name}")
    check(
        "_meta sobrescribe la heurística",
        override.surface_type == "shelf" and override.dwell_s == 9.0
        and override.arm_pose == "flat_stare",
        f"{room}/{name} -> {override.surface_type}, {override.dwell_s}s, {override.arm_pose}",
    )
    check(
        "las llaves _meta no se vuelven viewpoints",
        all(not v.name.endswith("_meta") for v in load_viewpoints(patched)),
    )


def test_route(areas_data: dict) -> None:
    print("\nroute — orden del patrullaje")
    vps = load_viewpoints(areas_data)
    start = tuple(areas_data.get("start_area", {}).get("safe_place", [0.0, 0.0])[:2])

    naive = route_length(vps, start)
    ordered, total = order_route(vps, start)
    check("el orden no pierde viewpoints", len(ordered) == len(vps), f"{len(ordered)}")

    if len(vps) < 3:
        warn("mapa con muy pocos muebles", f"{len(vps)} viewpoints; se omite el resto de la ruta")
        return

    check("ordenar acorta la ruta", total <= naive, f"{naive:.1f} m -> {total:.1f} m")

    grouped_runs = sum(1 for a, b in zip(ordered, ordered[1:]) if a.area != b.area)
    check(
        "cada cuarto se termina antes de cambiar",
        grouped_runs == len({v.area for v in vps}) - 1,
        f"{grouped_runs} cambios de cuarto para {len({v.area for v in vps})} cuartos",
    )

    again, total_again = order_route(vps, start)
    check(
        "el orden es determinista",
        [v.key for v in again] == [v.key for v in ordered] and total_again == total,
    )

    now = time.time()
    seen = {ordered[0].key: now - 5.0, ordered[1].key: now - 900.0}
    revisit, _ = order_by_staleness(vps, seen, now, start)
    check(
        "revisit pone lo nunca visto primero y lo reciente al final",
        revisit[-1].key == ordered[0].key and ordered[1].key in [v.key for v in revisit[:-1]],
        f"último = {revisit[-1].key}",
    )
    check("revisit conserva todos", len(revisit) == len(vps))


def test_staleness(areas_data: dict) -> None:
    print("\nstaleness — registro de escaneos")
    vps = load_viewpoints(areas_data)
    if not vps:
        warn("mapa sin muebles etiquetados", "se omiten las pruebas de staleness")
        return
    target = vps[0]
    directory = tempfile.mkdtemp(prefix="semantic_nav_selftest_")
    log = ScanLog(map_name="selftest", path=os.path.join(directory, "staleness.json"))
    now = 1000.0

    marked = log.mark_if_at(vps, target.x, target.y, target.yaw, now, 1.0, math.radians(35))
    check("una parada marca exactamente una superficie", marked == [target.key], str(marked))

    check(
        "con el heading equivocado no marca",
        log.mark_if_at(vps, target.x, target.y, target.yaw + math.pi / 2, now, 1.0,
                       math.radians(35)) == [],
    )
    check(
        "lejos no marca",
        log.mark_if_at(vps, target.x + 5.0, target.y, target.yaw, now, 1.0, math.radians(35)) == [],
    )
    check(
        "lo no escaneado es infinitamente viejo",
        log.age("cuarto_inexistente/mueble", now) == float("inf"),
    )

    wall = 5_000_000.0
    log.save(now, wall_now=wall)
    check("el snapshot se escribió", os.path.exists(log.path))

    # Restart: ROS clock back to 0, ten minutes of wall time gone by.
    reloaded = ScanLog(map_name="selftest", path=log.path)
    restored, why = reloaded.load(now=0.0, wall_now=wall + 600.0)
    check("el snapshot se recarga", restored == 1, why or "ok")
    age = reloaded.age(target.key, 0.0)
    check(
        "la edad sobrevive el reinicio como tiempo real",
        595.0 <= age <= 605.0,
        f"{age:.0f}s (esperado ~600)",
    )

    other = ScanLog(map_name="otro_mapa", path=log.path)
    check("un snapshot de otro mapa se rechaza", other.load(0.0)[0] == 0, other.load(0.0)[1])


def test_viewpoints(areas_data: dict, maps_dir: str, map_name: str) -> None:
    print("\nviewpoints — validación contra el costmap")
    pgm = os.path.join(maps_dir, f"{map_name}.pgm")
    yaml_path = os.path.join(maps_dir, f"{map_name}.yaml")
    if not (os.path.exists(pgm) and os.path.exists(yaml_path)):
        check(f"mapa {map_name}.pgm disponible", False, "sin grid, se omiten las pruebas")
        return

    grid = grid_from_pgm(pgm, load_map_yaml(yaml_path))
    check(
        "el grid se carga",
        grid.width > 0 and grid.height > 0 and len(grid.data) == grid.width * grid.height,
        f"{grid.width}x{grid.height} @ {grid.resolution} m",
    )

    vps = load_viewpoints(areas_data)
    if not vps:
        warn("mapa sin muebles etiquetados", "se omiten las pruebas de costmap")
        return
    blocked = [v.key for v in vps if not is_area_free(grid, v.x, v.y, 0.25, 50, True)]
    if blocked:
        # Exactly what the costmap check exists for: the map moved under the tags.
        warn(
            "poses etiquetadas bloqueadas en el mapa guardado",
            f"{blocked} — se reubican solas, pero conviene reetiquetarlas",
        )
    else:
        check("las poses etiquetadas están libres en el mapa guardado", True, f"{len(vps)}/{len(vps)}")

    started = time.time()
    report = validate_viewpoints(vps, grid)
    elapsed_ms = (time.time() - started) * 1000
    check(
        "la validación no pierde viewpoints recuperables",
        len(report.kept) + len(report.dropped) == len(vps) and len(report.kept) >= len(vps) - len(blocked),
        report.summary,
    )
    check("la validación es barata", elapsed_ms < 250, f"{elapsed_ms:.0f} ms")

    check("sin grid no se descarta nada", len(validate_viewpoints(vps, None).kept) == len(vps))

    # Block the cells around one viewpoint and confirm it is moved, not dropped.
    victim = next((v for v in vps if v.key not in blocked), vps[0])
    data = list(grid.data)
    blocked_cells = 0
    reach = int(0.45 / grid.resolution)
    cell = grid.cell_of(victim.x, victim.y)
    if cell:
        col0, row0 = cell
        for drow in range(-reach, reach + 1):
            for dcol in range(-reach, reach + 1):
                col, row = col0 + dcol, row0 + drow
                if 0 <= col < grid.width and 0 <= row < grid.height:
                    data[row * grid.width + col] = 100
                    blocked_cells += 1
    damaged = Grid(data, grid.width, grid.height, grid.resolution, grid.origin_x, grid.origin_y)
    check("el escenario bloquea al viewpoint", blocked_cells > 0 and not is_area_free(
        damaged, victim.x, victim.y, 0.25, 50, True))

    moved = relocate(victim, damaged)
    ok = moved is not None
    check("un viewpoint bloqueado se reubica", ok, victim.key)
    if ok:
        shift = math.hypot(moved.x - victim.x, moved.y - victim.y)
        check("la reubicación es libre", is_area_free(damaged, moved.x, moved.y, 0.25, 50, False))
        check("la reubicación se queda cerca", shift <= 2.0, f"{shift:.2f} m")
        tx, ty = target_of(victim)
        expected = math.atan2(ty - moved.y, tx - moved.x)
        error = abs(math.atan2(math.sin(moved.yaw - expected), math.cos(moved.yaw - expected)))
        check("la reubicación sigue mirando al mueble", error < math.radians(1.0),
              f"{math.degrees(error):.2f}°")

    report = validate_viewpoints(vps, damaged)
    check(
        "el reporte contabiliza la reubicación",
        len(report.kept) == len(vps) and len(report.relocated) >= 1,
        report.summary,
    )

    walled = Grid([100] * len(grid.data), grid.width, grid.height, grid.resolution,
                  grid.origin_x, grid.origin_y)
    report = validate_viewpoints(vps, walled)
    check("con todo bloqueado se descartan (no se inventan)", not report.kept,
          f"{len(report.dropped)} descartados")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", default="robocup2026_1", help="map name under map_context/maps")
    parser.add_argument("--maps-dir", default="", help="override the maps directory")
    args = parser.parse_args()

    maps_dir = args.maps_dir
    if not maps_dir:
        repo = os.path.dirname(os.path.dirname(os.path.dirname(_PKG_ROOT)))
        maps_dir = os.path.join(repo, "navigation", "packages", "map_context", "maps")
        if not os.path.isdir(maps_dir):
            try:
                from ament_index_python.packages import get_package_share_directory

                maps_dir = os.path.join(get_package_share_directory("map_context"), "maps")
            except Exception:
                pass

    areas_path = os.path.join(maps_dir, "areas", f"areas_{args.map}.json")
    if not os.path.exists(areas_path):
        print(f"no se encontró {areas_path}")
        return 2

    with open(areas_path) as handle:
        areas_data = json.load(handle)

    print(f"semantic nav selftest — mapa '{args.map}'\n{areas_path}")
    test_areas(areas_data)
    test_surfaces(areas_data)
    test_route(areas_data)
    test_staleness(areas_data)
    test_viewpoints(areas_data, maps_dir, args.map)

    print(f"\n{PASSED} pasaron, {FAILED} fallaron")
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
