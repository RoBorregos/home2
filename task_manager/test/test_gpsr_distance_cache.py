"""Distance-cache unit checks runnable without ROS: python3 -m unittest discover
-s task_manager/test -p 'test_gpsr_distance_cache.py'.

Load the production methods with lightweight message/transport doubles, avoiding
hardware imports and node startup. ROS builds and Gazebo remain integration checks.
"""

import ast
import importlib.util
import json
import math
import os
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[2]
_spec = importlib.util.spec_from_file_location(
    "path_cache_store", ROOT / "frida_constants/frida_constants/path_cache_store.py"
)
STORE = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(STORE)
AREAS = [("living_room", "sofa"), ("kitchen", "table")]
LIVE = {
    "living_room": {"sofa": [0, 0, 0, 0, 0, 0, 1]},
    "kitchen": {"table": [4, 0, 0, 0, 0, 0, 1]},
}
Status = NS(EXECUTION_SUCCESS=1, EXECUTION_ERROR=0)


def pose(x=0.0, y=0.0, yaw=0.0):
    return NS(
        header=NS(frame_id="map", stamp=0),
        pose=NS(
            position=NS(x=x, y=y, z=0.0),
            orientation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
        ),
    )


class Goal:
    FULL = 0
    LOCATION_PAIRS = 1
    ROBOT_ORIGIN = 2

    def __init__(self):
        self.mode = self.FULL
        self.robot_start = pose()
        self.robot_start.header.frame_id = ""
        self.budget_seconds = 0.0
        self.locations = [a[0] for a in AREAS]
        self.sublocations = [a[1] for a in AREAS]


def load_methods(path, class_name, names, **extra):
    """Execute unchanged method bodies, excluding ROS imports/decorators."""
    tree = ast.parse((ROOT / path).read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    cls.bases = []
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    for method in cls.body:
        method.decorator_list = []
    namespace = dict(
        math=math,
        time=time,
        t=time,
        json=json,
        SimpleNamespace=NS,
        Status=Status,
        PoseStamped=pose,
        NavQueryMatrix=NS(Goal=Goal, Result=NS, Feedback=NS),
        TIMEOUT_NAV_QUERY_MATRIX=8.0,
        DETOUR_FACTOR=1.4,
        os=os,
        NAV_MAPS_DIR="/nonexistent/maps",
        NAV_PATH_CACHE_DIR="/nonexistent/maps/path_cache",
        path_cache_file=STORE.path_cache_file,
        map_files_changed_since=STORE.map_files_changed_since,
        PATH_CACHE_SAVE_PERIOD=20.0,
        load_path_cache=STORE.load_path_cache,
        save_path_cache=STORE.save_path_cache,
        path_cache_digest=STORE.path_cache_digest,
    )
    namespace.update(extra)
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(ROOT / path), "exec"), namespace)
    return namespace[class_name]


GPSR = load_methods(
    "task_manager/scripts/gpsr_task_manager.py",
    "GPSRTM",
    {
        "_query_paths",
        "_get_areas",
        "_map_areas_file",
        "_saved_paths",
        "_saved_batch_pairs",
        "_load_batch_path_costs",
        "_discard_robot_paths",
        "_origin_matches",
        "_refresh_robot_paths",
        "_path_cost",
        "run",
        "_execute_interleaved_batch",
    },
    rclpy=NS(ok=lambda: True, spin_once=Mock()),
)
GPSR.TaskStates = NS(
    **{
        s: s
        for s in (
            "WAITING_FOR_BUTTON",
            "START",
            "WAIT_BUTTON_COMMAND",
            "WAITING_FOR_COMMAND",
            "PLAN_AND_EXECUTE_BATCH",
            "EXECUTING_COMMAND",
            "FINISHED_COMMAND",
            "DONE",
        )
    }
)
SERVER = load_methods(
    "navigation/packages/nav_main/scripts/nav_central.py",
    "Nav_Central",
    {
        "_run_path_matrix",
        "_plan_matrix",
        "_sync_path_cache",
        "_restore_path_cache",
        "_persist_path_cache",
        "_refresh_path_cache",
        "_matrix_needs_planner",
        "_setup",
        "_path_cache_inputs_ready",
        "_path_planner_ready",
        "cache_pairs_callback",
        "_named_area_list",
    },
    GetRobotPose=NS(Response=NS),
    GetState=NS(Request=NS),
    State=NS(PRIMARY_STATE_ACTIVE=3),
    ReentrantCallbackGroup=NS,
    MutuallyExclusiveCallbackGroup=NS,
    ManageLifecycleNodes=NS,
    NAV2_LIFECYCLE_SERVICE="/lifecycle_manager_navigation/manage_nodes",
    MONITOR_RATE=1.0,
)
CLIENT_ROS = NS(spin_until_future_complete=Mock())
CLIENT = load_methods(
    "task_manager/task_manager/subtask_managers/nav_tasks.py",
    "NavigationTasks",
    {
        "get_path_matrix",
    },
    rclpy=CLIENT_ROS,
    CLog=NS(nav=Mock()),
    GoalStatus=NS(STATUS_CANCELED=5),
    SUBTASK_MANAGER=NS(
        SERVICE_TIMEOUT=NS(value=2),
        NAV_QUERY_MATRIX_TIMEOUT=NS(value=12),
        NAV_QUERY_MATRIX_CANCEL_TIMEOUT=NS(value=95),
    ),
)


# One batch visiting both test areas
BOTH = (NS(commands=[NS(action="go_to", location_to_go=n) for n in ("sofa", "table")]),)


def disk(tmp):
    """Point GPSR's maps dir and path cache dir at ``tmp``."""
    return patch.dict(
        GPSR._saved_paths.__globals__,
        {"NAV_MAPS_DIR": tmp, "NAV_PATH_CACHE_DIR": f"{tmp}/path_cache"},
    )


def write_areas(tmp, areas, map_name="arena"):
    os.makedirs(f"{tmp}/areas", exist_ok=True)
    Path(f"{tmp}/areas/areas_{map_name}.json").write_text(json.dumps(areas))


def gpsr():
    tm = GPSR()
    tm._path_cache = {}
    tm._location_cache = {"obsolete": (1, 2)}
    tm._area_cache = {"obsolete": AREAS[0]}
    tm._areas = LIVE
    tm._origin_pose = None
    tm._plan_origin = None
    tm._resolve_area = lambda name: {"sofa": AREAS[0], "table": AREAS[1]}.get(name)
    tm.get_logger = Mock(return_value=Mock())
    parameters = {
        "path_origin_translation_tolerance": 0.1,
        "path_origin_rotation_tolerance": math.radians(10),
        "map_name": "arena",
    }
    tm.get_parameter = lambda key: NS(value=parameters[key])
    tm.subtask_manager = NS(
        nav=NS(
            retrieve_areas=Mock(return_value=(1, LIVE)), get_path_matrix=Mock(return_value=(1, {}))
        )
    )
    tm._current_robot_pose = Mock(return_value=pose())
    return tm


def grid():
    return NS(
        header=NS(frame_id="map"), info=NS(width=2, height=2, resolution=0.05), data=[0, 0, 100, -1]
    )


def server(mode=Goal.FULL, *, paused=True):
    nav = SERVER()
    nav.mapping = False
    nav.areas_data = LIVE
    nav.map_name = "arena"
    nav.areas_map_name = "arena"
    nav.config_path = "localization.yaml"
    nav._location_path_cache = {}
    nav._path_cache_context = None
    nav._path_cache_warm_solved = -1
    nav._path_cache_warm_report = None
    nav._path_cache_file = ""
    nav._map_loaded_at = time.time()
    nav.use_nav2 = True
    nav._static_grid = grid()
    nav._costmap_grid = grid()
    nav.compute_path_client = Mock()
    nav.compute_path_client.server_is_ready.return_value = True
    nav._path_state_clients = []
    for _ in range(2):
        client = Mock()
        client.service_is_ready.return_value = True
        future = Mock()
        future.done.return_value = True
        future.result.return_value = NS(current_state=NS(id=3))
        client.call_async.return_value = future
        nav._path_state_clients.append(client)
    nav.get_parameter = lambda _: NS(value=120.0)
    nav.rtabmap_loaded = True
    nav.nodes_status = True
    nav.nav2_paused = paused
    nav.nav_logger = Mock()
    nav._fetch_area_coords = lambda loc, sub: LIVE.get(loc, {}).get(sub)
    nav._pose_from_coords = lambda coords: pose(coords[0], coords[1])
    nav._hold_service_group = Mock(return_value=True)
    nav._release_service_group = Mock()
    nav.resume_slam = Mock()
    nav.pause_slam = Mock()
    nav.resume_nav2 = Mock(side_effect=lambda: setattr(nav, "nav2_paused", False))
    nav.pause_nav2 = Mock(side_effect=lambda: setattr(nav, "nav2_paused", True))
    # nav_central's _set_paused(paused) pauses/resumes SLAM, then nav2
    nav._set_paused = Mock(
        side_effect=lambda paused: (
            (nav.pause_slam(), nav.pause_nav2())
            if paused
            else (nav.resume_slam(), nav.resume_nav2())
        )
    )
    nav.get_robot_pose_callback = Mock(return_value=NS(success=True, pose=pose(10)))
    nav._compute_path = Mock(
        side_effect=lambda goal, start, *_: abs(goal.pose.position.x - start.pose.position.x)
    )
    nav._path_length = lambda path: path
    request = Goal()
    request.mode = mode
    handle = NS(request=request, is_cancel_requested=False, publish_feedback=Mock())
    return nav, handle


def run_batch(tm):
    """Run _execute_interleaved_batch with the merger and behaviour tree stubbed."""
    tm._completed = set()
    tm._spoken_merged_plan = Mock(return_value="Visit the kitchen")
    tm.subtask_manager.hri = Mock()
    tm.gpsr_tasks = tm.gpsr_individual_tasks = NS()
    tm._on_action_start = tm._on_action_complete = Mock()
    plan = NS(actions=[], fallback=[])
    root = NS(status="SUCCESS")
    tree = NS(setup=Mock(), tick=Mock())
    merge = Mock(return_value=plan)
    with patch.dict(
        GPSR._execute_interleaved_batch.__globals__,
        {
            "merge": merge,
            "build_tree": Mock(return_value=root),
            "render_tree_ascii": lambda _: "tree",
            "GLOBAL_BUDGET_S": 100,
            "ANSWER_PUBLISHER": "/answers",
            "GPSR_TASK_STEP_TOPIC": "/gpsr/step",
            "py_trees": NS(
                trees=NS(BehaviourTree=lambda _: tree),
                common=NS(Status=NS(SUCCESS="SUCCESS", FAILURE="FAILURE", INVALID="INVALID")),
            ),
        },
    ):
        tm._execute_interleaved_batch()
    return merge


class CacheTests(unittest.TestCase):
    def test_batch_reads_only_its_named_pairs_from_file_and_drops_old_origin(self):
        with tempfile.TemporaryDirectory() as tmp:
            tm = gpsr()
            tm._path_cache = {(AREAS[1], AREAS[0]): 99, (None, AREAS[0]): 88}
            tm._origin_pose = pose()
            saved = {(AREAS[0], AREAS[1]): 6, (AREAS[0], AREAS[0]): 0, (AREAS[1], ("x", "y")): 3}
            path = STORE.path_cache_file(f"{tmp}/path_cache", "arena")
            STORE.save_path_cache(path, "digest", LIVE, saved, map_name="arena")
            commands = [
                NS(
                    commands=[
                        NS(action="go_to", location_to_go=name)
                        for name in ("sofa", "table", "sofa")
                    ]
                )
            ]
            with disk(tmp):
                areas = tm._load_batch_path_costs(commands)
            self.assertEqual(areas, AREAS)
            self.assertEqual(tm._path_cache, {(AREAS[0], AREAS[1]): 6, (AREAS[0], AREAS[0]): 0})
            self.assertEqual(tm._location_cache, {})
            self.assertEqual(tm._area_cache, {})
            self.assertIsNone(tm._origin_pose)
            tm.subtask_manager.nav.get_path_matrix.assert_not_called()

    def test_batch_queries_robot_row_only_for_its_areas(self):
        tm = gpsr()
        names = {"sofa": AREAS[0], "table": AREAS[1], "bed": ("bedroom", "bed")}
        tm._resolve_area = names.get
        tm._areas = dict(LIVE, bedroom={"bed": [8, 0, 0, 0, 0, 0, 1]}, hall={"door": [9] * 7})
        tm.subtask_manager.nav.retrieve_areas.return_value = (1, tm._areas)
        tm.subtask_manager.nav.get_path_matrix.return_value = (
            1,
            {(None, AREAS[0]): 2, (None, ("bedroom", "bed")): 9},
        )
        tm.batched_commands = [
            NS(commands=[NS(action="go_to", location_to_go="sofa"), NS(action="say")]),
            NS(commands=[NS(action="go_to", location_to_go="bed")]),
            NS(commands=[NS(action="go_to", location_to_go="sofa")]),
        ]
        merge = run_batch(tm)
        nav = tm.subtask_manager.nav.get_path_matrix
        nav.assert_called_once()
        self.assertEqual(nav.call_args.args[0], [AREAS[0], ("bedroom", "bed")])
        self.assertEqual(nav.call_args.kwargs["mode"], Goal.ROBOT_ORIGIN)
        self.assertEqual(merge.call_args.kwargs["origin"], (0, 0))

    def test_no_destinations_does_not_query_but_keeps_straight_line_origin(self):
        tm = gpsr()
        tm._current_robot_pose.return_value = pose(3)
        tm._refresh_robot_paths(tm._load_batch_path_costs([NS(commands=[NS(action="say")])]))
        tm.subtask_manager.nav.get_path_matrix.assert_not_called()
        self.assertEqual(tm._path_cache, {})
        self.assertEqual(tm._plan_origin, (3, 0))

    def test_new_batch_drops_previous_batch_pairs(self):
        tm = gpsr()
        tm._path_cache = {(AREAS[0], AREAS[1]): 99}
        tm._load_batch_path_costs([NS(commands=[NS(action="go_to", location_to_go="sofa")])])
        self.assertEqual(tm._path_cache, {})

    def test_origin_is_the_pose_when_the_batch_is_planned(self):
        tm = gpsr()
        tm._origin_pose = pose(50)  # Old entrance origin.
        tm._path_cache = {(None, AREAS[0]): 50, (AREAS[0], AREAS[1]): 6}
        tm._current_robot_pose.return_value = pose(10)
        tm.subtask_manager.nav.get_path_matrix.return_value = (1, {(None, AREAS[0]): 10})
        tm._refresh_robot_paths([AREAS[0]])
        self.assertEqual(tm._plan_origin, (10, 0))
        self.assertEqual(tm._path_cache[(None, AREAS[0])], 10)
        self.assertEqual(tm._path_cache[(AREAS[0], AREAS[1])], 6)
        query = tm.subtask_manager.nav.get_path_matrix.call_args
        self.assertEqual(query.args[0], [AREAS[0]])
        self.assertEqual(query.kwargs["mode"], Goal.ROBOT_ORIGIN)
        self.assertEqual(query.kwargs["robot_start"].pose.position.x, 10)

    def test_yaw_wrap_counts_as_the_same_origin(self):
        tm = gpsr()
        tm._origin_pose = pose(yaw=math.pi - 0.01)
        self.assertTrue(tm._origin_matches(pose(0.01, yaw=-math.pi + 0.01)))

    def test_rotation_during_query_discards_origin(self):
        tm = gpsr()
        tm._current_robot_pose.side_effect = [pose(), pose(yaw=0.5)]
        tm.subtask_manager.nav.get_path_matrix.return_value = (1, {(None, AREAS[0]): 10})
        tm._refresh_robot_paths(AREAS)
        self.assertEqual(tm._path_cache, {})
        self.assertIsNone(tm._plan_origin)

    def test_movement_during_query_discards_origin_but_keeps_named_pairs(self):
        tm = gpsr()
        tm._path_cache = {(AREAS[0], AREAS[1]): 6}
        tm._current_robot_pose.side_effect = [pose(10), pose(12)]
        tm.subtask_manager.nav.get_path_matrix.return_value = (1, {(None, AREAS[0]): 10})
        tm._refresh_robot_paths(AREAS)
        self.assertEqual(tm._path_cache, {(AREAS[0], AREAS[1]): 6})
        self.assertIsNone(tm._plan_origin)

    def test_missing_tf_discards_stale_origin(self):
        tm = gpsr()
        tm._path_cache = {(None, AREAS[0]): 50}
        tm._origin_pose = pose(50)
        tm._current_robot_pose.return_value = None
        tm._refresh_robot_paths(AREAS)
        self.assertEqual(tm._path_cache, {})
        self.assertIsNone(tm._plan_origin)
        tm.subtask_manager.nav.get_path_matrix.assert_not_called()

    def test_origin_query_failure_uses_current_straight_line_fallback(self):
        tm = gpsr()
        tm._current_robot_pose.return_value = pose(10)
        tm.subtask_manager.nav.get_path_matrix.return_value = (0, "Planner unavailable")
        tm._refresh_robot_paths(AREAS)
        tm._resolve_area = lambda name: AREAS[0]
        tm._resolve_xy = lambda name: (0, 0)
        self.assertEqual(tm._path_cost(None, "sofa"), 14)

    def test_gpsr_startup_never_queries_paths(self):
        tm = gpsr()
        # An idle synthetic state exercises startup without HRI/navigation side effects.
        tm.current_state = "IDLE"
        for test_mode in (False, True):
            tm.interleave_enabled = True
            tm.test_mode = test_mode
            tm.run()
            tm.run()
        tm.subtask_manager.nav.get_path_matrix.assert_not_called()

    def test_start_location_does_not_query_the_robot_row(self):
        tm = gpsr()
        tm.interleave_enabled = True
        tm.test_mode = False
        tm.current_state = GPSR.TaskStates.START
        tm._track_state_change = Mock()
        tm.subtask_manager.nav.check_door = Mock()
        events = []
        tm.navigate_to = Mock(side_effect=lambda *_: events.append("arrive") or 1)
        tm.subtask_manager.hri = NS(say=lambda *_: events.append("announce"))
        tm.run()
        self.assertEqual(events, ["arrive", "announce"])
        tm.subtask_manager.nav.get_path_matrix.assert_not_called()
        self.assertEqual(tm.current_state, GPSR.TaskStates.WAIT_BUTTON_COMMAND)

    def saved_cache(self, tmp, areas=LIVE, *, map_name="arena", saved_at=None):
        path = STORE.path_cache_file(f"{tmp}/path_cache", map_name)
        pairs = {(AREAS[0], AREAS[1]): 6.5, (AREAS[1], AREAS[0]): 7.0, (AREAS[0], AREAS[0]): 0}
        STORE.save_path_cache(path, "digest", areas, pairs, map_name=map_name, saved_at=saved_at)
        return path

    def nav_down(self):
        tm = gpsr()
        tm._areas = None
        tm.subtask_manager.nav.retrieve_areas.return_value = (0, "down")
        tm.subtask_manager.nav.areas_backup = {"other": {"place": [0] * 7}}
        tm.subtask_manager.nav.get_path_matrix.return_value = (0, "Action server not started")
        return tm

    def test_navigation_down_checks_pairs_against_the_areas_file_on_disk(self):
        with tempfile.TemporaryDirectory() as tmp:
            tm = self.nav_down()
            write_areas(tmp, LIVE)
            self.saved_cache(tmp)
            with disk(tmp):
                tm._load_batch_path_costs(BOTH)
            self.assertEqual(tm._areas, LIVE)
            self.assertEqual(tm._path_cache[(AREAS[0], AREAS[1])], 6.5)
            self.assertEqual(tm._path_cost("table", "sofa"), 7.0)

    def test_navigation_down_retagged_location_drops_only_its_pairs(self):
        with tempfile.TemporaryDirectory() as tmp:
            tm = self.nav_down()
            retagged = json.loads(json.dumps(LIVE))
            retagged["kitchen"]["table"][0] = 9
            write_areas(tmp, retagged)
            self.saved_cache(tmp)  # Planned before the table moved
            with disk(tmp):
                tm._load_batch_path_costs(BOTH)
            self.assertEqual(tm._path_cache, {(AREAS[0], AREAS[0]): 0})

    def test_navigation_down_never_validates_the_cache_against_itself(self):
        with tempfile.TemporaryDirectory() as tmp:
            tm = self.nav_down()
            self.saved_cache(tmp)  # No areas file: falls back to the unrelated backup
            with disk(tmp):
                tm._load_batch_path_costs(BOTH)
            self.assertEqual(tm._areas, {"other": {"place": [0] * 7}})
            self.assertEqual(tm._path_cache, {})

    def test_map_files_newer_than_the_cache_make_it_stale(self):
        with tempfile.TemporaryDirectory() as tmp:
            tm = gpsr()
            self.saved_cache(tmp, saved_at=time.time() - 100)
            Path(f"{tmp}/arena.pgm").write_text("edited")
            with disk(tmp):
                tm._load_batch_path_costs(BOTH)
            self.assertEqual(tm._path_cache, {})
            warnings = [c.args[0] for c in tm.get_logger.return_value.warning.call_args_list]
            self.assertTrue(any("arena.pgm" in w and "--cache-pairs" in w for w in warnings))

    def test_map_files_older_than_the_cache_keep_it(self):
        with tempfile.TemporaryDirectory() as tmp:
            tm = gpsr()
            Path(f"{tmp}/arena.pgm").write_text("map")
            os.utime(f"{tmp}/arena.pgm", (time.time() - 100, time.time() - 100))
            self.saved_cache(tmp)
            with disk(tmp):
                tm._load_batch_path_costs(BOTH)
            self.assertEqual(tm._path_cache[(AREAS[0], AREAS[1])], 6.5)

    def test_cache_for_another_map_or_unknown_map_is_ignored(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = self.saved_cache(tmp, map_name="other")
            os.replace(path, STORE.path_cache_file(f"{tmp}/path_cache", "arena"))
            tm = gpsr()
            with disk(tmp):
                tm._load_batch_path_costs(BOTH)
            self.assertEqual(tm._path_cache, {})
            tm = gpsr()
            tm.get_parameter = lambda key: NS(value="" if key == "map_name" else 0.1)
            with disk(tmp):
                self.assertEqual(tm._saved_paths(), (None, {}))

    def test_saved_pairs_skip_locations_that_moved(self):
        with tempfile.TemporaryDirectory() as tmp:
            moved = json.loads(json.dumps(LIVE))
            moved["kitchen"]["table"][0] = 9
            tm = gpsr()
            tm.subtask_manager.nav.get_path_matrix.return_value = (0, "timeout")
            self.saved_cache(tmp, moved)
            with disk(tmp):
                tm._load_batch_path_costs(
                    [
                        NS(commands=[NS(action="go_to", location_to_go="sofa")]),
                        NS(commands=[NS(action="go_to", location_to_go="table")]),
                    ]
                )
            self.assertEqual(tm._path_cache, {(AREAS[0], AREAS[0]): 0})

    def test_missing_file_uses_straight_line_without_querying_navigation(self):
        tm = gpsr()
        tm._plan_origin = None
        tm._resolve_xy = {"sofa": (0, 0), "table": (4, 0)}.get
        tm._load_batch_path_costs(
            [NS(commands=[NS(action="go_to", location_to_go=n) for n in ("sofa", "table")])]
        )
        self.assertEqual(tm._path_cache, {})
        self.assertAlmostEqual(tm._path_cost("sofa", "table"), 1.4 * 4)
        tm.subtask_manager.nav.get_path_matrix.assert_not_called()

    def test_corrupt_or_invalid_cache_file_is_ignored(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "cache.json"
            path.write_text("{not json")
            self.assertIsNone(STORE.load_path_cache(str(path)))
            rows = [
                ["a", "b", "c", "d", float("nan")],
                ["a", "b", "c", "d", -1],
                ["a", "b", "c", "d", 3],
            ]
            data = {"version": 2, "digest": "x", "saved_at": 1.0, "areas": {}, "pairs": rows}
            path.write_text(json.dumps(data))
            self.assertEqual(STORE.load_path_cache(str(path))[2], {(("a", "b"), ("c", "d")): 3.0})
            # Old /tmp-era files (version 1, no saved_at) are not trusted
            path.write_text(json.dumps(dict(data, version=1)))
            self.assertIsNone(STORE.load_path_cache(str(path)))
            path.write_text(json.dumps({k: v for k, v in data.items() if k != "saved_at"}))
            self.assertIsNone(STORE.load_path_cache(str(path)))

    def test_cache_file_is_per_map_in_a_created_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = STORE.path_cache_file(f"{tmp}/new/path_cache", "arena")
            self.assertTrue(path.endswith("/new/path_cache/arena.json"))
            STORE.save_path_cache(path, "d", LIVE, {}, map_name="arena", saved_at=5.0)
            self.assertEqual(STORE.load_path_cache(path)[3:], (5.0, "arena"))

    def test_batch_cleanup_discards_its_snapshot_not_navigation_cache(self):
        tm = gpsr()
        tm.batched_commands = [NS(), NS(), NS()]
        tm._refresh_robot_paths = Mock()
        tm._load_batch_path_costs = Mock(return_value=[])

        def snapshot(_):
            tm._path_cache = {(AREAS[0], AREAS[1]): 6, (None, AREAS[0]): 10}
            tm._origin_pose = pose(10)
            tm._plan_origin = (10, 0)

        tm._refresh_robot_paths.side_effect = snapshot
        run_batch(tm)
        tm._refresh_robot_paths.assert_called_once_with([])
        self.assertEqual(tm._path_cache, {})
        self.assertIsNone(tm._areas)
        self.assertIsNone(tm._plan_origin)
        tm.subtask_manager.nav.get_path_matrix.assert_not_called()


class MatrixServerTests(unittest.TestCase):
    def test_warmup_waits_for_populated_map_and_costmap(self):
        for missing in ("map", "costmap", "unknown", "malformed", "wrong_frame", "locations"):
            with self.subTest(missing=missing):
                nav, _ = server()
                nav._location_path_cache[(AREAS[0], AREAS[1])] = 8
                if missing == "map":
                    nav._static_grid = None
                elif missing == "costmap":
                    nav._costmap_grid = None
                elif missing == "unknown":
                    nav._costmap_grid.data = [-1] * 4
                elif missing == "malformed":
                    nav._static_grid.data = [0]
                elif missing == "wrong_frame":
                    nav._static_grid.header.frame_id = "odom"
                else:
                    nav.areas_data = {}
                self.assertFalse(nav._refresh_path_cache())
                nav._hold_service_group.assert_not_called()
                nav._compute_path.assert_not_called()
                self.assertEqual(nav._location_path_cache[(AREAS[0], AREAS[1])], 8)

    def test_action_discovery_does_not_replace_active_lifecycle_check(self):
        for client_index in range(2):
            nav, _ = server()
            nav._sync_path_cache()
            nav._location_path_cache[(AREAS[0], AREAS[1])] = 8
            nav._path_state_clients[
                client_index
            ].call_async.return_value.result.return_value.current_state.id = 2
            self.assertFalse(nav._refresh_path_cache())
            nav._compute_path.assert_not_called()
            self.assertEqual(nav._location_path_cache[(AREAS[0], AREAS[1])], 8)
            self.assertTrue(nav.nav2_paused)

    def test_cache_pairs_service_refuses_until_ready_then_warms(self):
        nav, _ = server()
        nav._static_grid = None
        response = nav.cache_pairs_callback(None, NS())
        self.assertFalse(response.success)
        self.assertIn("not ready", response.message)
        nav._compute_path.assert_not_called()
        nav._static_grid = grid()
        response = nav.cache_pairs_callback(None, NS())
        self.assertTrue(response.success)
        self.assertIn("Cached 4/4 named pairs", response.message)
        self.assertIn("(complete)", response.message)
        self.assertEqual(len(nav._location_path_cache), 4)
        self.assertTrue(nav.nav2_paused)

    def test_cache_pairs_service_rebuilds_stale_pairs(self):
        nav, _ = server()
        nav._sync_path_cache()
        nav._location_path_cache[(AREAS[0], AREAS[1])] = 99
        self.assertTrue(nav.cache_pairs_callback(None, NS()).success)
        self.assertEqual(nav._location_path_cache[(AREAS[0], AREAS[1])], 4)

    def test_cache_pairs_service_retries_budget_cut_rounds_and_reports_incomplete(self):
        nav, _ = server()
        nav.get_parameter = lambda _: NS(value=0.05)
        # (delay, path): round 1 ends past the budget, round 2 too but adds nothing
        steps = iter([(0.03, None), (0.03, 4), (0.06, None)])

        def planner(*_):
            delay, path = next(steps)
            time.sleep(delay)
            return path

        nav._compute_path.side_effect = planner
        response = nav.cache_pairs_callback(None, NS())
        self.assertFalse(response.success)
        self.assertIn("Cached 3/4 named pairs", response.message)
        self.assertIn("incomplete: 1 unsolved", response.message)

    def test_cache_pairs_service_reports_planner_failure(self):
        nav, _ = server()
        nav.compute_path_client.server_is_ready.return_value = False
        response = nav.cache_pairs_callback(None, NS())
        self.assertFalse(response.success)
        self.assertIn("warmup failed", response.message)
        self.assertIn("planner not ready", response.message)

    def test_named_area_list_excludes_metadata(self):
        nav, _ = server()
        nav.areas_data = dict(
            LIVE,
            metadata="arena",
            polygons={"bounds": [[0, 0], [1, 1]]},
            invalid={"pose": [float("nan")] * 7},
        )
        self.assertEqual(nav._named_area_list(), AREAS)

    def test_readiness_timeout_cancels_pending_state_requests(self):
        nav, _ = server()
        future = nav._path_state_clients[0].call_async.return_value
        future.done.return_value = False
        self.assertFalse(nav._path_planner_ready(time.monotonic() - 1, lambda: False))
        future.cancel.assert_called_once()

    def test_missing_state_service_or_planner_action_defers_warmup(self):
        for unavailable in ("state", "action"):
            nav, _ = server()
            if unavailable == "state":
                nav._path_state_clients[0].service_is_ready.return_value = False
            else:
                nav.compute_path_client.server_is_ready.return_value = False
            self.assertFalse(nav._refresh_path_cache())
            nav._compute_path.assert_not_called()

    def test_setup_restores_and_reports_cache_without_warming(self):
        nav, _ = server()
        nav._setup_done = False
        nav._setup_timer = object()
        nav.destroy_timer = Mock()
        nav.create_client = Mock()
        nav.create_timer = Mock()
        nav.get_publishers_info_by_topic = Mock(return_value=[])
        nav._monitoring = Mock()
        events = []
        nav.load_map_areas = lambda: events.append("areas")
        nav.wait_for_requirements = lambda: events.append("requirements")
        nav.start_slam = lambda: events.append("slam")
        nav.load_nav2 = lambda: events.append("nav2")
        nav._wait_for_initial_pose = lambda: events.append("pose")
        nav._refresh_path_cache = Mock()
        nav._setup()
        nav._setup()
        self.assertEqual(events, ["areas", "requirements", "slam", "nav2", "pose"])
        self.assertTrue(nav.nodes_status)
        nav._refresh_path_cache.assert_not_called()
        nav._compute_path.assert_not_called()
        self.assertIsNotNone(nav._path_cache_context)
        self.assertTrue(any("--cache-pairs" in c.args[1] for c in nav.nav_logger.call_args_list))

    def test_navigation_warms_without_a_gpsr_process(self):
        nav, _ = server()
        nav._refresh_path_cache()
        self.assertEqual(len(nav._location_path_cache), 4)
        self.assertTrue(all(start is not None for start, _ in nav._location_path_cache))
        self.assertEqual(nav._compute_path.call_count, 2)
        self.assertTrue(nav.nav2_paused)

    def test_mapping_and_uninitialized_navigation_do_not_warm(self):
        for field, value in (("mapping", True), ("use_nav2", False), ("nodes_status", False)):
            nav, _ = server()
            setattr(nav, field, value)
            nav._refresh_path_cache()
            self.assertFalse(nav.cache_pairs_callback(None, NS()).success)
            nav._hold_service_group.assert_not_called()

    def test_warmed_queries_reuse_directed_pairs_in_any_request_order(self):
        nav, handle = server(Goal.LOCATION_PAIRS)
        nav._compute_path.side_effect = lambda goal, start, *_: (
            6 if start.pose.position.x == 0 else 9
        )
        nav._refresh_path_cache()
        nav._compute_path.reset_mock()
        nav.resume_nav2.reset_mock()
        nav.pause_nav2.reset_mock()
        handle.request.locations.reverse()
        handle.request.sublocations.reverse()
        self.assertEqual(nav._run_path_matrix(handle).distances, [0, 9, 6, 0])
        self.assertEqual(nav._run_path_matrix(handle).distances, [0, 9, 6, 0])
        nav._compute_path.assert_not_called()
        nav.resume_nav2.assert_not_called()
        nav.pause_nav2.assert_not_called()

    def test_partial_cache_only_recomputes_missing_pairs(self):
        nav, handle = server(Goal.LOCATION_PAIRS)
        nav._compute_path.side_effect = [None, 4]
        nav._refresh_path_cache()
        self.assertNotIn((AREAS[0], AREAS[1]), nav._location_path_cache)
        nav._compute_path.reset_mock()
        nav._compute_path.side_effect = lambda *_: 6
        self.assertEqual(nav._run_path_matrix(handle).distances, [0, 6, 4, 0])
        nav._compute_path.assert_called_once()

    def test_warmed_named_pairs_are_available_even_when_planning_budget_expires(self):
        nav, handle = server()
        nav._refresh_path_cache()
        nav._compute_path.reset_mock()
        self.assertEqual(
            nav._plan_matrix(handle, [pose(), pose(4)], 0, pose(10)), [-1, -1, 0, 4, 4, 0]
        )
        nav._compute_path.assert_not_called()

    def test_warmup_failure_does_not_prevent_navigation_starting(self):
        nav, _ = server()
        nav._compute_path.side_effect = RuntimeError("planner failed")
        nav._refresh_path_cache()
        nav.pause_nav2.assert_called_once()
        nav._release_service_group.assert_called_once()
        self.assertTrue(any("warmup failed" in c.args[1] for c in nav.nav_logger.call_args_list))

    def test_warmup_saves_pairs_and_restart_restores_without_planning(self):
        with tempfile.TemporaryDirectory() as tmp:
            nav, handle = server(Goal.LOCATION_PAIRS)
            nav._path_cache_file = f"{tmp}/cache.json"
            self.assertTrue(nav._refresh_path_cache(full=False))
            warmed = dict(nav._location_path_cache)
            restarted, handle = server(Goal.LOCATION_PAIRS)
            restarted._path_cache_file = nav._path_cache_file
            self.assertTrue(restarted._refresh_path_cache(full=False))
            self.assertEqual(restarted._location_path_cache, warmed)
            restarted._compute_path.assert_not_called()
            restarted.resume_nav2.assert_not_called()
            self.assertEqual(restarted._run_path_matrix(handle).distances, [0, 4, 4, 0])

    def test_restart_ignores_cache_older_than_the_map_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            nav, _ = server()
            nav._path_cache_file = f"{tmp}/cache.json"
            nav._map_loaded_at = time.time() - 100
            self.assertTrue(nav._refresh_path_cache(full=False))
            Path(f"{tmp}/arena.posegraph").write_text("re-mapped")
            restarted, _ = server()
            restarted._path_cache_file = nav._path_cache_file
            with patch.dict(SERVER._restore_path_cache.__globals__, {"NAV_MAPS_DIR": tmp}):
                restarted._sync_path_cache()
            self.assertEqual(restarted._location_path_cache, {})
            self.assertEqual(STORE.load_path_cache(restarted._path_cache_file)[2], {})

    def test_saved_pairs_for_other_areas_are_not_restored(self):
        with tempfile.TemporaryDirectory() as tmp:
            nav, _ = server()
            nav._path_cache_file = f"{tmp}/cache.json"
            nav._refresh_path_cache(full=False)
            moved, _ = server()
            moved._path_cache_file = nav._path_cache_file
            moved.areas_data = json.loads(json.dumps(LIVE))
            moved.areas_data["kitchen"]["table"][0] = 8
            moved._fetch_area_coords = lambda loc, sub: moved.areas_data.get(loc, {}).get(sub)
            moved._refresh_path_cache(full=False)
            self.assertEqual(moved._compute_path.call_count, 2)
            self.assertEqual(moved._location_path_cache[(AREAS[0], AREAS[1])], 8)

    def test_remap_rewrites_cache_file_before_planning(self):
        with tempfile.TemporaryDirectory() as tmp:
            nav, _ = server()
            nav._path_cache_file = f"{tmp}/cache.json"
            nav._refresh_path_cache(full=False)
            moved, _ = server()
            moved._path_cache_file = nav._path_cache_file
            moved.areas_data = json.loads(json.dumps(LIVE))
            moved.areas_data["kitchen"]["table"][0] = 8
            moved._sync_path_cache()
            _, areas, pairs, _, _ = STORE.load_path_cache(moved._path_cache_file)
            self.assertEqual(areas["kitchen"]["table"][0], 8)
            self.assertEqual(pairs, {})

    def test_warmup_plans_on_costmap_as_is(self):
        # The costmap is cleared before --cache-pairs, so warmup must not touch it
        nav, _ = server()
        nav._call_service_with_timeout = Mock(return_value=True)
        self.assertTrue(nav._refresh_path_cache(full=True))
        nav._call_service_with_timeout.assert_not_called()
        self.assertEqual(len(nav._location_path_cache), len(nav._named_area_list()) ** 2)

    def test_unwritable_cache_file_does_not_fail_warmup(self):
        with tempfile.NamedTemporaryFile() as blocker:
            nav, _ = server()
            # Under a regular file, so even root cannot create the directory
            nav._path_cache_file = f"{blocker.name}/dir/cache.json"
            self.assertTrue(nav._refresh_path_cache(full=False))
        self.assertTrue(any("Could not save" in c.args[1] for c in nav.nav_logger.call_args_list))

    def test_budget_cut_warmup_retries_until_no_new_pairs(self):
        nav, _ = server()
        nav.get_parameter = lambda _: NS(value=0.05)
        # (delay, path): round 1 ends past the budget, round 2 too but adds nothing
        steps = iter([(0.03, None), (0.03, 4), (0.06, None)])

        def planner(*_):
            delay, path = next(steps)
            time.sleep(delay)
            return path

        nav._compute_path.side_effect = planner
        self.assertFalse(nav._refresh_path_cache(full=False))
        self.assertIn((AREAS[1], AREAS[0]), nav._location_path_cache)
        self.assertTrue(nav._refresh_path_cache(full=False))
        self.assertNotIn((AREAS[0], AREAS[1]), nav._location_path_cache)

    def test_full_refresh_removes_previously_solved_but_now_unsolved_pairs(self):
        nav, _ = server()
        nav._refresh_path_cache()
        self.assertIn((AREAS[0], AREAS[1]), nav._location_path_cache)
        nav._compute_path.side_effect = lambda *_: None
        nav._refresh_path_cache()
        self.assertEqual(
            nav._location_path_cache, {(AREAS[0], AREAS[0]): 0, (AREAS[1], AREAS[1]): 0}
        )

    def test_reloaded_locations_or_map_identity_invalidate_cache(self):
        for changed in ("locations", "map"):
            with self.subTest(changed=changed):
                nav, handle = server(Goal.LOCATION_PAIRS)
                nav._refresh_path_cache()
                nav._compute_path.reset_mock()
                if changed == "locations":
                    nav.areas_data = json.loads(json.dumps(LIVE))
                    nav.areas_data["kitchen"]["table"][0] = 8
                    nav._fetch_area_coords = lambda loc, sub: nav.areas_data.get(loc, {}).get(sub)
                else:
                    nav.map_name = "new_arena"
                nav._run_path_matrix(handle)
                self.assertEqual(nav._compute_path.call_count, 2)

    def test_robot_rows_are_fresh_and_never_enter_navigation_cache(self):
        nav, handle = server(Goal.ROBOT_ORIGIN)
        nav._refresh_path_cache()
        baseline = dict(nav._location_path_cache)
        handle.request.robot_start = pose(10)
        self.assertEqual(nav._run_path_matrix(handle).distances, [10, 6])
        handle.request.robot_start = pose(20)
        self.assertEqual(nav._run_path_matrix(handle).distances, [20, 16])
        self.assertEqual(nav._location_path_cache, baseline)

    def test_modes_shapes_and_ordered_pairs(self):
        for mode, expected in (
            (Goal.FULL, [10, 6, 0, 4, 4, 0]),
            (Goal.LOCATION_PAIRS, [0, 4, 4, 0]),
            (Goal.ROBOT_ORIGIN, [10, 6]),
        ):
            with self.subTest(mode=mode):
                nav, handle = server(mode)
                result = nav._run_path_matrix(handle)
                self.assertTrue(result.success)
                self.assertEqual(result.distances, expected)
                self.assertEqual(
                    handle.publish_feedback.call_args.args[0].pairs_total, len(expected)
                )
                nav._release_service_group.assert_called_once()
                self.assertTrue(nav.nav2_paused)

    def test_named_pairs_do_not_need_robot_tf(self):
        nav, handle = server(Goal.LOCATION_PAIRS)
        nav._run_path_matrix(handle)
        nav.get_robot_pose_callback.assert_not_called()
        self.assertEqual(nav._compute_path.call_count, 2)

    def test_explicit_robot_snapshot_is_used_for_every_goal(self):
        nav, handle = server(Goal.ROBOT_ORIGIN)
        handle.request.robot_start = pose(20)
        result = nav._run_path_matrix(handle)
        self.assertEqual(result.distances, [20, 16])
        nav.get_robot_pose_callback.assert_not_called()
        self.assertTrue(
            all(c.args[1] is handle.request.robot_start for c in nav._compute_path.call_args_list)
        )

    def test_implicit_robot_pose_is_captured_once(self):
        nav, handle = server()
        nav._run_path_matrix(handle)
        nav.get_robot_pose_callback.assert_called_once()

    def test_missing_tf_still_returns_named_pairs(self):
        nav, handle = server()
        nav.get_robot_pose_callback.return_value = NS(success=False)
        self.assertEqual(nav._run_path_matrix(handle).distances, [-1, -1, 0, 4, 4, 0])

    def test_cancel_preserves_solved_pairs_and_restores_navigation(self):
        nav, handle = server(Goal.ROBOT_ORIGIN)

        def compute(*args):
            handle.is_cancel_requested = True
            return 10

        nav._compute_path.side_effect = compute
        self.assertEqual(nav._run_path_matrix(handle).distances, [10, -1])
        nav.pause_nav2.assert_called_once()
        nav._release_service_group.assert_called_once()

    def test_cancel_while_queued_does_not_touch_navigation(self):
        nav, handle = server()
        nav._hold_service_group.return_value = False
        self.assertFalse(nav._run_path_matrix(handle).success)
        nav.resume_nav2.assert_not_called()
        nav._release_service_group.assert_not_called()

    def test_expired_budget_skips_paths(self):
        nav, handle = server()
        self.assertEqual(
            nav._plan_matrix(handle, [pose(), pose(4)], 0, pose(10)), [-1, -1, 0, -1, -1, 0]
        )
        nav._compute_path.assert_not_called()

    def test_longer_budget_reaches_planner(self):
        nav, handle = server()
        handle.request.budget_seconds = 120
        nav._run_path_matrix(handle)
        remaining = nav._compute_path.call_args.args[2] - time.monotonic()
        self.assertGreater(remaining, 119)
        self.assertLessEqual(remaining, 120)

    def test_already_active_navigation_stays_active(self):
        nav, handle = server(paused=False)
        nav._run_path_matrix(handle)
        nav.pause_nav2.assert_not_called()
        self.assertFalse(nav.nav2_paused)

    def test_planner_exception_restores_navigation(self):
        nav, handle = server()
        nav._compute_path.side_effect = RuntimeError("planner failed")
        with self.assertRaises(RuntimeError):
            nav._run_path_matrix(handle)
        self.assertTrue(nav.nav2_paused)
        nav._release_service_group.assert_called_once()

    def test_resume_exception_restores_navigation(self):
        nav, handle = server()
        nav.resume_nav2.side_effect = RuntimeError("activation failed")
        with self.assertRaises(RuntimeError):
            nav._run_path_matrix(handle)
        nav.pause_slam.assert_called_once()
        nav.pause_nav2.assert_called_once()
        nav._release_service_group.assert_called_once()

    def test_waits_for_setup_to_finish_before_touching_navigation(self):
        nav, handle = server()
        nav.nodes_status = False
        result = nav._run_path_matrix(handle)
        self.assertFalse(result.success)
        self.assertEqual(result.error, "Navigation not initialized")
        nav._hold_service_group.assert_not_called()

    def test_invalid_goals_rejected_without_navigation(self):
        for field, value in (
            ("mode", 99),
            ("budget_seconds", -1),
            ("budget_seconds", float("inf")),
            ("sublocations", []),
        ):
            nav, handle = server()
            setattr(handle.request, field, value)
            self.assertFalse(nav._run_path_matrix(handle).success)
            nav.resume_nav2.assert_not_called()
        nav, handle = server()
        handle.request.robot_start.header.frame_id = "odom"
        self.assertFalse(nav._run_path_matrix(handle).success)


class MatrixClientTests(unittest.TestCase):
    def client(self, distances, *, cancelled=False):
        client = CLIENT()
        client.node = NS()
        result = NS(success=True, distances=distances, error="")
        future = Mock()
        future.result.return_value = NS(result=result, status=5 if cancelled else 4)
        future.done.return_value = True
        handle = NS(accepted=True, get_result_async=lambda: future, cancel_goal_async=Mock())
        send = Mock()
        send.result.return_value = handle
        client.nav_query_matrix_client = NS(send_goal_async=Mock(return_value=send))
        CLIENT_ROS.spin_until_future_complete.reset_mock()
        return client, handle, future

    def test_all_layouts_decode_to_existing_dictionary(self):
        for mode, distances, starts in (
            (Goal.FULL, [10, 6, 0, 4, 5, 0], [None] + AREAS),
            (Goal.LOCATION_PAIRS, [0, 4, 5, 0], AREAS),
            (Goal.ROBOT_ORIGIN, [10, 6], [None]),
        ):
            client, _, _ = self.client(distances)
            status, matrix = client.get_path_matrix(AREAS, mode=mode)
            self.assertEqual(status, 1)
            self.assertEqual(
                matrix,
                {
                    (s, g): distances[r * 2 + c]
                    for r, s in enumerate(starts)
                    for c, g in enumerate(AREAS)
                },
            )

    def test_cancel_waits_for_stop_and_returns_partial(self):
        client, handle, future = self.client([10, -1], cancelled=True)
        future.done.side_effect = [False, True]
        status, matrix = client.get_path_matrix(AREAS, mode=Goal.ROBOT_ORIGIN)
        handle.cancel_goal_async.assert_called_once()
        self.assertEqual((status, matrix), (1, {(None, AREAS[0]): 10}))

    def test_no_result_after_cancel_is_error(self):
        client, handle, future = self.client([])
        future.done.return_value = False
        self.assertEqual(client.get_path_matrix(AREAS)[0], 0)
        handle.cancel_goal_async.assert_called_once()

    def test_budget_and_snapshot_forwarded_and_client_wait_extended(self):
        client, _, _ = self.client([10, 6])
        start = pose(10)
        client.get_path_matrix(AREAS, mode=Goal.ROBOT_ORIGIN, robot_start=start, budget_seconds=120)
        goal = client.nav_query_matrix_client.send_goal_async.call_args.args[0]
        self.assertIs(goal.robot_start, start)
        self.assertEqual(goal.budget_seconds, 120)
        self.assertEqual(CLIENT_ROS.spin_until_future_complete.call_args.kwargs["timeout_sec"], 124)

    def test_malformed_and_nonfinite_results(self):
        client, _, _ = self.client([10])
        self.assertEqual(client.get_path_matrix(AREAS)[0], 0)
        client, _, _ = self.client([float("inf"), float("nan")])
        self.assertEqual(client.get_path_matrix(AREAS, mode=Goal.ROBOT_ORIGIN), (1, {}))


if __name__ == "__main__":
    unittest.main()
