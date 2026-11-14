"""Distance-cache unit checks runnable without ROS: python3 -m unittest discover
-s task_manager/test -p 'test_gpsr_distance_cache.py'.

Load the production methods with lightweight message/transport doubles, avoiding
hardware imports and node startup. ROS builds and Gazebo remain integration checks.
"""

import ast
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[2]
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
    )
    namespace.update(extra)
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(ROOT / path), "exec"), namespace)
    return namespace[class_name]


GPSR = load_methods(
    "task_manager/scripts/gpsr_task_manager.py",
    "GPSRTM",
    {
        "_named_areas",
        "_query_paths",
        "_get_areas",
        "_load_batch_path_costs",
        "_discard_robot_paths",
        "_origin_matches",
        "_refresh_robot_paths",
        "_ensure_plan_origin",
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
        "_refresh_path_cache",
        "_matrix_needs_planner",
        "_setup",
    },
    GetRobotPose=NS(Response=NS),
    ReentrantCallbackGroup=NS,
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


def gpsr():
    tm = GPSR()
    tm._path_cache = {}
    tm._location_cache = {"obsolete": (1, 2)}
    tm._area_cache = {"obsolete": AREAS[0]}
    tm._areas = LIVE
    tm._origin_pose = None
    tm._plan_origin = None
    tm._resolve_area = lambda name: {"sofa": AREAS[0], "table": AREAS[1]}.get(name)
    tm._test_origin_initialized = False
    tm.get_logger = Mock(return_value=Mock())
    parameters = {
        "path_origin_translation_tolerance": 0.1,
        "path_origin_rotation_tolerance": math.radians(10),
    }
    tm.get_parameter = lambda key: NS(value=parameters[key])
    tm.subtask_manager = NS(
        nav=NS(
            retrieve_areas=Mock(return_value=(1, LIVE)), get_path_matrix=Mock(return_value=(1, {}))
        )
    )
    tm._current_robot_pose = Mock(return_value=pose())
    return tm


def server(mode=Goal.FULL, *, paused=True):
    nav = SERVER()
    nav.mapping = False
    nav.areas_data = LIVE
    nav.map_name = "arena"
    nav.areas_map_name = "arena"
    nav.config_path = "localization.yaml"
    nav._location_path_cache = {}
    nav._path_cache_context = None
    nav.use_nav2 = True
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
    nav.get_robot_pose_callback = Mock(return_value=NS(success=True, pose=pose(10)))
    nav._compute_path = Mock(
        side_effect=lambda goal, start, *_: abs(goal.pose.position.x - start.pose.position.x)
    )
    nav._path_length = lambda path: path
    request = Goal()
    request.mode = mode
    handle = NS(request=request, is_cancel_requested=False, publish_feedback=Mock())
    return nav, handle


class CacheTests(unittest.TestCase):
    def test_batch_reads_only_its_named_pairs_and_keeps_current_origin(self):
        tm = gpsr()
        tm._path_cache = {(AREAS[1], AREAS[0]): 99, (None, AREAS[0]): 88}
        tm._origin_pose = pose()
        fresh = {(AREAS[0], AREAS[1]): 6, (AREAS[0], AREAS[0]): 0}
        tm.subtask_manager.nav.get_path_matrix.return_value = (1, fresh)
        commands = [
            NS(
                commands=[
                    NS(action="go_to", location_to_go=name) for name in ("sofa", "table", "sofa")
                ]
            )
        ]
        tm._load_batch_path_costs(commands)
        self.assertEqual(tm._path_cache, {**fresh, (None, AREAS[0]): 88})
        self.assertEqual(tm._location_cache, {})
        self.assertEqual(tm._area_cache, {})
        self.assertIsNotNone(tm._origin_pose)
        args, kwargs = tm.subtask_manager.nav.get_path_matrix.call_args
        self.assertEqual(args[0], AREAS)
        self.assertEqual(kwargs, {"mode": Goal.LOCATION_PAIRS})

    def test_changed_live_areas_discard_old_robot_row(self):
        tm = gpsr()
        tm._areas = {"old": {"place": [0] * 7}}
        tm._origin_pose = pose(10)
        tm._path_cache[(None, AREAS[0])] = 88
        tm._load_batch_path_costs([])
        self.assertEqual(tm._named_areas(), AREAS)
        self.assertIsNone(tm._origin_pose)
        self.assertEqual(tm._path_cache, {})

    def test_named_areas_excludes_metadata(self):
        tm = gpsr()
        tm._areas = dict(
            LIVE,
            metadata="arena",
            polygons={"bounds": [[0, 0], [1, 1]]},
            invalid={"pose": [float("nan")] * 7},
        )
        self.assertEqual(tm._named_areas(), AREAS)

    def test_no_destinations_does_not_query_a_matrix(self):
        tm = gpsr()
        tm._load_batch_path_costs([NS(commands=[NS(action="say")])])
        tm.subtask_manager.nav.get_path_matrix.assert_not_called()
        self.assertEqual(tm._path_cache, {})

    def test_failed_query_drops_obsolete_batch_pairs(self):
        tm = gpsr()
        tm._path_cache = {(AREAS[0], AREAS[1]): 99}
        tm.subtask_manager.nav.get_path_matrix.return_value = (0, "Planner unavailable")
        tm._load_batch_path_costs([NS(commands=[NS(action="go_to", location_to_go="sofa")])])
        self.assertEqual(tm._path_cache, {})

    def test_origin_is_instruction_point_not_earlier_start(self):
        tm = gpsr()
        tm._origin_pose = pose(50)  # Old entrance origin.
        tm._path_cache = {(None, AREAS[0]): 50, (AREAS[0], AREAS[1]): 6}
        tm._current_robot_pose.return_value = pose(10)
        tm.subtask_manager.nav.get_path_matrix.return_value = (1, {(None, AREAS[0]): 10})
        tm._ensure_plan_origin()
        self.assertEqual(tm._plan_origin, (10, 0))
        self.assertEqual(tm._path_cache[(None, AREAS[0])], 10)
        self.assertEqual(tm._path_cache[(AREAS[0], AREAS[1])], 6)
        query = tm.subtask_manager.nav.get_path_matrix.call_args.kwargs
        self.assertEqual(query["mode"], Goal.ROBOT_ORIGIN)
        self.assertEqual(query["robot_start"].pose.position.x, 10)

    def test_unchanged_pose_and_yaw_wrap_avoid_query(self):
        tm = gpsr()
        tm._origin_pose = pose(yaw=math.pi - 0.01)
        tm._current_robot_pose.return_value = pose(0.01, yaw=-math.pi + 0.01)
        tm._ensure_plan_origin()
        tm.subtask_manager.nav.get_path_matrix.assert_not_called()

    def test_rotation_refreshes_only_origin(self):
        tm = gpsr()
        tm._origin_pose = pose()
        tm._current_robot_pose.return_value = pose(yaw=0.5)
        tm._ensure_plan_origin()
        self.assertEqual(
            tm.subtask_manager.nav.get_path_matrix.call_args.kwargs["mode"], Goal.ROBOT_ORIGIN
        )

    def test_movement_during_query_discards_origin_but_keeps_named_pairs(self):
        tm = gpsr()
        tm._path_cache = {(AREAS[0], AREAS[1]): 6}
        tm._current_robot_pose.side_effect = [pose(10), pose(12)]
        tm.subtask_manager.nav.get_path_matrix.return_value = (1, {(None, AREAS[0]): 10})
        tm._refresh_robot_paths()
        self.assertEqual(tm._path_cache, {(AREAS[0], AREAS[1]): 6})
        self.assertIsNone(tm._plan_origin)

    def test_missing_tf_discards_stale_origin(self):
        tm = gpsr()
        tm._path_cache = {(None, AREAS[0]): 50}
        tm._origin_pose = pose(50)
        tm._current_robot_pose.return_value = None
        tm._ensure_plan_origin()
        self.assertEqual(tm._path_cache, {})
        self.assertIsNone(tm._plan_origin)

    def test_origin_query_failure_uses_current_straight_line_fallback(self):
        tm = gpsr()
        tm._current_robot_pose.return_value = pose(10)
        tm.subtask_manager.nav.get_path_matrix.return_value = (0, "Planner unavailable")
        tm._refresh_robot_paths()
        tm._resolve_area = lambda name: AREAS[0]
        tm._resolve_xy = lambda name: (0, 0)
        self.assertEqual(tm._path_cost(None, "sofa"), 14)

    def test_gpsr_startup_never_warms_named_pairs(self):
        tm = gpsr()
        # An idle synthetic state exercises startup without HRI/navigation side effects.
        tm.current_state = "IDLE"
        tm.interleave_enabled = True
        tm.test_mode = False
        tm._refresh_robot_paths = Mock()
        tm.run()
        tm.run()
        tm._refresh_robot_paths.assert_not_called()
        tm.subtask_manager.nav.get_path_matrix.assert_not_called()
        tm.test_mode = True
        tm.run()
        tm.run()
        tm._refresh_robot_paths.assert_called_once()
        tm._test_origin_initialized = False
        tm.interleave_enabled = False
        tm.run()
        tm._refresh_robot_paths.assert_called_once()

    def test_instruction_point_origin_is_captured_after_arrival_before_commands(self):
        tm = gpsr()
        tm.interleave_enabled = True
        tm.test_mode = False
        tm.current_state = GPSR.TaskStates.START
        tm._track_state_change = Mock()
        tm.subtask_manager.nav.check_door = Mock()
        events = []
        tm.navigate_to = Mock(side_effect=lambda *_: events.append("arrive") or 1)
        tm._refresh_robot_paths = Mock(side_effect=lambda: events.append("origin"))
        tm.subtask_manager.hri = NS(say=lambda *_: events.append("announce"))
        tm.run()
        self.assertEqual(events, ["arrive", "origin", "announce"])
        self.assertEqual(tm.current_state, GPSR.TaskStates.WAIT_BUTTON_COMMAND)

    def test_batch_cleanup_discards_its_snapshot_not_navigation_cache(self):
        tm = gpsr()
        named = {(AREAS[0], AREAS[1]): 6}
        tm._path_cache = dict(named)
        tm._path_cache[(None, AREAS[0])] = 10
        tm._origin_pose = pose(10)
        tm._plan_origin = (10, 0)
        tm._completed = set()
        tm.batched_commands = [NS(), NS(), NS()]
        tm._ensure_plan_origin = Mock()
        tm._load_batch_path_costs = Mock()
        tm._spoken_merged_plan = Mock(return_value="Visit the kitchen")
        tm.subtask_manager.hri = Mock()
        tm.gpsr_tasks = tm.gpsr_individual_tasks = NS()
        tm._on_action_start = tm._on_action_complete = Mock()
        plan = NS(actions=[], fallback=[])
        root = NS(status="SUCCESS")
        tree = NS(setup=Mock(), tick=Mock())
        with patch.dict(
            GPSR._execute_interleaved_batch.__globals__,
            {
                "merge": Mock(return_value=plan),
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
        self.assertEqual(tm._path_cache, {})
        self.assertIsNone(tm._areas)
        self.assertIsNone(tm._plan_origin)
        tm.subtask_manager.nav.get_path_matrix.assert_not_called()


class MatrixServerTests(unittest.TestCase):
    def test_setup_warms_once_after_navigation_and_initial_pose(self):
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
        nav._refresh_path_cache = Mock(side_effect=lambda: events.append("warm"))
        nav._setup()
        nav._setup()
        self.assertEqual(events, ["areas", "requirements", "slam", "nav2", "pose", "warm"])
        self.assertTrue(nav.nodes_status)
        nav._refresh_path_cache.assert_called_once()

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
            nav._hold_service_group.assert_not_called()

    def test_warmed_queries_reuse_directed_pairs_in_any_request_order(self):
        nav, handle = server(Goal.LOCATION_PAIRS)
        nav._compute_path.side_effect = (
            lambda goal, start, *_: 6 if start.pose.position.x == 0 else 9
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
