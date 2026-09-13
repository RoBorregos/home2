"""Timeout budgets (seconds) for the GPSR interleaved executor."""

NAVIGATE_TIMEOUT_S = 45.0
PICK_TIMEOUT_S = 30.0
FIND_TIMEOUT_S = 60.0
SAY_TIMEOUT_S = 10.0
DEFAULT_TIMEOUT_S = 30.0
# Follow-person budget: FOLLOW_MAX_DURATION (120 s, gpsr_tasks) + lock-on
# retries and speech. It must sit ABOVE the handler's own cap or the Retry
# decorator would restart a follow that ended normally.
FOLLOW_TIMEOUT_S = 160.0
# Shelf placing is the worst measured sink (60-120 s, docs/task_manager/ppc).
# Capping it is also what keeps the arm from idling past the 30 s arena rule.
SHELF_PLACE_TIMEOUT_S = 100.0
POUR_TIMEOUT_S = 60.0

# Budget for one interleaved batch of commands.
GLOBAL_BUDGET_S = 300.0
# Whole-test budget from the rulebook: 7:00 for HRIC/PPC/GPSR/Laundry.
TASK_BUDGET_S = 420.0

ACTION_TIMEOUTS = {
    "go_to": NAVIGATE_TIMEOUT_S,
    "pick_object": PICK_TIMEOUT_S,
    "place_object": PICK_TIMEOUT_S,
    "give_object": PICK_TIMEOUT_S,
    "find_person": FIND_TIMEOUT_S,
    "find_person_by_name": FIND_TIMEOUT_S,
    "get_person_info": FIND_TIMEOUT_S,
    "follow_person_until": FOLLOW_TIMEOUT_S,
    "guide_person_to": NAVIGATE_TIMEOUT_S,
    "count": FIND_TIMEOUT_S,
    "get_visual_info": FIND_TIMEOUT_S,
    "answer_question": FIND_TIMEOUT_S,
    "say_with_context": SAY_TIMEOUT_S,
    "place_on_shelf": SHELF_PLACE_TIMEOUT_S,
    "pour": POUR_TIMEOUT_S,
    "detect_objects": FIND_TIMEOUT_S,
}


def timeout_for(action: str) -> float:
    return ACTION_TIMEOUTS.get(action, DEFAULT_TIMEOUT_S)
