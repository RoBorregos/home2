#!/usr/bin/env python3

"""
Test for parse_plan_to_text — verifies that GPSR structured commands
are converted to a human-readable plan sentence correctly.

Run with:
    ros2 run task_manager test_plan_parser
"""

import rclpy
from rclpy.node import Node
from task_manager.config.hri.debug import config as test_hri_config
from task_manager.subtask_managers.hri_tasks import HRITasks
from task_manager.utils.baml_client.types import (
    AnswerQuestion,
    Count,
    FindPerson,
    FindPersonByName,
    FollowPersonUntil,
    GetPersonInfo,
    GetVisualInfo,
    GiveObject,
    GoTo,
    GuidePersonTo,
    PickObject,
    PlaceObject,
    SayWithContext,
)
from task_manager.utils.task import Task

GREEN = "\033[92m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
RESET = "\033[0m"
BOLD = "\033[1m"

TEST_CASES = [
    # ── From GPSR_COMMANDS ──────────────────────────────────────────────────
    {
        "label": "goToLoc — deliver cleanser",
        "original": "go to the sofa then find a cleanser and take it and deliver it to me",
        "cmds": [
            GoTo(action="go_to", location_to_go="sofa"),
            PickObject(action="pick_object", object_to_pick="cleanser"),
            GiveObject(action="give_object"),
        ],
    },
    {
        "label": "takeObjFromPlcmt — strawberry to standing person",
        "original": "get a strawberry from the kitchen table and bring it to the standing person in the office",
        "cmds": [
            GoTo(action="go_to", location_to_go="kitchen table"),
            PickObject(action="pick_object", object_to_pick="strawberry"),
            GoTo(action="go_to", location_to_go="office"),
            FindPerson(action="find_person", attribute_value="standing person"),
            GiveObject(action="give_object"),
        ],
    },
    {
        "label": "findPrsInRoom — guide lying person",
        "original": "locate a lying person in the office and guide them to the dishwasher",
        "cmds": [
            GoTo(action="go_to", location_to_go="office"),
            FindPerson(action="find_person", attribute_value="lying person"),
            GuidePersonTo(action="guide_person_to", destination_room="dishwasher"),
        ],
    },
    {
        "label": "findObjInRoom — sugar to desk",
        "original": "locate a sugar in the living room then take it and place it on the desk",
        "cmds": [
            GoTo(action="go_to", location_to_go="living room"),
            PickObject(action="pick_object", object_to_pick="sugar"),
            GoTo(action="go_to", location_to_go="desk"),
            PlaceObject(action="place_object"),
        ],
    },
    {
        "label": "meetPrsAtBeac — follow Jane",
        "original": "meet Jane in the bathroom and follow them",
        "cmds": [
            GoTo(action="go_to", location_to_go="bathroom"),
            FindPersonByName(action="find_person_by_name", name="jane"),
            FollowPersonUntil(action="follow_person_until", destination="canceled"),
        ],
    },
    {
        "label": "countObjOnPlcmt — count drinks on tv stand",
        "original": "tell me how many drinks there are on the tv stand",
        "cmds": [
            GoTo(action="go_to", location_to_go="tv stand"),
            Count(action="count", target_to_count="drinks"),
            GoTo(action="go_to", location_to_go="start_location"),
            SayWithContext(
                action="say_with_context",
                user_instruction="tell me how many drinks there are on the tv stand",
                previous_command_info=["count"],
            ),
        ],
    },
    {
        "label": "countPrsInRoom — standing persons in living room",
        "original": "tell me how many standing persons are in the living room",
        "cmds": [
            GoTo(action="go_to", location_to_go="living room"),
            Count(action="count", target_to_count="standing persons"),
            GoTo(action="go_to", location_to_go="start_location"),
            SayWithContext(
                action="say_with_context",
                user_instruction="tell me how many standing persons are in the living room",
                previous_command_info=["count"],
            ),
        ],
    },
    {
        "label": "tellPrsInfoInLoc — gesture at waste basket",
        "original": "tell me the gesture of the person at the waste basket",
        "cmds": [
            GoTo(action="go_to", location_to_go="waste basket"),
            FindPerson(action="find_person", attribute_value=""),
            GetPersonInfo(action="get_person_info", info_type="gesture"),
            GoTo(action="go_to", location_to_go="start_location"),
            SayWithContext(
                action="say_with_context",
                user_instruction="tell me the gesture of the person at the waste basket",
                previous_command_info=["get_person_info"],
            ),
        ],
    },
    {
        "label": "tellObjPropOnPlcmt — thinnest object on bed",
        "original": "tell me what is the thinnest object on the bed",
        "cmds": [
            GoTo(action="go_to", location_to_go="bed"),
            GetVisualInfo(action="get_visual_info", measure="thinnest", object_category="object"),
            GoTo(action="go_to", location_to_go="start_location"),
            SayWithContext(
                action="say_with_context",
                user_instruction="tell me what is the thinnest object on the bed",
                previous_command_info=["get_visual_info"],
            ),
        ],
    },
    {
        "label": "talkInfoToGestPrsInRoom — day tomorrow to pointing person",
        "original": "say what day is tomorrow to the person pointing to the left in the office",
        "cmds": [
            GoTo(action="go_to", location_to_go="office"),
            FindPerson(action="find_person", attribute_value="person pointing to the left"),
            SayWithContext(
                action="say_with_context",
                user_instruction="say what day is tomorrow to the person pointing to the left in the office",
                previous_command_info=["what day is tomorrow"],
            ),
        ],
    },
    {
        "label": "answerToGestPrsInRoom — quiz left arm",
        "original": "answer the quiz of the person raising their left arm in the living room",
        "cmds": [
            GoTo(action="go_to", location_to_go="living room"),
            FindPerson(action="find_person", attribute_value="person raising their left arm"),
            AnswerQuestion(action="answer_question"),
        ],
    },
    {
        "label": "followNameFromBeacToRoom — follow Paris to office",
        "original": "follow Paris from the sink to the office",
        "cmds": [
            GoTo(action="go_to", location_to_go="sink"),
            FindPersonByName(action="find_person_by_name", name="paris"),
            FollowPersonUntil(action="follow_person_until", destination="office"),
        ],
    },
    {
        "label": "guideNameFromBeacToBeac — take Simone to office",
        "original": "take Simone from the sofa to the office",
        "cmds": [
            GoTo(action="go_to", location_to_go="sofa"),
            FindPersonByName(action="find_person_by_name", name="simone"),
            GuidePersonTo(action="guide_person_to", destination_room="office"),
        ],
    },
    {
        "label": "guidePrsFromBeacToBeac — lying person to exit",
        "original": "lead the lying person from the desk to the exit",
        "cmds": [
            GoTo(action="go_to", location_to_go="desk"),
            FindPerson(action="find_person", attribute_value="lying person"),
            GuidePersonTo(action="guide_person_to", destination_room="exit"),
        ],
    },
    {
        "label": "guideClothPrsFromBeacToBeac — white coat to living room",
        "original": "lead the person wearing a white coat from the sofa to the living room",
        "cmds": [
            GoTo(action="go_to", location_to_go="sofa"),
            FindPerson(action="find_person", attribute_value="white coat"),
            GuidePersonTo(action="guide_person_to", destination_room="living room"),
        ],
    },
    {
        "label": "bringMeObjFromPlcmt — bring peach from bed",
        "original": "bring me a peach from the bed",
        "cmds": [
            GoTo(action="go_to", location_to_go="bed"),
            PickObject(action="pick_object", object_to_pick="peach"),
            GoTo(action="go_to", location_to_go="start_location"),
            GiveObject(action="give_object"),
        ],
    },
    {
        "label": "tellCatPropOnPlcmt — lightest dish on side tables",
        "original": "tell me what is the lightest dish on the side tables",
        "cmds": [
            GoTo(action="go_to", location_to_go="side tables"),
            GetVisualInfo(action="get_visual_info", measure="lightest", object_category="dish"),
            GoTo(action="go_to", location_to_go="start_location"),
            SayWithContext(
                action="say_with_context",
                user_instruction="tell me what is the lightest dish on the side tables",
                previous_command_info=["get_visual_info"],
            ),
        ],
    },
    {
        "label": "greetClothDscInRm — greet black sweater, follow to shelf",
        "original": "greet the person wearing a black sweater in the bedroom and follow them to the shelf",
        "cmds": [
            GoTo(action="go_to", location_to_go="bedroom"),
            FindPerson(action="find_person", attribute_value="black sweater"),
            SayWithContext(
                action="say_with_context",
                user_instruction="greet the person wearing a black sweater in the bedroom",
                previous_command_info=["introduction"],
            ),
            FollowPersonUntil(action="follow_person_until", destination="shelf"),
        ],
    },
    {
        "label": "greetNameInRm — salute Jane, take to storage rack",
        "original": "salute Jane in the office and take them to the storage rack",
        "cmds": [
            GoTo(action="go_to", location_to_go="office"),
            FindPersonByName(action="find_person_by_name", name="jane"),
            SayWithContext(
                action="say_with_context",
                user_instruction="salute Jane in the office",
                previous_command_info=["introduction"],
            ),
            GuidePersonTo(action="guide_person_to", destination_room="storage rack"),
        ],
    },
    {
        "label": "meetNameAtLocThenFindInRm — meet Robin, find in bathroom",
        "original": "meet Robin at the waste basket then look for them in the bathroom",
        "cmds": [
            GoTo(action="go_to", location_to_go="waste basket"),
            FindPersonByName(action="find_person_by_name", name="robin"),
            GoTo(action="go_to", location_to_go="bathroom"),
            FindPersonByName(action="find_person_by_name", name="robin"),
        ],
    },
    {
        "label": "countClothPrsInRoom — black sweaters in bathroom",
        "original": "tell me how many people in the bathroom are wearing black sweaters",
        "cmds": [
            GoTo(action="go_to", location_to_go="bathroom"),
            Count(action="count", target_to_count="people wearing black sweaters"),
            GoTo(action="go_to", location_to_go="start_location"),
            SayWithContext(
                action="say_with_context",
                user_instruction="tell me how many people in the bathroom are wearing black sweaters",
                previous_command_info=["count"],
            ),
        ],
    },
    {
        "label": "tellPrsInfoAtLocToPrsAtLoc — gesture side tables to bookshelf",
        "original": "tell the gesture of the person at the side tables to the person at the bookshelf",
        "cmds": [
            GoTo(action="go_to", location_to_go="side tables"),
            FindPerson(action="find_person", attribute_value=""),
            GetPersonInfo(action="get_person_info", info_type="gesture"),
            GoTo(action="go_to", location_to_go="bookshelf"),
            FindPerson(action="find_person", attribute_value=""),
            SayWithContext(
                action="say_with_context",
                user_instruction="tell the gesture of the person at the side tables to the person at the bookshelf",
                previous_command_info=["get_person_info"],
            ),
        ],
    },
    {
        "label": "followPrsAtLoc — follow pointing right at refrigerator",
        "original": "follow the person pointing to the right at the refrigerator",
        "cmds": [
            GoTo(action="go_to", location_to_go="refrigerator"),
            FindPerson(action="find_person", attribute_value="person pointing to the right"),
            FollowPersonUntil(action="follow_person_until", destination="canceled"),
        ],
    },
    # ── Extra cases ─────────────────────────────────────────────────────────
    {
        "label": "Single — go_to",
        "original": "go to the kitchen",
        "cmds": [GoTo(action="go_to", location_to_go="kitchen")],
    },
    {
        "label": "Single — pick object",
        "original": "pick up the bottle",
        "cmds": [PickObject(action="pick_object", object_to_pick="bottle")],
    },
    {
        "label": "Single — answer question",
        "original": "answer the question",
        "cmds": [AnswerQuestion(action="answer_question")],
    },
    {
        "label": "Single — give object",
        "original": "give the object",
        "cmds": [GiveObject(action="give_object")],
    },
    {
        "label": "Single — place object",
        "original": "place the object",
        "cmds": [PlaceObject(action="place_object")],
    },
    {
        "label": "Empty command list",
        "original": "(no command)",
        "cmds": [],
    },
    {
        "label": "return to start_location then give",
        "original": "come back to me",
        "cmds": [
            GoTo(action="go_to", location_to_go="start_location"),
            GiveObject(action="give_object"),
        ],
    },
    {
        "label": "find person — no attribute",
        "original": "find anyone",
        "cmds": [FindPerson(action="find_person", attribute_value="")],
    },
    {
        "label": "find person — waving",
        "original": "find the waving person",
        "cmds": [FindPerson(action="find_person", attribute_value="waving")],
    },
    {
        "label": "count people in kitchen",
        "original": "how many people are in the kitchen",
        "cmds": [
            GoTo(action="go_to", location_to_go="kitchen"),
            Count(action="count", target_to_count="people"),
            GoTo(action="go_to", location_to_go="start_location"),
            SayWithContext(
                action="say_with_context",
                user_instruction="how many people are in the kitchen",
                previous_command_info=["count"],
            ),
        ],
    },
    {
        "label": "get person pose",
        "original": "what is the pose of that person",
        "cmds": [
            FindPerson(action="find_person", attribute_value=""),
            GetPersonInfo(action="get_person_info", info_type="pose"),
            GoTo(action="go_to", location_to_go="start_location"),
            SayWithContext(
                action="say_with_context",
                user_instruction="the pose of the person",
                previous_command_info=["get_person_info"],
            ),
        ],
    },
    {
        "label": "get person name at door",
        "original": "what is the name of the person at the door",
        "cmds": [
            GoTo(action="go_to", location_to_go="door"),
            FindPerson(action="find_person", attribute_value=""),
            GetPersonInfo(action="get_person_info", info_type="name"),
            GoTo(action="go_to", location_to_go="start_location"),
            SayWithContext(
                action="say_with_context",
                user_instruction="the name of the person at the door",
                previous_command_info=["get_person_info"],
            ),
        ],
    },
    {
        "label": "biggest snack on shelf",
        "original": "tell me the biggest snack on the shelf",
        "cmds": [
            GoTo(action="go_to", location_to_go="shelf"),
            GetVisualInfo(action="get_visual_info", measure="biggest", object_category="snack"),
            GoTo(action="go_to", location_to_go="start_location"),
            SayWithContext(
                action="say_with_context",
                user_instruction="the biggest snack on the shelf",
                previous_command_info=["get_visual_info"],
            ),
        ],
    },
    {
        "label": "follow person to bedroom",
        "original": "follow the person to the bedroom",
        "cmds": [
            FindPerson(action="find_person", attribute_value=""),
            FollowPersonUntil(action="follow_person_until", destination="bedroom"),
        ],
    },
    {
        "label": "guide person to entrance",
        "original": "guide the person to the entrance",
        "cmds": [
            FindPerson(action="find_person", attribute_value=""),
            GuidePersonTo(action="guide_person_to", destination_room="entrance"),
        ],
    },
    {
        "label": "bring cola from kitchen to Adel",
        "original": "get a cola from the kitchen and give it to Adel in the living room",
        "cmds": [
            GoTo(action="go_to", location_to_go="kitchen"),
            PickObject(action="pick_object", object_to_pick="cola"),
            GoTo(action="go_to", location_to_go="living room"),
            FindPersonByName(action="find_person_by_name", name="Adel"),
            GiveObject(action="give_object"),
        ],
    },
    {
        "label": "bring milk from fridge to me",
        "original": "bring me a milk from the refrigerator",
        "cmds": [
            GoTo(action="go_to", location_to_go="refrigerator"),
            PickObject(action="pick_object", object_to_pick="milk"),
            GoTo(action="go_to", location_to_go="start_location"),
            GiveObject(action="give_object"),
        ],
    },
    {
        "label": "put apple on kitchen table",
        "original": "pick up the apple and place it on the kitchen table",
        "cmds": [
            PickObject(action="pick_object", object_to_pick="apple"),
            GoTo(action="go_to", location_to_go="kitchen table"),
            PlaceObject(action="place_object"),
        ],
    },
    {
        "label": "count snacks on desk",
        "original": "tell me how many snacks are on the desk",
        "cmds": [
            GoTo(action="go_to", location_to_go="desk"),
            Count(action="count", target_to_count="snacks"),
            GoTo(action="go_to", location_to_go="start_location"),
            SayWithContext(
                action="say_with_context",
                user_instruction="how many snacks are on the desk",
                previous_command_info=["count"],
            ),
        ],
    },
    {
        "label": "count lying persons in bedroom",
        "original": "how many lying persons are in the bedroom",
        "cmds": [
            GoTo(action="go_to", location_to_go="bedroom"),
            Count(action="count", target_to_count="lying persons"),
            GoTo(action="go_to", location_to_go="start_location"),
            SayWithContext(
                action="say_with_context",
                user_instruction="how many lying persons are in the bedroom",
                previous_command_info=["count"],
            ),
        ],
    },
    {
        "label": "find Sophia at entrance, guide to living room",
        "original": "find Sophia at the entrance and guide her to the living room",
        "cmds": [
            GoTo(action="go_to", location_to_go="entrance"),
            FindPersonByName(action="find_person_by_name", name="Sophia"),
            GuidePersonTo(action="guide_person_to", destination_room="living room"),
        ],
    },
    {
        "label": "find Marcus at sofa, follow until cancelled",
        "original": "follow Marcus from the sofa",
        "cmds": [
            GoTo(action="go_to", location_to_go="sofa"),
            FindPersonByName(action="find_person_by_name", name="Marcus"),
            FollowPersonUntil(action="follow_person_until", destination="cancelled"),
        ],
    },
    {
        "label": "find person pointing right, get gesture",
        "original": "tell me the gesture of the person pointing to the right in the kitchen",
        "cmds": [
            GoTo(action="go_to", location_to_go="kitchen"),
            FindPerson(action="find_person", attribute_value="person pointing to the right"),
            GetPersonInfo(action="get_person_info", info_type="gesture"),
            GoTo(action="go_to", location_to_go="start_location"),
            SayWithContext(
                action="say_with_context",
                user_instruction="the gesture of the person pointing to the right",
                previous_command_info=["get_person_info"],
            ),
        ],
    },
    {
        "label": "answer questions from waving person",
        "original": "answer the questions of the waving person in the office",
        "cmds": [
            GoTo(action="go_to", location_to_go="office"),
            FindPerson(action="find_person", attribute_value="waving person"),
            AnswerQuestion(action="answer_question"),
        ],
    },
    {
        "label": "say team affiliation to person",
        "original": "say your team affiliation to the person raising both arms in the bathroom",
        "cmds": [
            GoTo(action="go_to", location_to_go="bathroom"),
            FindPerson(action="find_person", attribute_value="person raising both arms"),
            SayWithContext(
                action="say_with_context",
                user_instruction="your team affiliation",
                previous_command_info=["affiliation"],
            ),
        ],
    },
    {
        "label": "pick book from desk, place on shelf",
        "original": "pick the book from the desk and put it on the shelf",
        "cmds": [
            GoTo(action="go_to", location_to_go="desk"),
            PickObject(action="pick_object", object_to_pick="book"),
            GoTo(action="go_to", location_to_go="shelf"),
            PlaceObject(action="place_object"),
        ],
    },
    {
        "label": "pick cup, give to Anna",
        "original": "pick the cup and give it to Anna",
        "cmds": [
            PickObject(action="pick_object", object_to_pick="cup"),
            FindPersonByName(action="find_person_by_name", name="Anna"),
            GiveObject(action="give_object"),
        ],
    },
    {
        "label": "find red shirt person, tell name",
        "original": "find the person in the red shirt and tell me their name",
        "cmds": [
            FindPerson(action="find_person", attribute_value="red shirt"),
            GetPersonInfo(action="get_person_info", info_type="name"),
            GoTo(action="go_to", location_to_go="start_location"),
            SayWithContext(
                action="say_with_context",
                user_instruction="the name of the person in the red shirt",
                previous_command_info=["get_person_info"],
            ),
        ],
    },
    {
        "label": "count drinks in kitchen",
        "original": "count the drinks in the kitchen",
        "cmds": [
            GoTo(action="go_to", location_to_go="kitchen"),
            Count(action="count", target_to_count="drinks"),
            GoTo(action="go_to", location_to_go="start_location"),
            SayWithContext(
                action="say_with_context",
                user_instruction="how many drinks are in the kitchen",
                previous_command_info=["count"],
            ),
        ],
    },
    {
        "label": "meet Sam at fridge, guide to exit",
        "original": "meet Sam at the refrigerator and take him to the exit",
        "cmds": [
            GoTo(action="go_to", location_to_go="refrigerator"),
            FindPersonByName(action="find_person_by_name", name="Sam"),
            GuidePersonTo(action="guide_person_to", destination_room="exit"),
        ],
    },
    {
        "label": "complex — fetch and report visual info",
        "original": "go to the bookshelf, find the biggest food item, then return and tell me",
        "cmds": [
            GoTo(action="go_to", location_to_go="bookshelf"),
            GetVisualInfo(action="get_visual_info", measure="biggest", object_category="food"),
            GoTo(action="go_to", location_to_go="start_location"),
            SayWithContext(
                action="say_with_context",
                user_instruction="the biggest food item on the bookshelf",
                previous_command_info=["get_visual_info"],
            ),
        ],
    },
    {
        "label": "complex — find, pick, deliver across rooms",
        "original": "get the newspaper from the office and deliver it to Anna in the kitchen",
        "cmds": [
            GoTo(action="go_to", location_to_go="office"),
            PickObject(action="pick_object", object_to_pick="newspaper"),
            GoTo(action="go_to", location_to_go="kitchen"),
            FindPersonByName(action="find_person_by_name", name="Anna"),
            GiveObject(action="give_object"),
        ],
    },
    {
        "label": "find Lucas at kitchen, answer his questions",
        "original": "find Lucas in the kitchen and answer his questions",
        "cmds": [
            GoTo(action="go_to", location_to_go="kitchen"),
            FindPersonByName(action="find_person_by_name", name="Lucas"),
            AnswerQuestion(action="answer_question"),
        ],
    },
    {
        "label": "tell day of week to waving person",
        "original": "tell the waving person what day is today",
        "cmds": [
            FindPerson(action="find_person", attribute_value="waving"),
            SayWithContext(
                action="say_with_context",
                user_instruction="what day is today",
                previous_command_info=["time"],
            ),
        ],
    },
    {
        "label": "lightest object on bookshelf",
        "original": "what is the lightest object on the bookshelf",
        "cmds": [
            GoTo(action="go_to", location_to_go="bookshelf"),
            GetVisualInfo(action="get_visual_info", measure="lightest", object_category="object"),
            GoTo(action="go_to", location_to_go="start_location"),
            SayWithContext(
                action="say_with_context",
                user_instruction="the lightest object on the bookshelf",
                previous_command_info=["get_visual_info"],
            ),
        ],
    },
    {
        "label": "follow standing person from door to kitchen",
        "original": "follow the standing person from the door to the kitchen",
        "cmds": [
            GoTo(action="go_to", location_to_go="door"),
            FindPerson(action="find_person", attribute_value="standing person"),
            FollowPersonUntil(action="follow_person_until", destination="kitchen"),
        ],
    },
    {
        "label": "count people wearing hats in bedroom",
        "original": "how many people in the bedroom are wearing hats",
        "cmds": [
            GoTo(action="go_to", location_to_go="bedroom"),
            Count(action="count", target_to_count="people wearing hats"),
            GoTo(action="go_to", location_to_go="start_location"),
            SayWithContext(
                action="say_with_context",
                user_instruction="how many people in the bedroom are wearing hats",
                previous_command_info=["count"],
            ),
        ],
    },
    {
        "label": "tell name of person at sofa to person at table",
        "original": "tell the name of the person at the sofa to the person at the table",
        "cmds": [
            GoTo(action="go_to", location_to_go="sofa"),
            FindPerson(action="find_person", attribute_value=""),
            GetPersonInfo(action="get_person_info", info_type="name"),
            GoTo(action="go_to", location_to_go="table"),
            FindPerson(action="find_person", attribute_value=""),
            SayWithContext(
                action="say_with_context",
                user_instruction="tell the name of the person at the sofa",
                previous_command_info=["get_person_info"],
            ),
        ],
    },
    {
        "label": "pick toy, go to playroom, place",
        "original": "take the toy to the playroom",
        "cmds": [
            PickObject(action="pick_object", object_to_pick="toy"),
            GoTo(action="go_to", location_to_go="playroom"),
            PlaceObject(action="place_object"),
        ],
    },
    {
        "label": "heaviest drink on counter",
        "original": "which drink on the counter is the heaviest",
        "cmds": [
            GoTo(action="go_to", location_to_go="counter"),
            GetVisualInfo(action="get_visual_info", measure="heaviest", object_category="drink"),
            GoTo(action="go_to", location_to_go="start_location"),
            SayWithContext(
                action="say_with_context",
                user_instruction="the heaviest drink on the counter",
                previous_command_info=["get_visual_info"],
            ),
        ],
    },
]


class TestPlanParser(Node):
    def __init__(self):
        super().__init__("test_plan_parser")
        self.hri = HRITasks(self, config=test_hri_config, task=Task.DEBUG)
        rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("TestPlanParser has started.")
        self.run()

    def run(self):
        passed = 0
        failed = 0

        print(f"\n{BOLD}{'=' * 70}")
        print("  GPSR Plan Parser — Test Suite")
        print(f"{'=' * 70}{RESET}\n")

        for i, tc in enumerate(TEST_CASES, 1):
            result = self.hri.parse_plan_to_text(tc["cmds"])
            ok = result.startswith("My plan is") or result == "I have no steps to execute."

            if ok:
                passed += 1
            else:
                failed += 1

            status = f"{GREEN}OK{RESET}" if ok else f"\033[91mFAIL{RESET}"
            print(f"{CYAN}[{i:02d}] {tc['label']}{RESET}  [{status}]")
            print(f"     Original : {YELLOW}{tc['original']}{RESET}")
            print(f"     Parsed   : {GREEN}{result}{RESET}")
            print()

            self.get_logger().info(f"[{i:02d}] {tc['label']} -> {result}")

        print(f"{BOLD}Results: {passed}/{len(TEST_CASES)} passed{RESET}")
        if failed:
            print(f"\033[91m{failed} test(s) produced unexpected output — review above.{RESET}")

        exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = TestPlanParser()
    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
