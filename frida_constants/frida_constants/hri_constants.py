from enum import Enum

SPEAK_SERVICE = "/hri/speech/speak"
STT_SERVICE_NAME = "/hri/speech/STT"
HOTWORD_SERVICE_NAME = "/hri/speech/hotword_service"
KEYWORD_TOPIC = "/hri/speech/kws"
WAKEWORD_TOPIC = "/hri/speech/oww"
USEFUL_AUDIO_TOPIC = "/hri/speech/useful_audio"
COMMAND_INTERPRETER_SERVICE = "/hri/nlp/command_interpreter"
LLM_WRAPPER_SERVICE = "/hri/nlp/llm"
EXTRACT_DATA_SERVICE = "/hri/nlp/data_extractor"
IS_POSITIVE_SERVICE = "/hri/nlp/is_positive"
IS_NEGATIVE_SERVICE = "/hri/nlp/is_negative"
IS_COHERENT_SERVICE = "/hri/nlp/is_coherent"
CATEGORIZE_SERVICE = "/hri/nlp/categorize"
CONVESATION_SERVICE = "/hri/nlp/conversation"
GRAMMAR_SERVICE = "/hri/nlp/grammar"
COMMON_INTEREST_SERVICE = "/hri/nlp/common_interest"
RAG_SERVICE = "/hri/rag/answer_question"
DISPLAY_IMAGE_TOPIC = "/hri/display/change_video"
DISPLAY_MAP_TOPIC = "/hri/display/map"
DISPLAY_PUBLISHER = "/hri/display/frida_questions"
ANSWER_PUBLISHER = "/hri/display/answers"
RESPEAKER_LIGHT_TOPIC = "/hri/speech/respeaker/light"
RESPEAKER_DOA_TOPIC = "/hri/speech/respeaker/doa"
STT_ACTION_SERVER_NAME = "/hri/speech/STT_action_server"
START_BUTTON_CLIENT = "/hri/display/button_press"

GPSR_COMMANDS = {
    "go_to",
    "pick_object",
    "place_object",
    "say_with_context",
    "say",
    "answer_question",
    "get_visual_info",
    "give_object",
    "follow_person_until",
    "guide_person_to",
    "get_person_info",
    # "find_object",
    "count",
    "find_person",
    "find_person_by_name",
}


class MODEL(Enum):
    GRAMMAR = "qwen3"
    LLM_WRAPPER = "qwen3"
    COMMON_INTEREST = "qwen3"
    CATEGORIZE_SHELVES = "qwen3"
    IS_POSITIVE = "qwen3"
    IS_NEGATIVE = "qwen3"
    GENERIC_STRUCTURED_OUTPUT = "qwen3"
    GENERATE_RESPONSE = "qwen3"
    STRUCTURED_RESPONSE = "qwen3"
    EXTRACT_INFO_REQUESTED = "qwen3"
    GET_COMMANDS = "qwen3"
    CommonInterest = "qwen3"


class KNOWLEDGE_TYPE(Enum):
    TEC = "tec_knowledge"
    FRIDA = "frida_knowledge"
    ROBORREGOS = "roborregos_knowledge"


# OpenWakeWord
USE_OWW = True
SENSITIVITY_THRESHOLD = 0.2
threshold = 0.6

# STT
USEFUL_AUDIO_NODE_NAME = "useful_audio_node"
DEFAULT_HOTWORDS = "Frida RoBorregos"


# HRI Subtask Manager
SKIP_CONFIRMATION_SIMILARITY_THRESHOLD = 0.5
SKIP_CONFIRMATION_CONFIDENCE_THRESHOLD = 0.5
