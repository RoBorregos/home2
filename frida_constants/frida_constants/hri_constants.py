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
CATEGORIZE_SERVICE = "/hri/nlp/categorize"
ITEM_CATEGORIZATION_SERVICE = "/hri/nlp/item_categorization"
CONVESATION_SERVICE = "/hri/nlp/conversation"
GRAMMAR_SERVICE = "/hri/nlp/grammar"
COMMON_INTEREST_SERVICE = "/hri/nlp/common_interest"

ADD_ENTRY_SERVICE = "/hri/nlp/embeddings/add_entry_service"
QUERY_ENTRY_SERVICE = "/hri/nlp/embeddings/query_entry_service"
BUILD_EMBEDDINGS_SERVICE = "/hri/nlp/embeddings/build_embeddings_service"
USE_RESPEAKER = True
RESPEAKER_LIGHT_TOPIC = "/hri/speech/respeaker/light"
RESPEAKER_DOA_TOPIC = "/hri/speech/respeaker/doa"


class MODEL(Enum):
    GRAMMAR = "qwen2.5"
    LLM_WRAPPER = "qwen2.5"
    COMMON_INTEREST = "qwen2.5"
    CATEGORIZE_SHELVES = "qwen3"
    IS_POSITIVE = "qwen2.5"
    IS_NEGATIVE = "qwen2.5"
    GENERIC_STRUCTURED_OUTPUT = "qwen2.5"
    GENERATE_RESPONSE = "qwen2.5"
    STRUCTURED_RESPONSE = "qwen3"
    EXTRACT_INFO_REQUESTED = "qwen2.5"
    GET_COMMANDS = "qwen2.5"


USEFUL_AUDIO_NODE_NAME = "useful_audio_node"

# OpenWakeWord
USE_OWW = True
SENSITIVITY_THRESHOLD = 0.2
threshold = 0.6

DEFAULT_HOTWORDS = "Frida RoBorregos"
