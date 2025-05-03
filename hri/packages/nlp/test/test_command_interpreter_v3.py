import sys
import time

sys.path.append("..")
import ast
import json
import re

# from baml_client.sync_client import b
from deepeval.test_case import LLMTestCase
from nlp.assets.dialogs import format_response
from nlp.assets.schemas import CommandListShape
from openai import OpenAI
from tqdm import tqdm

from config import API_KEY, BASE_URL, MODEL, TEMPERATURE
from metrics.json_embedding_similarity import JsonEmbeddingSimilarity

STARTING_CASE = 1


def format_json(data):
    try:
        return json.loads(data), True
    except (TypeError, ValueError):
        return json.loads(json.dumps(data)), False


def remove_extra_arguments(json_response):
    actions_no_complement = ["place", "ask_answer_question", "give"]
    actions_no_characteristic = ["go", "pick", "place", "say", "ask_answer_question", "give", "find_person_info", "find_person_by_name"]

    for command in json_response.get("commands", []):
        action = command.get("action", "")

        # Remove complement for specific actions
        if action in actions_no_complement:
            command["complement"] = ""

        # Remove characteristic for specific actions
        if action in actions_no_characteristic:
            command["characteristic"] = ""

    return json.dumps(json_response)


def extract_answer(response):
    extracted = re.search("<think>(.+?)<\/think>\s*<answer>(.+?)<\/answer>", response, re.DOTALL)
    print(extracted)

    if extracted:
        json_response, parsed = format_json(extracted.group(2))

        if parsed:
            json_response = remove_extra_arguments(json_response)
            return json_response, extracted.group(1)

    # Retry extraction by removing <think> tags
    extracted = re.search("<think>(.+?)<\/think>\s*(.+?)", response, re.DOTALL)
    print(extracted)

    if extracted:
        json_response, parsed = format_json(extracted.group(2))

        if parsed:
            print(f"parsed by removing thinking {extracted.group(2)}")
            json_response = remove_extra_arguments(json_response)
            return json_response, extracted.group(1)

    # Retry by asking llm to format response
    structured = structured_response(response, CommandListShape)
    json_response, parsed = format_json(structured)
    if parsed:
        print(f"formatted by llm {structured}")
        json_response = remove_extra_arguments(json_response)
        return json_response, None

    print("unable to parse")
    print(response)
    return response, None


# Sample function to test
def generate_response(full_text, two_steps=False):
    client = OpenAI(api_key=API_KEY, base_url=BASE_URL)
    # messages = get_command_interpreter_args(full_text)

    # print(messages)
    start = time.time()
    response = client.beta.chat.completions.parse(model=MODEL, temperature=TEMPERATURE, messages=[{"role": "user", "content": full_text}])
    # response = b.GenerateCommandList(full_text)
    end = time.time()
    print(f"Time waiting for response: {end - start}")

    print(response)

    # content, thinking = extract_answer(response.choices[0].message.content)

    # if end - start > 60:
    #     print(f'Spent over one minute on generation, thinking: {thinking}')

    # if two_steps:
    #     content = structured_response(content, CommandListShape)
    #     print("----------------------------------------------")
    #     print("Structured response:\n", content)
    #     return content, response.choices[0].message.content

    # str_response = json.dumps(response.model_dump())
    # Change ' to " and None to null
    # str_response = str_response.replace("'", '"').replace("None", "null")
    # return str_response
    return response, response.choices[0].message.content


def structured_response(response, response_format):
    client = OpenAI(api_key=API_KEY, base_url=BASE_URL)
    formatted_response = client.beta.chat.completions.parse(
        model=MODEL,
        temperature=TEMPERATURE,
        messages=format_response(response),
        response_format=response_format,
    )
    return formatted_response.choices[0].message.content


with open("data/command_interpreter_v3_paraphrased.json") as f:
    command_dataset = json.load(f)

test_cases = [(command["string_cmd"], json.dumps(command["structured_cmd"])) for command in command_dataset[STARTING_CASE:]]


# Define test cases
test_cases = [LLMTestCase(input=test_case[0], expected_output=test_case[1], actual_output="") for test_case in test_cases]

THRESHOLD = 0.7
metric = JsonEmbeddingSimilarity(embeddings_keys=["complement"], threshold=THRESHOLD)
count = 0
wrong_cases = []

for test_case in tqdm(test_cases, desc="Generating test case responses"):
    test_case.actual_output = generate_response(test_case.input, two_steps=False)[1]  # json.dumps()
    test_case.actual_output = json.dumps(ast.literal_eval(test_case.actual_output))  # generate_response(test_case.input, two_steps=False)[1]  # json.dumps()
    # print("test_case.actual_output")
    # print(json.dumps(ast.literal_eval(test_case.actual_output), indent=2))
    # print(json.loads(json.dumps(test_case.actual_output)))
    # test_case.actual_output = json.loads(json.dumps(test_case.actual_output, indent=2))

    a = metric.measure(test_case)
    if metric.measure(test_case) < THRESHOLD:
        count += 1
        print("\033[91mTest case failed:\033[0m")
        print("\033[94minput:\033[0m", test_case.input)
        print("\033[93mexpected_output:\033[0m", format_json(test_case.expected_output)[0])
        print("\033[93mactual_output:\033[0m", format_json(test_case.actual_output)[0])
        # print("\033[93mfull_response:\033[0m", full_response)
        print("\n\n")

        case = f"""
----------------------------------------------------
Input: {test_case.input}
Expected output: {format_json(test_case.expected_output)[0]}
Actual output: {format_json(test_case.actual_output)[0]}

"""
        wrong_cases.append(case)
        with open("data/command_interpreter_wrong_cases.txt", "a") as f:
            f.write(case)

if count == 0:
    print(f"\n\033[92mAll test cases passed ({len(test_cases) - count}/{len(test_cases)})!\033[0m")
else:
    print(f"\n\033[91m{count} test cases failed out of {len(test_cases)}\033[0m")
    print("Wrong cases saved to data/command_interpreter_wrong_cases.txt")
