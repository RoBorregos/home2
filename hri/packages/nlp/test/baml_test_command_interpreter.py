import sys
import json
import time
from typing import List, Optional

# Add parent directory to path if needed (adjust based on your structure)
# sys.path.append("..") 

from pydantic import BaseModel, Field
from baml_client.sync_client import b
from baml_client.types import CommandListLLM_V3
# We'll need sentence-transformers for cosine similarity later
from sentence_transformers import SentenceTransformer
from scipy.spatial.distance import cosine
from tqdm import tqdm
import numpy as np
from baml_client.config import set_log_level

# Turn off all logging
set_log_level("ERROR")


# --- Configuration ---
STARTING_CASE = 0 # Adjust if needed
SIMILARITY_THRESHOLD = 0.8 # Threshold for complement similarity
OVERALL_THRESHOLD = 0.75 # Threshold for the overall test case score
TEST_DATA_FILE = "data/command_interpreter_v3.json"
EMBEDDING_MODEL = 'all-MiniLM-L6-v2' # Example model


# --- Helper Functions ---

def parse_expected_output(json_string) -> Optional[CommandListLLM_V3]:
    """Parses the JSON string from the dataset into an ExpectedCommandList model."""
    try:
        # Replace single quotes and None representation if necessary
        # Handle potential variations in JSON string format
        data = {"commands": json_string}
        return CommandListLLM_V3(**data)
    except (json.JSONDecodeError, TypeError, ValueError) as e:
        print(f"Error parsing JSON string: {json_string}\nError: {e}")
        return None

# --- Comparison Logic ---

# Load the embedding model globally
print(f"Loading embedding model: {EMBEDDING_MODEL}...")
try:
    model = SentenceTransformer(EMBEDDING_MODEL)
    print("Embedding model loaded.")
except Exception as e:
    print(f"\x1B[91mError loading sentence transformer model '{EMBEDDING_MODEL}': {e}\x1B[0m")
    print("\x1B[93mPlease ensure 'sentence-transformers' and 'torch' (or 'tensorflow') are installed.\x1B[0m")
    print("Cosine similarity will default to exact string matching.")
    model = None

def calculate_cosine_similarity(text1: Optional[str], text2: Optional[str]) -> float:
    """Calculates cosine similarity between two texts using an embedding model.
    Returns 1.0 for perfect match (including both None), 0.0 if only one is None,
    and cosine similarity if both are strings and model is loaded, otherwise exact match.
    """
    if text1 is None and text2 is None:
        return 1.0
    if text1 is None or text2 is None:
        return 0.0
    # Now we know both text1 and text2 are strings
    if model:
        try:
            # Ensure texts are not empty strings, as encode might handle them differently
            if not text1 or not text2:
                 return 1.0 if text1 == text2 else 0.0
                 
            embeddings = model.encode([text1, text2])
            # Ensure embeddings are 2D numpy arrays for cosine distance
            if embeddings.ndim == 1:
                # This case might happen if encode returns a single vector for short/empty strings depending on the model
                # We should handle based on specific model behavior, but reshaping assumes two vectors were intended.
                 embeddings = embeddings.reshape(2, -1) # Adjust if model behaves differently
            
            # Check if embeddings have the expected shape (2, embedding_dim)
            if embeddings.shape[0] != 2:
                print(f"Warning: Unexpected embedding shape {embeddings.shape} for texts: '{text1}', '{text2}'")
                return 1.0 if text1 == text2 else 0.0 # Fallback to exact match

            # scipy.spatial.distance.cosine returns the distance (1 - similarity)
            similarity = 1 - cosine(embeddings[0], embeddings[1])
            # Handle potential NaN results from cosine if vectors are zero, etc.
            return float(np.nan_to_num(similarity)) 
        except Exception as e:
            print(f"Error calculating similarity for '{text1}' vs '{text2}': {e}")
            # Fallback to exact match on error
            return 1.0 if text1 == text2 else 0.0
    else:
        # Fallback to exact string match if model wasn't loaded
        print(f"WARN: Embedding model not loaded. Comparing '{text1}' vs '{text2}' as exact match.")
        return 1.0 if text1 == text2 else 0.0


def compare_commands(actual: CommandListLLM_V3, expected: CommandListLLM_V3) -> float:
    """Compares the actual BAML output with the expected output."""
    if len(actual.commands) != len(expected.commands):
        print(f"Mismatch in number of commands: Actual={len(actual.commands)}, Expected={len(expected.commands)}")
        return 0.0

    if not actual.commands: # Handle empty command list case
        return 1.0

    scores = []
    for i, (act_cmd, exp_cmd) in enumerate(zip(actual.commands, expected.commands)):
        command_score = 0.0
        action_match = False
        similarity = 0.0

        # Compare Action (Enum vs String)
        # Convert Enum member to its value (string representation)
        if act_cmd.action == exp_cmd.action:
            action_match = True
            # Compare Complement using Cosine Similarity (handles None)
            similarity = calculate_cosine_similarity(str(act_cmd), str(exp_cmd))

            # Score for this command: Weighted average? Let's try 40% action, 60% complement
            # command_score = 0.4 + (0.6 * similarity) if similarity >= 0 else 0.0 # Ensure similarity isn't negative
            
            # Simpler: 1 if action matches and similarity is above threshold, 0 otherwise?
            # Let's stick to the average for now, gives more granular scoring.
            command_score = (1.0 + similarity) / 2.0 # Simple average of action (1) and similarity

        else:
            print(f"Command {i}: Action mismatch - Actual='{act_cmd.action}', Expected='{exp_cmd.action}'")
            command_score = 0.0 # Action mismatch means 0 score for the command

        scores.append(command_score)
        print(f"  Cmd {i}: Act='{act_cmd.action}' vs Exp='{exp_cmd.action}' -> ActionMatch={action_match}, Sim={similarity:.2f}, Score={command_score:.2f}")

    overall_score = np.mean(scores) if scores else 1.0
    return overall_score


# --- Main Test Execution ---

def run_tests():
    """Loads data, runs tests, and reports results."""
    # --- Model Name Input ---
    model_name = input("Model name for test run (for logging, set the actual client in `GenerateCommandListV3`): ")
    print(f"Testing with model: {model_name}")
    # --- End Model Name Input ---

    print(f"Loading test data from: {TEST_DATA_FILE}")
    try:
        with open(TEST_DATA_FILE, "r") as f:
            command_dataset = json.load(f)
    except FileNotFoundError:
        print(f"Error: Test data file not found at {TEST_DATA_FILE}")
        return
    except json.JSONDecodeError:
        print(f"Error: Could not decode JSON from {TEST_DATA_FILE}")
        return

    test_cases = [
        (
            command["string_cmd"],
            command["structured_cmd"]
        )
        for command in command_dataset[STARTING_CASE:] # Adjust slice as needed
    ]

    print(f"Loaded {len(test_cases)} test cases.")

    passed_count = 0
    failed_cases = []
    execution_times = [] # <-- Add list to store times

    for i, (input_str, expected_str) in enumerate(tqdm(test_cases, desc="Running BAML tests")):
        # TODO: Remove delay for local testing, it was used to avoid rate limiting
        #time.sleep(3)
        print(f"\n--- Test Case {STARTING_CASE + i} ---")
        print(f"Input: {input_str}")

        expected_command_list = parse_expected_output(expected_str)
        if not expected_command_list:
            print(" \x1B[91mFailed (Skipping due to parse error in expected output)\x1B[0m")
            failed_cases.append({
                "index": STARTING_CASE + i,
                "input": input_str,
                "expected_str": expected_str,
                "reason": "Failed to parse expected output JSON."
            })
            continue


        try:
            start_time = time.time()
            # Call the BAML function
            actual_command_list = b.GenerateCommandListV3(request=input_str)
            end_time = time.time()
            duration = end_time - start_time # <-- Calculate duration
            execution_times.append(duration) # <-- Store duration
            print(f"Expected: {expected_command_list.model_dump_json(indent=2)}")
            print(f"BAML Response ({duration:.2f}s): {actual_command_list.model_dump_json(indent=2)}")

            # Compare results
            score = compare_commands(actual_command_list, expected_command_list)
            print(f"Comparison Score: {score:.3f}")

            if score >= OVERALL_THRESHOLD:
                print(f" \x1B[92mPassed (Score: {score:.3f} >= {OVERALL_THRESHOLD})\x1B[0m")
                passed_count += 1
            else:
                print(f" \x1B[91mFailed (Score: {score:.3f} < {OVERALL_THRESHOLD})\x1B[0m")
                failed_cases.append({
                    "index": STARTING_CASE + i,
                    "input": input_str,
                    "expected_str": expected_str,
                    "expected_parsed": expected_command_list.model_dump(),
                    "actual": actual_command_list.model_dump(),
                    "score": score
                })

        except Exception as e:
            print(f" \x1B[91mFailed (Error during BAML call or comparison): {e}\x1B[0m")
            failed_cases.append({
                "index": STARTING_CASE + i,
                "input": input_str,
                "expected_str": expected_str,
                "reason": f"Runtime error: {e}"
            })
            # Ensure duration is recorded even on error if start_time was set
            if 'start_time' in locals():
                 duration = time.time() - start_time
                 execution_times.append(duration)
        
        # Optional: Add delay between API calls if needed
        # time.sleep(1) 


    if failed_cases:
        print("\n--- Failed Cases ---")
        for case in failed_cases:
            print(f"Index: {case['index']}")
            print(f"  Input: {case['input']}")
            print(f"  Expected (raw): {case['expected_str']}")
            if "reason" in case:
                print(f"  Reason: {case['reason']}")
            else:
                print(f"  Expected (parsed): {json.dumps(case['expected_parsed'], indent=2)}")
                print(f"  Actual: {json.dumps(case['actual'], indent=2)}")
                print(f"  Score: {case['score']:.3f}")
            print("-" * 20)
        # Optionally write failed cases to a file
        # with open("baml_test_failures.json", "w") as f:
        #     json.dump(failed_cases, f, indent=2)
        # print("Failed cases saved to baml_test_failures.json")
    
    # --- Reporting ---
    print("\n--- Test Summary ---")
    total_cases = len(test_cases)
    print(f"Total Cases: {total_cases}")
    print(f" \x1B[92mPassed: {passed_count}\x1B[0m")
    print(f" \x1B[91mFailed: {len(failed_cases)}\x1B[0m")
    
    # Calculate average time
    average_time = np.mean(execution_times) if execution_times else 0
    print(f"Average Time per Case: {average_time:.2f}s")
    print(f"Model Tested: {model_name}") # <-- Print model name


if __name__ == "__main__":
    # Ensure BAML client is initialized (adjust if you have a central init)
    # You might need to configure your BAML client here if it's not automatic
    # Example: b.configure(api_key="YOUR_API_KEY") 
    run_tests()
