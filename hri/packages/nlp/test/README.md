# Testing BAML command interpreter

1. Install the BAML package and sentence transformers (for embeddings):

```bash
pip install -r requirements.txt
```

2. Install the Baml VS Code extension.
3. In `client.baml` add the desired LLMs either via API or local.
4. Set the client object in the function `GenerateCommandListV3` inside `commands_v3/robot_commands.baml`
5. Save the file and the client will be generated (a notification will appear).
6. If an API Key was set in BAML, make sure to add it as an env var in the terminal:

```bash
export OPENROUTER_API_KEY="super-secret-not-from-work-api-key"
export OPENAI_API_KEY="ivan-key-xd"
```

7. Run the test file:

```bash
python3 baml_test_command_interpreter.py
```

# Testing using Deepeval

As a recommendation, you should use a python3 venv. In case of assertion errors upgrade pip: `pip install --upgrade pip setuptools wheel`. Remember to change <> variables for the correct options in the commands.

1. Install the deepeval package

```bash
# pwd -> hri/packages/nlp/test
pip install -r requirements.txt
```

2. Run the ollama server

```bash
# pwd -> docker/hri/compose
COMPOSE_PROFILES=<available_profile> docker compose -f ollama-<env_type>.yaml up
```

3. Set local embeddings for evaluations

```bash
# Set local embeddings for evaluations
deepeval set-local-embeddings --model-name=nomic-embed-text:latest \
    --base-url="http://<localhost/ip>:11434/" \
    --api-key="ollama"
```

4. If you're running the ollama server in another device edit extract_data.yaml, llm_utils.yaml and config.py to point to it by changing localhost for the IP.

5. Run the tests

```bash
deepeval test run test_<script>.py
```
