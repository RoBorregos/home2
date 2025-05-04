# Testing BAML command interpreter
1. Install the BAML package and sentence transformers (for embeddings):
```bash
pip install -r requirements.txt
```
2. Install the Baml VS Code extension.
3. In `client.baml` add the desired LLMs either via API or local.
4. Set the client object in the function `GenerateCommandList` inside `robot_commands.baml`
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

As a recommendation, you should use a python3 venv.

1. Install the deepeval package
```bash
# pwd -> hri/packages/nlp/test
pip install -r requirements.txt
```

2. Run the ollama server
```bash
# pwd-> docker/hri
docker compose -f ollama.yaml up
```

3. Set local embeddings for evaluations

```bash
# Set local embeddings for evaluations
deepeval set-local-embeddings --model-name=nomic-embed-text:latest \
    --base-url="http://localhost:11434/" \
    --api-key="ollama"

# Alternatively, you can set the an endpoint for the embeddings:
# deepeval set-local-embeddings --model-name=nomic-embed-text:latest \
#     --base-url="http://<orin_ip>:11434/" \
#     --api-key="ollama" \

```

4. Run the tests
```bash
deepeval test run test_<script>.py
```

Alternatively, some scripts can be run directly (i.e., `test_command_interpreter.py`):
```bash
python test_<script>.py
```
