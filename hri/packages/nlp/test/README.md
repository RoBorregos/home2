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
```

4. Run the tests
```bash
deepeval test run test_<script>.py
```