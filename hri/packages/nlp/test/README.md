# Testing using Deepeval

1. Install the deepeval package
```bash
pip install deepeval
```

2. Run the tests
```bash
deepeval test run test_data_extractor.py
```

```bash
# Set local embeddings for evaluations
deepeval set-local-embeddings --model-name=nomic-embed-text:latest \
    --base-url="http://localhost:11434/v1/" \
    --api-key="ollama"
```