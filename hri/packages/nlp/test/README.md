# NLP tests

## How to run

This directory holds json files with tests for different `hri_tasks.py` functions and are used in `test_hri_manager.py`. Modify the constants in the script to select which tests to run.

```bash
# pwd -> home2
./run.sh integration --test-hri --build
```

Most tests will output a report with the results here in `output/`.
