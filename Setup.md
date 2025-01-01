# Project setup

## Pre-commit

[Pre-commit](https://pre-commit.com/) is a framework for managing and maintaining multi-language pre-commit hooks. It is a useful tool to ensure that code is formatted and linted before committing. The following hooks are used in this project:

- [ruff](https://github.com/astral-sh/ruff-pre-commit/tree/main) (linter and formatter for python)

To install pre-commit and the repo's hooks, run the following commands:

```bash
pip install pre-commit
pre-commit install
```

To run against all files (useful when adding new hooks), use:

```bash
pre-commit run --all-files
```

## Ruff

Ruff is a linter and formatter for python. It is used to mantain consistency and readability in the codebase. Ruff in installed as a pre-commit hook, but can also be run manually. To install ruff, run the following command:

```bash
pip install ruff
```

To run ruff, use the following command:

```bash
ruff check
```

### Vscode integration

If you are using vscode, you can add the following in `.vscode/settings.json` to enable ruff formatting on save. For this to work, you must also install the [Ruff extension for vscode](https://marketplace.visualstudio.com/items?itemName=charliermarsh.ruff).

```json
{
  "[python]": {
    "editor.formatOnSave": true,
    "editor.codeActionsOnSave": {
      "source.fixAll": "explicit",
      "source.organizeImports": "explicit"
    },
    "editor.defaultFormatter": "charliermarsh.ruff"
  }
}
```
