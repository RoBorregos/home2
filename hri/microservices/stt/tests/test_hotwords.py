"""Run with: python3 -m unittest discover -s hri/microservices/stt/tests."""

import ast
from pathlib import Path
import sys
import unittest
import warnings

STT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STT_DIR))

from hotwords import build_context_tokens, normalize_hotwords, parse_hotwords  # noqa: E402


class CharacterTokenizer:
    """Deterministic token accounting without downloading a Whisper model."""

    sot_prev = -1
    sot_sequence = [-2, -3, -4]
    no_timestamps = -5
    timestamp_begin = -6

    def encode(self, text):
        return list(map(ord, text))


class HotwordTests(unittest.TestCase):
    def setUp(self):
        self.tokenizer = CharacterTokenizer()

    def test_normalization_preserves_phrases_and_priority(self):
        self.assertEqual(
            normalize_hotwords("  Mary  Jane;orange juice\nMARY JANE, Kuat,,"),
            "Mary Jane, orange juice, Kuat",
        )
        self.assertEqual(parse_hotwords("Mary Jane"), ["Mary Jane"])
        self.assertEqual(parse_hotwords(None), [])
        self.assertEqual(parse_hotwords(" , ;\n"), [])
        self.assertEqual(parse_hotwords("Jose\u0301, José"), ["José"])

    def test_joint_budget_preserves_recent_history(self):
        history = list(range(1000, 1400))
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            result = build_context_tokens(
                self.tokenizer, history, "Frida, " + "x" * 300, 223
            )
        self.assertEqual(len(result), 223)
        self.assertEqual(result[:6], self.tokenizer.encode(" Frida"))
        self.assertEqual(result[6:], history[-217:])

    def test_never_cuts_a_phrase_and_warns_for_dropped_hints(self):
        with self.assertWarns(RuntimeWarning):
            result = build_context_tokens(
                self.tokenizer, [], "Mary Jane, Alexandria, Kuat", 16
            )
        self.assertEqual(result, self.tokenizer.encode(" Mary Jane, Kuat"))

    def test_history_without_hints_matches_original_tail(self):
        self.assertEqual(
            build_context_tokens(self.tokenizer, [1, 2, 3], None, 2), [2, 3]
        )

    def test_exact_budget_and_empty_context(self):
        self.assertEqual(
            build_context_tokens(self.tokenizer, [], "Frida", 6),
            self.tokenizer.encode(" Frida"),
        )
        self.assertEqual(build_context_tokens(self.tokenizer, [1], "Frida", 0), [])
        self.assertEqual(build_context_tokens(self.tokenizer, [], "", 223), [])

    def test_production_prompt_assembly(self):
        # Execute the actual method in isolation: importing the vendored module
        # requires CUDA/ASR packages that are not needed to test token assembly.
        module = ast.parse((STT_DIR / "transcriber_faster_whisper.py").read_text())
        model = next(
            node
            for node in module.body
            if isinstance(node, ast.ClassDef) and node.name == "WhisperModel"
        )
        method = next(
            node
            for node in model.body
            if isinstance(node, ast.FunctionDef) and node.name == "get_prompt"
        )
        isolated = ast.Module(body=[method], type_ignores=[])
        namespace = {
            "build_context_tokens": build_context_tokens,
            "Tokenizer": object,
            "List": list,
            "Optional": __import__("typing").Optional,
        }
        exec(compile(isolated, "get_prompt", "exec"), namespace)
        owner = type("Model", (), {"max_length": 448})()
        get_prompt = namespace["get_prompt"]
        prompt = get_prompt(owner, self.tokenizer, [900] * 223, hotwords="Frida")
        self.assertEqual(len(prompt), 227)  # 223 context + sot_prev + 3 start tokens
        self.assertEqual(prompt[-3:], self.tokenizer.sot_sequence)
        self.assertEqual(prompt[1:7], self.tokenizer.encode(" Frida"))
        # Prefix must still disable hotwords and retain timestamp behavior.
        prompt = get_prompt(owner, self.tokenizer, [], prefix="Hi", hotwords="Frida")
        self.assertEqual(prompt, [-2, -3, -4, -6] + self.tokenizer.encode(" Hi"))
        self.assertEqual(
            get_prompt(owner, self.tokenizer, [], without_timestamps=True),
            [-2, -3, -4, -5],
        )


if __name__ == "__main__":
    unittest.main()
