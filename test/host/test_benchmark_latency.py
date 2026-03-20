#!/usr/bin/env python3
"""Unit tests for benchmark_latency helpers."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path


TEST_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = TEST_DIR.parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from benchmark_latency import build_request_message


class BenchmarkLatencyTests(unittest.TestCase):
    def test_build_request_message_leaves_prompt_unchanged_by_default(self) -> None:
        self.assertEqual(build_request_message("ping", 3, False), "ping")

    def test_build_request_message_appends_counter_when_enabled(self) -> None:
        self.assertEqual(build_request_message("ping", 3, True), "ping 3")


if __name__ == "__main__":
    unittest.main()
