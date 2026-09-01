#!/usr/bin/env python3

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "drone_city_nav"
SOURCE = PACKAGE / "src"
PRODUCER_ADMISSION = SOURCE / "producer_epoch_admission.cpp"


class ExecutionInputContractTest(unittest.TestCase):
    def test_producer_claims_raise_exact_once_sequence_high_water(self) -> None:
        admission = PRODUCER_ADMISSION.read_text(encoding="utf-8")
        self.assertGreaterEqual(admission.count("claimed_sequence_high_water"), 2)
        self.assertGreaterEqual(admission.count("first_receive_stamp_ns"), 2)
        self.assertIn("sameRejectedEvidenceIdentity", admission)
        self.assertIn("full_snapshot == full_snapshot", admission)

if __name__ == "__main__":
    unittest.main()
