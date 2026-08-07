"""Unit tests for the self-contained web log viewer package."""

from __future__ import annotations

import math
import unittest
from pathlib import Path

from tools.dogfight_dashboard.server import DEFAULT_ENV_ROOT as DASHBOARD_ENV_ROOT
from tools.web_log_viewer.log_data import (
    angle_between_deg,
    build_viewer_data,
    discover_log_pairs,
    forward_vector,
    in_wez,
    nearest_index,
    tactical_snapshot,
)
from tools.web_log_viewer.server import DEFAULT_ENV_ROOT as VIEWER_ENV_ROOT


FIXTURE_DIR = Path(__file__).resolve().parent / "fixtures"


class LogDataTest(unittest.TestCase):
    """Validate replay parsing without relying on another environment's logs."""

    def test_pair_discovery_and_replay_math(self) -> None:
        pairs = discover_log_pairs(FIXTURE_DIR)
        self.assertEqual(len(pairs), 1)
        ownship, target = pairs[0]
        self.assertIn("[Blue]", ownship.name)
        self.assertIn("[Red]", target.name)

        data = build_viewer_data(ownship, target, None)
        self.assertEqual(nearest_index(data.ownship, 0.5), 0)
        self.assertEqual(nearest_index(data.ownship, 1.2), 1)
        snapshot = tactical_snapshot(data.ownship, data.target, 0.0)
        self.assertGreater(snapshot["distance_m"], 50.0)
        self.assertLess(abs(snapshot["relative_alt_m"]), 1.0)

    def test_vectors_and_wez_contract(self) -> None:
        forward = forward_vector(yaw_deg=90.0, pitch_deg=0.0)
        self.assertAlmostEqual(forward[0], 1.0, places=6)
        self.assertAlmostEqual(forward[1], 0.0, places=6)

        north_climb = forward_vector(yaw_deg=0.0, pitch_deg=30.0)
        self.assertAlmostEqual(north_climb[0], 0.0, places=6)
        self.assertAlmostEqual(north_climb[1], math.sqrt(3.0) / 2.0, places=6)
        self.assertAlmostEqual(north_climb[2], 0.5, places=6)

        east_climb = forward_vector(yaw_deg=90.0, pitch_deg=30.0)
        self.assertAlmostEqual(east_climb[0], math.sqrt(3.0) / 2.0, places=6)
        self.assertAlmostEqual(east_climb[1], 0.0, places=6)
        self.assertAlmostEqual(east_climb[2], 0.5, places=6)

        self.assertTrue(math.isclose(angle_between_deg((1, 0, 0), (0, 1, 0)), 90.0))
        self.assertTrue(in_wez(500.0, 0.5, 100.0, 1_000.0, 2.0))
        self.assertFalse(in_wez(50.0, 0.5, 100.0, 1_000.0, 2.0))
        self.assertFalse(in_wez(500.0, 2.0, 100.0, 1_000.0, 2.0))

    def test_default_environment_root_is_local(self) -> None:
        expected_root = Path(__file__).resolve().parents[3]
        self.assertEqual(DASHBOARD_ENV_ROOT.resolve(), expected_root)
        self.assertEqual(VIEWER_ENV_ROOT.resolve(), expected_root)


if __name__ == "__main__":
    unittest.main()