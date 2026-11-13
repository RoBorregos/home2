"""Conformal quantile and the softmax mass behind APS. No models needed."""

import math

import numpy as np

from core.conformal import mass_before, quantile


def test_quantile_is_the_finite_sample_rank():
    # ceil((9 + 1) * 0.8) = 8th smallest value.
    assert quantile(range(1, 10), alpha=0.2) == 8


def test_quantile_with_too_few_cases_keeps_everything():
    assert quantile([0.1, 0.2], alpha=0.1) == math.inf


def test_coverage_holds_on_exchangeable_data():
    rng = np.random.default_rng(0)
    calibration, test = rng.normal(size=500), rng.normal(size=5000)
    covered = np.mean(test <= quantile(calibration, alpha=0.1))
    assert covered >= 0.9 - 0.02


def test_mass_before_starts_at_zero_and_grows():
    mass = mass_before(np.array([2.0, 1.0, 0.0]), temperature=1.0)
    assert mass[0] == 0.0 and np.all(np.diff(mass) > 0) and mass[-1] < 1.0
