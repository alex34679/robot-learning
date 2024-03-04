#!/usr/bin/env python
from __future__ import division
import numpy as np
import rosunit
import unittest

from localization.sensor_model import SingleBeamSensorModel


class TestSensorModel(unittest.TestCase):
    def test_rejects_invalid_params(self):
        self.assertRaises(
            ValueError,
            SingleBeamSensorModel,
            hit_std=0,
            z_short=0,
            z_max=0,
            z_rand=0,
            z_hit=0,
        )
        self.assertRaises(
            ValueError,
            SingleBeamSensorModel,
            hit_std=1.0,
            z_short=0,
            z_max=0,
            z_rand=0,
            z_hit=0,
        )

    def test_cache_shape_is_correct(self):
        sensor_model = SingleBeamSensorModel()
        max_range = 123
        table = sensor_model.precompute_sensor_model(max_range)
        self.assertEqual(
            table.shape,
            (max_range + 1, max_range + 1),
            msg="The shape of the model table should be [max_range + 1, max_range + 1]",
        )

    def test_cache_is_probability(self):
        sensor_model = SingleBeamSensorModel(
            hit_std=2.0,
            z_short=10e2,
            z_max=444,
            z_rand=123e5,
            z_hit=789e4,
        )
        max_range = 100
        table = sensor_model.precompute_sensor_model(max_range)
        np.testing.assert_allclose(
            table.sum(axis=0),
            1.0,
            err_msg="The masses for the different possible ranges given any particular "
            "expected reading d should form a proper distribution",
        )

    def test_compute_sensor_model_at_min(self):
        sensor_model = SingleBeamSensorModel(
            hit_std=0.0,
            z_short=1,
            z_max=0,
            z_rand=0,
            z_hit=0,
        )
        table = sensor_model.precompute_sensor_model(1)
        self.assertEqual(
            1.0,
            table[0, 1],
            msg="An event under the expected should have mass from the 'short' component",
        )

        sensor_model = SingleBeamSensorModel(
            hit_std=0.0,
            z_short=0,
            z_max=0,
            z_rand=7,
            z_hit=0,
        )
        table = sensor_model.precompute_sensor_model(1)
        self.assertEqual(
            1.0,
            table[0, 1],
            msg="An event under the expected should have mass from the 'random' component",
        )

        sensor_model = SingleBeamSensorModel(
            hit_std=0.0,
            z_short=0,
            z_max=7,
            z_rand=0,
            z_hit=0,
        )
        table = sensor_model.precompute_sensor_model(1)
        self.assertEqual(
            0.0,
            table[0, 0],
            msg="An event under the expected should not have mass from the 'max' component",
        )

        sensor_model = SingleBeamSensorModel(
            hit_std=1.0,
            z_short=0,
            z_max=0,
            z_rand=0,
            z_hit=7,
        )
        table = sensor_model.precompute_sensor_model(1)
        self.assertGreater(
            table[0, 1],
            0.0,
            msg="An event under the expected should have mass from the 'hit' component",
        )

    def test_sensor_model_hit_symmetric(self):
        sensor_model = SingleBeamSensorModel(
            hit_std=1.0,
            z_short=0,
            z_max=0,
            z_rand=0,
            z_hit=7,
        )
        table = sensor_model.precompute_sensor_model(10)
        # NOTE(nickswalker5-6-21): This is the more verbose way of doing what np.flip(table) would do by default
        # in new versions of numpy.
        np.testing.assert_allclose(
            table,
            np.flip(np.flip(table, 0), 1),
            err_msg="The 'hit' component should produce a centrosymmetric table; "
            "distribution when we expect the minimum should be the mirror "
            "of the distribution when we expect the maximum",
        )

    def test_compute_sensor_model_at_max(self):
        sensor_model = SingleBeamSensorModel(
            hit_std=0.0,
            z_short=1,
            z_max=0,
            z_rand=0,
            z_hit=0,
        )
        table = sensor_model.precompute_sensor_model(0)
        # The probability of the one event in this model is 0, so we should
        # get back a sign that this isn't a distribution
        self.assertTrue(
            np.isnan(table.item()),
            msg="An event at the sensor max should not have mass from the 'short' "
            "component, so the resulting distribution should be undefined",
        )

        sensor_model = SingleBeamSensorModel(
            hit_std=0.0,
            z_short=0,
            z_max=0,
            z_rand=7,
            z_hit=0,
        )
        table = sensor_model.precompute_sensor_model(0)
        self.assertEqual(
            1.0,
            np.isnan(table.item()),
            msg="An event at the sensor max should not have mass from the 'random' "
            "component, so the resulting distribution should be undefined",
        )

        sensor_model = SingleBeamSensorModel(
            hit_std=0.0,
            z_short=0,
            z_max=7,
            z_rand=0,
            z_hit=0,
        )
        table = sensor_model.precompute_sensor_model(0)
        self.assertEqual(
            1.0,
            table.item(),
            msg="An event at the sensor max should have mass from the 'max' component",
        )

        sensor_model = SingleBeamSensorModel(
            hit_std=1.0,
            z_short=0,
            z_max=0,
            z_rand=0,
            z_hit=7,
        )
        table = sensor_model.precompute_sensor_model(0)
        self.assertEqual(
            1.0,
            table.item(),
            msg="An event at the sensor max should have mass from the 'hit' component",
        )


if __name__ == "__main__":
    rosunit.unitrun("localization", "test_sensor_model", TestSensorModel)
