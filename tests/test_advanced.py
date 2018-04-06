# -*- coding: utf-8 -*-

from .context import fusion_mujoco_py

import unittest


class AdvancedTestSuite(unittest.TestCase):
    """Advanced test cases."""

    def test_thoughts(self):
        self.assertIsNone(fusion_mujoco_py.hmm())


if __name__ == '__main__':
    unittest.main()
