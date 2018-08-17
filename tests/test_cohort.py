# -*- coding: utf-8 -*-
from .context import fusion_mujoco_py

def test_make_combinations():
    all_values = [
        ('foo', [1, 2]),
        ('bar', [7]),
        ('baz', [17, 23])
    ]
    assert(
        fusion_mujoco_py.make_combinations(all_values)
        ==
        [(('foo', 1), ('bar', 7), ('baz', 17)),
         (('foo', 1), ('bar', 7), ('baz', 23)),
         (('foo', 2), ('bar', 7), ('baz', 17)),
         (('foo', 2), ('bar', 7), ('baz', 23))])
