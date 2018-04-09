# -*- coding: utf-8 -*-
import xml.etree.ElementTree as ET
from . import helpers

def get_hmm():
    """Get a thought."""
    return 'hmmm...'


def hmm():
    """Contemplation..."""
    if helpers.get_answer():
        print(get_hmm())

mujoco = ET.Element('mujoco')
compiler = ET.SubElement(mujoco, 'compiler')
compiler.set('angle', 'radian')
compiler.set('coordinate', 'local')
compiler.set('inertiafromgeom', 'true')

def to_xml(model_name):
    mujoco.set('model', model_name)
    return ET.tostring(mujoco, encoding='unicode')
