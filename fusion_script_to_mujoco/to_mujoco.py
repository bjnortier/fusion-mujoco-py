import adsk.core, adsk.fusion, traceback
import os, sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from fusion_mujoco_py.export import write_stls_and_mojoco_xml

def run(context):
    try:
        app = adsk.core.Application.get()
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        exportDir = '/tmp'
        try:
            os.mkdir(os.path.join(exportDir, 'stl'))
        except FileExistsError as e:
            print('overwriting existing STLs')
        write_stls_and_mojoco_xml(design, exportDir, 'tmp')
        print('written')
    except:
        print('Failed:\n{}'.format(traceback.format_exc()))
