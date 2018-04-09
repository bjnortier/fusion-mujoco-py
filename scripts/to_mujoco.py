import adsk.core, adsk.fusion, traceback
import os.path, sys
import xml.etree.ElementTree as ET
import xml.dom.minidom

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import fusion_mujoco_py

XML_TEMPLATE = """<?xml version="1.0" ?>
<mujoco model="cheetah">
  <compiler angle="radian" coordinate="local" inertiafromgeom="true" settotalmass="14"/>
  <default>
    <joint armature=".1" damping=".01" limited="true" solimplimit="0 .8 .03" solreflimit=".02 1" stiffness="8"/>
    <geom conaffinity="0" condim="3" contype="1" friction=".4 .1 .1" rgba="0.8 0.6 .4 1" solimp="0.0 0.8 0.01" solref="0.02 1"/>
    <motor ctrllimited="true" ctrlrange="-1 1"/>
  </default>
  <size nstack="300000" nuser_geom="1"/>
  <option gravity="0 0 -9.81" timestep="0.01"/>
  <asset>
    <texture builtin="flat" height="1278" mark="cross" markrgb="1 1 1" name="texgeom" random="0.01" rgb1="0.8 0.6 0.4" rgb2="0.8 0.6 0.4" type="cube" width="127"/>
    <texture builtin="checker" height="100" name="texplane" rgb1="0 0 0" rgb2="0.8 0.8 0.8" type="2d" width="100"/>
    <material name="groundplanemat" reflectance="0.5" shininess="1" specular="1" texrepeat="60 60" texture="texplane"/>
    <material name="geom" texture="texgeom" texuniform="true"/>
  </asset>
  <worldbody>
    <light cutoff="100" diffuse="1 1 1" dir="-0 0 -1.3" directional="true" exponent="1" pos="0 0 1.3" specular=".1 .1 .1"/>
    <geom conaffinity="1" condim="3" material="groundplanemat" name="floor" pos="0 0 0" rgba="0.8 0.9 0.8 1" size="40 40 40" type="plane"/>
  </worldbody>
</mujoco>
"""

mujoco = ET.XML(XML_TEMPLATE)
asset = mujoco[4]
worldbody = mujoco[5]
assert asset.tag == 'asset'
assert worldbody.tag == 'worldbody'

def pretty_write(root, filename):
    output = xml.dom.minidom.parseString(ET.tostring(root)).toprettyxml()
    f = open(filename, 'w')
    f.write(output)
    f.close()

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        # get active design
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        # get root component in this design
        rootComp = design.rootComponent
        # create a single exportManager instance
        exportMgr = design.exportManager
        # get the script location
        exportDir = os.path.join(os.path.expanduser('~'), 'fusion-mujoco-export')
        print('exportdir:', exportDir)

        # export the occurrence one by one in the root component to a specified file
        allOccu = rootComp.allOccurrences
        for occ in allOccu:
            stl_filename = os.path.join(exportDir, 'stl', occ.component.name)

            # create stl exportOptions
            stlExportOptions = exportMgr.createSTLExportOptions(occ, stl_filename)
            stlExportOptions.sendToPrintUtility = False

            exportMgr.execute(stlExportOptions)
            mesh = ET.SubElement(asset, 'mesh')
            mesh.set('name', occ.component.name)
            mesh.set('file', 'stl/{0}.stl'.format(occ.component.name))
            mesh.set('scale', '0.1 0.1 0.1')

            body = ET.SubElement(worldbody, 'body')
            body.set('name', occ.component.name)

            (origin, xAxis, yAxis, zAxis) = occ.transform.getAsCoordinateSystem()
            [x, y, z] = origin.asArray()

            geom = ET.SubElement(body, 'geom')
            geom.set('type', 'mesh')
            body.set('pos', '{0} {1} {2}'.format(x, y, z))
            geom.set('mesh', occ.component.name)

        # export the body one by one in the design to a specified file
        allBodies = rootComp.bRepBodies
        for body in allBodies:
            stl_filename = exportDir + "/" + body.parentComponent.name + '-' + body.name

            # create stl exportOptions
            stlExportOptions = exportMgr.createSTLExportOptions(body, stl_filename)
            stlExportOptions.sendToPrintUtility = False

            exportMgr.execute(stlExportOptions)

        pretty_write(mujoco, os.path.join(exportDir, '{0}.xml'.format(rootComp.name)))

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
