import adsk.core, adsk.fusion, traceback
import os.path, sys
import xml.etree.ElementTree as ET
import xml.dom.minidom
import re

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

XML_TEMPLATE = """<?xml version="1.0" ?>
<mujoco>
  <compiler angle="radian" coordinate="local" inertiafromgeom="true" settotalmass="14"/>
  <default>
    <joint armature=".1" damping=".01" limited="true" solimplimit="0 .8 .03" solreflimit=".02 1" stiffness="8" range="-3.1415926536 3.1415926536" />
    <geom rgba="0.2 0.8 0.2 0.5" />
    <site type="sphere" rgba=".9 .9 .9 1" size="0.1"/>
    <motor ctrllimited="true" ctrlrange="-1 1"/>
  </default>
  <size nstack="300000" nuser_geom="1"/>
  <option gravity="0 0 -9.81" timestep="0.01"/>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1=".4 .5 .6" rgb2="0 0 0" width="100" height="100"/>
    <texture builtin="flat" height="1278" mark="cross" markrgb="1 1 1" name="texgeom" random="0.01" rgb1="0.8 0.6 0.4" rgb2="0.8 0.6 0.4" type="cube" width="127"/>
    <texture builtin="checker" height="100" name="texplane" rgb1="0 0 0" rgb2="0.8 0.8 0.8" type="2d" width="100"/>
    <material name="groundplanemat" reflectance="0.5" shininess="1" specular="1" texrepeat="100 100" texture="texplane"/>
    <material name="geom" texture="texgeom" texuniform="true"/>
  </asset>
  <worldbody>
    <light cutoff="100" diffuse="1 1 1" dir="-0 0 -1.3" directional="true" exponent="1" pos="0 0 1.3" specular=".1 .1 .1"/>
    <geom conaffinity="1" condim="3" name="floor" pos="0 0 0" rgba="0.8 0.9 0.8 1" size="1000 1000 1000" type="plane" material="groundplanemat"/>
  </worldbody>
  <tendon />
  <actuator />
</mujoco>"""

mujoco = ET.XML(XML_TEMPLATE)
asset = mujoco[4]
worldbody = mujoco[5]
tendon = mujoco[6]
actuator = mujoco[7]
assert asset.tag == 'asset'
assert worldbody.tag == 'worldbody'
assert tendon.tag == 'tendon'
assert actuator.tag == 'actuator'

def pretty_write(root, filename):
    output = xml.dom.minidom.parseString(re.sub('\n\s*', '', str(ET.tostring(root), 'utf-8'))).toprettyxml()
    f = open(filename, 'w')
    f.write(output)
    f.close()

def append_root_join_elements(body):
    joint = ET.Element('joint')
    joint.set('armature', '0')
    joint.set('axis', '1 0 0')
    joint.set('damping', '0')
    joint.set('limited', 'false')
    joint.set('name', 'rootx')
    joint.set('pos', '0 0 0')
    joint.set('stiffness', '0')
    joint.set('type', 'slide')
    body.append(joint)
    joint = ET.Element('joint')
    joint.set('armature', '0')
    joint.set('axis', '0 1 0')
    joint.set('damping', '0')
    joint.set('limited', 'false')
    joint.set('name', 'rooty')
    joint.set('pos', '0 0 0')
    joint.set('stiffness', '0')
    joint.set('type', 'slide')
    body.append(joint)
    joint = ET.Element('joint')
    joint.set('armature', '0')
    joint.set('axis', '0 0 1')
    joint.set('damping', '0')
    joint.set('limited', 'false')
    joint.set('name', 'rootz')
    joint.set('pos', '0 0 0')
    joint.set('stiffness', '0')
    joint.set('type', 'slide')
    body.append(joint)
    joint = ET.Element('joint')
    joint.set('armature', '0')
    joint.set('axis', '1 0 0')
    joint.set('damping', '0')
    joint.set('limited', 'false')
    joint.set('name', 'rootpitch')
    joint.set('pos', '0 0 0')
    joint.set('stiffness', '0')
    joint.set('type', 'hinge')
    body.append(joint)
    joint = ET.Element('joint')
    joint.set('armature', '0')
    joint.set('axis', '0 1 0')
    joint.set('damping', '0')
    joint.set('limited', 'false')
    joint.set('name', 'rootroll')
    joint.set('pos', '0 0 0')
    joint.set('stiffness', '0')
    joint.set('type', 'hinge')
    body.append(joint)
    joint = ET.Element('joint')
    joint.set('armature', '0')
    joint.set('axis', '0 0 1')
    joint.set('damping', '0')
    joint.set('limited', 'false')
    joint.set('name', 'rootyaw')
    joint.set('pos', '0 0 0')
    joint.set('stiffness', '0')
    joint.set('type', 'hinge')
    body.append(joint)

def create_xml_body(occ):
    xml_body = ET.Element('body')
    xml_body.set('name', occ.component.name)
    (origin, xAxis, yAxis, zAxis) = occ.transform.getAsCoordinateSystem()
    [x, y, z] = origin.asArray()
    xml_body.set('pos', '{0} {1} {2}'.format(round(x, 3), round(y, 3), round(z, 3)))
    geom = ET.SubElement(xml_body, 'geom')
    geom.set('type', 'mesh')
    geom.set('mesh', occ.component.name)
    return xml_body

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        rootComp = design.rootComponent
        mujoco.set('model', rootComp.name)
        exportMgr = design.exportManager
        exportDir = os.path.join(os.path.expanduser('~'), 'fusion-mujoco-export')
        print('exportdir:', exportDir)

        # export the occurrence one by one in the root component to a specified file
        allOccu = rootComp.allOccurrences
        for occ in allOccu:
            stl_filename = os.path.join(exportDir, 'stl', occ.component.name)
            stlExportOptions = exportMgr.createSTLExportOptions(occ, stl_filename)
            stlExportOptions.sendToPrintUtility = False
            # stlExportOptions.isBinaryFormat = False
            exportMgr.execute(stlExportOptions)
            mesh = ET.SubElement(asset, 'mesh')
            mesh.set('name', occ.component.name)
            mesh.set('file', 'stl/{0}.stl'.format(occ.component.name))
            mesh.set('scale', '1 1 1')

        xml_body_for_occ = {}
        for occ in allOccu:
            xml_body = create_xml_body(occ)
            xml_body_for_occ[occ.component.name] = xml_body

        # Grounded component in Fusion 360 = Root components in Mujoco.
        # There can be only one
        found = False
        for occ in allOccu:
            if (occ.isGrounded):
                assert found == False
                root_body = xml_body_for_occ[occ.component.name]
                append_root_join_elements(root_body)
                worldbody.append(root_body)
                found = True
            else:
                # (origin, xAxis, yAxis, zAxis) = occ.transform.getAsCoordinateSystem()
                # [x, y, z] = .asArray()
                print(occ.component.name, occ.transform.translation.x, occ.transform.translation.y, occ.transform.translation.z)
                # print(occ.transform.scale)
        #
        # Joints
        for joint in rootComp.joints:
            parent_occ = joint.occurrenceTwo
            child_occ = joint.occurrenceOne
            if not child_occ.isVisible:
                continue
            parent = xml_body_for_occ[parent_occ.component.name]
            child_xml_body = xml_body_for_occ[child_occ.component.name]

            (origin1, xAxis1, yAxis1, zAxis1) = parent_occ.transform.getAsCoordinateSystem()
            (origin2, xAxis2, yAxis2, zAxis2) = child_occ.transform.getAsCoordinateSystem()
            [x1, y1, z1] = origin1.asArray()
            [x2, y2, z2] = origin2.asArray()
            x1 *= 10
            y1 *= 10
            z1 *= 10
            x2 *= 10
            y2 *= 10
            z2 *= 10

            local_x = x2 - x1
            local_y = y2 - y1
            local_z = z2 - z1
            # print('xAxis', xAxis2.asArray())
            # print('yAxis', yAxis2.asArray())
            # print('zAxis', zAxis2.asArray())
            # print('parent', x1, y1, z1, 'child', x2, y2, z2)
            child_xml_body.set('pos', '{0} {1} {2}'.format(round(local_x, 3), round(local_y, 3), round(local_z, 3)))
            parent.append(child_xml_body)

            # Fusion Revolute joint create Mujoco Hinge joints
            if joint.jointMotion.jointType == 1:
                print('----- 1:', joint.occurrenceOne.name, '>> 2:', joint.occurrenceTwo.name, '-----')
                print('1:', joint.geometryOrOriginOne.origin.asArray())
                [x, y, z] = joint.geometryOrOriginOne.origin.asArray()
                x *= 10
                y *= 10
                z *= 10
                joint_xml = ET.SubElement(child_xml_body, 'joint')
                joint_xml.set('axis', '{0} {1} {2}'.format(*joint.geometryOrOriginOne.primaryAxisVector.asArray()))
                joint_xml.set('name', '{0}'.format(joint.name))
                radius = 10
                mujuco_joint_pos = '{0} {1} {2}'.format(round(x - local_x - radius, 3), round(y - local_y, 3), round(z - local_z, 3))
                joint_xml.set('pos', mujuco_joint_pos)
                joint_xml.set('type', 'hinge')

                marker = ET.fromstring('<body pos="{0}"><geom pos="0 0 0" type="sphere" size="5" rgba="1.0 1.0 1.0 1"/></body>'.format(mujuco_joint_pos))
                child_xml_body.append(marker)
        #
        #
        # # Tendons sites
        # link_positions = {}
        # for joint in rootComp.joints:
        #     # Model "links" as "tendons" in mujoco
        #     if re.match('__Link__.*', joint.occurrenceOne.component.name):
        #         link_name = joint.occurrenceOne.component.name
        #         if not link_positions.get(link_name):
        #             link_positions[link_name] = []
        #         # print('1', joint.occurrenceOne.component.name, joint.geometryOrOriginOne.origin.asArray())
        #         # print('2', joint.occurrenceTwo.component.name, joint.geometryOrOriginTwo.origin.asArray())
        #         [x1, y1, z1] = joint.geometryOrOriginOne.origin.asArray()
        #         (origin2, xAxis2, yAxis2, zAxis2) = joint.occurrenceTwo.transform.getAsCoordinateSystem()
        #         [x2, y2, z2] = origin2.asArray()
        #         site_parent = xml_body_for_occ[joint.occurrenceTwo.component.name]
        #         site = ET.SubElement(site_parent, 'site')
        #         site_name = '{0}:{1}'.format(joint.occurrenceTwo.component.name, joint.occurrenceOne.component.name)
        #         site.set('name', site_name)
        #         site.set('pos',  '{0} {1} {2}'.format(x1 - x2, y1 - y2, z1 - z2))
        #         link_positions[link_name].append((site_name, [x1, y1, z1]))
        #     if re.match('__Motor__.*', joint.name):
        #         motor_xml = ET.SubElement(actuator, 'motor')
        #         motor_xml.set('gear', '100')
        #         motor_xml.set('joint', joint.name)
        #
        #
        # # Links/Tendons
        # for occ in allOccu:
        #     if re.match('__Link__.*', occ.component.name):
        #         link_name = occ.component.name
        #         spatial = ET.SubElement(tendon, 'spatial')
        #         spatial.set('limited', 'true')
        #         spatial.set('width', '0.1')
        #         p0 = link_positions[link_name][0][1]
        #         p1 = link_positions[link_name][1][1]
        #         link_length = sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2 + (p0[2] - p1[2])**2)
        #         spatial.set('range', '5.49 5.51')
        #         assert len(link_positions[link_name]) == 2
        #         site1 = ET.SubElement(spatial, 'site')
        #         site1.set('site', link_positions[link_name][0][0])
        #         site2 = ET.SubElement(spatial, 'site')
        #         site2.set('site', link_positions[link_name][1][0])
        #
        pretty_write(mujoco, os.path.join(exportDir, '{0}.xml'.format(rootComp.name)))

    except:
        print('Failed:\n{}'.format(traceback.format_exc()))
