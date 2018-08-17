# -*- coding: utf-8 -*-
import xml.etree.ElementTree as ET
import os.path
import xml.dom.minidom
import re

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

def append_root_join_elements(body):
    slide_template = '<joint armature="0" damping="0" limited="false" pos="0 0 0" stiffness="0"/>'
    for type in ['slide', 'hinge']:
        for dim in ['x', 'y', 'z']:
            joint = ET.fromstring(slide_template)
            joint.set('type', type)
            joint.set('name', '{}_{}'.format(type, dim))
            joint.set('axis', '{} {} {}'.format(
                1 if dim == 'x' else 0,
                1 if dim == 'y' else 0,
                1 if dim == 'z' else 0))
            body.append(joint)

def rescale(arr):
    [x, y, z] = arr
    x *= 10
    y *= 10
    z *= 10
    return [round(x, 3), round(y, 3), round(z, 3)]

def create_xml_body(occ):
    xml_body = ET.Element('body')
    xml_body.set('name', occ.component.name)
    (origin, xAxis, yAxis, zAxis) = occ.transform.getAsCoordinateSystem()
    [x, y, z] = rescale(origin.asArray())
    xml_body.set('pos', '{0} {1} {2}'.format(round(x, 3), round(y, 3), round(z, 3)))
    geom = ET.SubElement(xml_body, 'geom')
    geom.set('type', 'mesh')
    geom.set('mesh', occ.component.name)
    return xml_body

def write_stls_and_mojoco_xml(design, exportDir, instance_name):
    rootComp = design.rootComponent

    mujoco = ET.XML(XML_TEMPLATE)
    asset = mujoco[4]
    worldbody = mujoco[5]
    tendon = mujoco[6]
    actuator = mujoco[7]
    assert asset.tag == 'asset'
    assert worldbody.tag == 'worldbody'
    assert tendon.tag == 'tendon'
    assert actuator.tag == 'actuator'
    mujoco.set('model', rootComp.name)

    # export the occurrence one by one in the root component to a specified file
    exportMgr = design.exportManager
    allOccu = rootComp.allOccurrences
    for occ in allOccu:
        if not occ.isVisible:
            continue
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
        assert found == False
        if (occ.isGrounded):
            body = xml_body_for_occ[occ.component.name]
            append_root_join_elements(body)
            # marker = ET.fromstring('<body pos="0 0 0"><geom pos="0 0 0" type="sphere" size="2" rgba="1.0 0 1.0 1"/></body>'.format(body.get('pos')))
            # body.append(marker)
            worldbody.append(body)
            found = True
            break

    ### Some really flippen weird Fusion 360 behaviour happening here
    ### Joint origin seem to be very buggy or have bizarre behaviour
    ### This is a fudge to get the correct joint origins
    legRadiusParam = design.userParameters.itemByName('LegRadius')
    leg_radius = legRadiusParam.value * 10

    # Joints
    assert(found)
    for joint in rootComp.joints:
        parent_occ = joint.occurrenceTwo
        child_occ = joint.occurrenceOne
        if not child_occ.isVisible:
            continue
        parent = xml_body_for_occ[parent_occ.component.name]
        child_xml_body = xml_body_for_occ[child_occ.component.name]

        (origin1, xAxis1, yAxis1, zAxis1) = parent_occ.transform.getAsCoordinateSystem()
        (origin2, xAxis2, yAxis2, zAxis2) = child_occ.transform.getAsCoordinateSystem()
        [parent_x, parent_y, parent_z] = rescale(origin1.asArray())
        [child_x, child_y, child_z] = rescale(origin2.asArray())
        local_x = child_x - parent_x
        local_y = child_y - parent_y
        local_z = child_z - parent_z
        parent.append(child_xml_body)

        # Fusion Revolute joint create Mujoco Hinge joints
        if joint.jointMotion.jointType == 1:
            [joint_x1, joint_y1, joint_z1] = rescale(joint.geometryOrOriginOne.origin.asArray())
            joint_xml = ET.SubElement(child_xml_body, 'joint')
            joint_xml.set('axis', '{0} {1} {2}'.format(*joint.geometryOrOriginOne.primaryAxisVector.asArray()))
            joint_xml.set('name', '{0}'.format(joint.name))
            mujuco_joint_pos = '{0} {1} {2}'.format(round(joint_x1 - local_x - leg_radius, 3), round(joint_y1 - local_y, 3), round(joint_z1 - local_z, 3))
            joint_xml.set('pos', mujuco_joint_pos)
            joint_xml.set('type', 'hinge')
            marker = ET.fromstring('<body pos="{0}"><geom pos="0 0 0" type="sphere" size="2" rgba="0 1.0 1.0 1"/></body>'.format(mujuco_joint_pos))
            child_xml_body.append(marker)

    mujoco_xml = xml.dom.minidom.parseString(re.sub('\n\s*', '', str(ET.tostring(mujoco), 'utf-8'))).toprettyxml()
    filename = os.path.join(exportDir, '{}.xml'.format(instance_name))
    f = open(filename, 'w')
    f.write(mujoco_xml)
    f.close()
