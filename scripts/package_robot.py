"""Add upstream Tacto DIGITs, generate the aligned assembly and ROS description.

Run after scripts/build_assembly.py. Dependencies additionally include xacro and
urdf-parser-py. All packaged meshes and URDF transforms are in metres;
output/*Assembly.stl and output/intermediate/aligned_parts_mm/*.stl are in millimetres.
"""
import ast
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
import xacro
from urdf_parser_py.urdf import URDF

from project_paths import INPUT, OUT, ALIGNED, PKG, UPSTREAM

def number_list(values):
    return ' '.join(f'{float(v):.12g}' for v in values)

def circle_fit(vertices, z, x_range, y_range):
    a = vertices[(abs(vertices[:, 2] - z) < 1e-4)
                 & (vertices[:, 0] > x_range[0]) & (vertices[:, 0] < x_range[1])
                 & (vertices[:, 1] > y_range[0]) & (vertices[:, 1] < y_range[1])]
    assert len(a) >= 20
    coefs = np.linalg.lstsq(np.column_stack([2*a[:, 0], 2*a[:, 1], np.ones(len(a))]),
                            (a[:, :2]**2).sum(axis=1), rcond=None)[0]
    radius = np.sqrt(coefs[2] + (coefs[:2]**2).sum())
    residual = np.max(abs(np.linalg.norm(a[:, :2] - coefs[:2], axis=1) - radius))
    assert residual < 2e-5
    return coefs[:2]

def write(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding='utf-8')

def main():
    report = json.loads((OUT / 'validation.json').read_text())
    transforms = {k: np.array(v) for k, v in report['source_to_gripper_cad_transforms_mm'].items()}
    upstream_sensor = trimesh.load_mesh(UPSTREAM / 'digit.STL')
    sensor_mm = upstream_sensor.copy()
    sensor_mm.apply_scale(1000)
    original = trimesh.load_mesh(INPUT / 'DIGIT-Robotiq Adapter-2F85-V1.STL')
    adapter_holes = np.array([circle_fit(original.vertices, 21.8, (9, 12), r)
                              for r in [(1, 5), (11, 15)]])
    sensor_holes = np.array([circle_fit(sensor_mm.vertices, .534803, (1, 5), r)
                             for r in [(-7, -3), (3, 7)]])
    sensor_to_adapter = np.eye(4)
    sensor_to_adapter[:2, 3] = (adapter_holes - sensor_holes).mean(axis=0)
    sensor_to_adapter[2, 3] = 22.0 - sensor_mm.bounds[0, 2]
    sensor_axis_errors = np.linalg.norm(sensor_holes + sensor_to_adapter[:2, 3] - adapter_holes, axis=1)
    assert max(sensor_axis_errors) < 1e-4

    meshes = {name: trimesh.load_mesh(ALIGNED / f'{name}.stl') for name in transforms}
    for side in ['left', 'right']:
        name = f'digit_{side}'
        transforms[name] = transforms[f'digit_adapter_{side}'] @ sensor_to_adapter
        meshes[name] = sensor_mm.copy().apply_transform(transforms[name])
        meshes[name].export(ALIGNED / f'{name}.stl')
    # All sensor points lie above all CAD parts (contact at Y=42.1 mm), and
    # the two sensor bounding boxes are disjoint along the closing axis Z.
    sensor_clearances = {}
    for name in ['digit_left', 'digit_right']:
        for other in [n for n in meshes if n != name]:
            a, b = meshes[name].bounds, meshes[other].bounds
            separation = float(max(np.max(a[0] - b[1]), np.max(b[0] - a[1])))
            sensor_clearances[f'{name}__{other}'] = separation
            assert separation >= -1e-5, (name, other, separation)
    left_normal = transforms['digit_left'][:3, 0]
    right_normal = transforms['digit_right'][:3, 0]
    assert np.allclose(left_normal, [0, 0, -1])
    assert np.allclose(right_normal, [0, 0, 1])
    full_mesh = trimesh.util.concatenate(list(meshes.values()))
    full_mesh.export(OUT / 'HandE-DIGIT-Assembly.stl')

    mesh_dir = PKG / 'meshes'
    mesh_dir.mkdir(parents=True, exist_ok=True)
    for local_name, assembled_name in [('hand_e', 'hand_e'), ('finger_support', 'finger_support_left'),
                                        ('digit_adapter', 'digit_adapter_left')]:
        m = meshes[assembled_name].copy()
        m.apply_transform(np.linalg.inv(transforms[assembled_name]))
        m.apply_scale(.001)
        m.export(mesh_dir / f'{local_name}.stl')
    # Preserve the supplied Tacto mesh bytes and its native metre coordinates.
    shutil.copy2(UPSTREAM / 'digit.STL', mesh_dir / 'digit.STL')
    (mesh_dir / 'collision').mkdir(exist_ok=True)
    hull = upstream_sensor.convex_hull
    hull.export(mesh_dir / 'collision' / 'digit_hull.stl')
    assert hull.is_watertight and hull.is_convex
    (PKG / 'third_party' / 'tacto').mkdir(parents=True, exist_ok=True)
    for file in ['LICENSE', 'digit.urdf', 'config_digit.yml', 'source.json']:
        shutil.copy2(UPSTREAM / file, PKG / 'third_party' / 'tacto' / file)

    nodes = [('hand_e', 'hand_e_base_link', None, np.eye(4), 'hand_e.stl', '.28 .31 .36 1')]
    for side in ['left', 'right']:
        support_name, adapter_name, digit_name = [f'{key}_{side}' for key in ['finger_support', 'digit_adapter', 'digit']]
        nodes.extend([
            (support_name, f'{support_name}_link', 'hand_e_base_link', transforms[support_name], 'finger_support.stl', '.65 .69 .75 1'),
            (adapter_name, f'{adapter_name}_link', f'{support_name}_link', np.array(report['adapter_to_support']), 'digit_adapter.stl', '.08 .65 .39 1'),
            (digit_name, digit_name, f'{adapter_name}_link', sensor_to_adapter, 'digit.STL', '.82 .84 .87 1'),
        ])
    header = '''<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Fixed CAD assembly in the supplied Hand-E STEP frame, using metres.
       No actuator or dynamic inertial properties are invented. -->
  <xacro:macro name="robotiq_digit_assembly" params="prefix:='' parent:='' xyz:='0 0 0' rpy:='0 0 0' mesh_prefix:='package://robotiq_digit_description/meshes/'">
    <xacro:if value="${parent != ''}">
      <joint name="${prefix}hand_e_mount_joint" type="fixed">
        <parent link="${parent}"/><child link="${prefix}hand_e_base_link"/>
        <origin xyz="${xyz}" rpy="${rpy}"/>
      </joint>
    </xacro:if>
'''
    chunks = [header]
    for _, link, parent, t, filename, color in nodes:
        collision = 'collision/digit_hull.stl' if filename == 'digit.STL' else filename
        chunks.append(f'''    <link name="${{prefix}}{link}">
      <visual><origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry><mesh filename="${{mesh_prefix}}{filename}" scale="1 1 1"/></geometry>
        <material name="${{prefix}}{link}_color"><color rgba="{color}"/></material>
      </visual>
      <collision><origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry><mesh filename="${{mesh_prefix}}{collision}" scale="1 1 1"/></geometry>
      </collision>
    </link>
''')
        if parent:
            rpy = Rotation.from_matrix(t[:3, :3]).as_euler('xyz')
            chunks.append(f'''    <joint name="${{prefix}}{link}_joint" type="fixed">
      <parent link="${{prefix}}{parent}"/><child link="${{prefix}}{link}"/>
      <origin xyz="{number_list(t[:3, 3] * .001)}" rpy="{number_list(rpy)}"/>
    </joint>
''')
    for side in ['left', 'right']:
        chunks.append(f'''    <!-- Tacto's configured gel reference; its normal is local +X. -->
    <link name="${{prefix}}digit_{side}_gel"/>
    <joint name="${{prefix}}digit_{side}_gel_joint" type="fixed">
      <parent link="${{prefix}}digit_{side}"/><child link="${{prefix}}digit_{side}_gel"/>
      <origin xyz="0.022 0 0.015" rpy="0 0 0"/>
    </joint>
''')
    chunks.append('  </xacro:macro>\n</robot>\n')
    write(PKG / 'urdf' / 'robotiq_digit_macro.xacro', ''.join(chunks))
    write(PKG / 'urdf' / 'robotiq_digit_assembly.urdf.xacro', '''<?xml version="1.0"?>
<robot name="robotiq_hand_e_digit" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="prefix" default=""/>
  <xacro:arg name="mesh_prefix" default="package://robotiq_digit_description/meshes/"/>
  <xacro:include filename="robotiq_digit_macro.xacro"/>
  <xacro:robotiq_digit_assembly prefix="$(arg prefix)" mesh_prefix="$(arg mesh_prefix)"/>
</robot>
''')
    source = PKG / 'urdf' / 'robotiq_digit_assembly.urdf.xacro'
    urdf = xacro.process_file(str(source)).toprettyxml(indent='  ')
    urdf = urdf.replace(str(source), source.name)
    write(PKG / 'urdf' / 'robotiq_digit_assembly.urdf', urdf)
    portable = xacro.process_file(str(source), mappings={'mesh_prefix': '../meshes/'}).toprettyxml(indent='  ')
    portable = portable.replace(str(source), source.name)
    write(PKG / 'urdf' / 'robotiq_digit_assembly.portable.urdf', portable)
    prefixed = xacro.process_file(str(source), mappings={'prefix': 'test_'}).toprettyxml(indent='  ')
    assert URDF.from_xml_string(prefixed).get_root() == 'test_hand_e_base_link'
    robot = URDF.from_xml_string(urdf)
    assert robot.get_root() == 'hand_e_base_link' and len(robot.links) == 9 and len(robot.joints) == 8
    assert all(j.type == 'fixed' for j in robot.joints)
    for mesh in ET.fromstring(urdf).iter('mesh'):
        path = PKG / mesh.attrib['filename'].replace('package://robotiq_digit_description/', '')
        assert path.is_file(), path
        assert mesh.attrib['scale'] == '1 1 1'
    # Independent FK from the emitted URDF, compared with each aligned CAD mesh.
    fk = {robot.get_root(): np.eye(4)}
    pending = list(robot.joints)
    while pending:
        joint = next(j for j in pending if j.parent in fk)
        t = np.eye(4)
        t[:3, :3] = Rotation.from_euler('xyz', joint.origin.rpy).as_matrix()
        t[:3, 3] = joint.origin.xyz
        fk[joint.child] = fk[joint.parent] @ t
        pending.remove(joint)
    fk_errors = {}
    for key, link, _, _, filename, _ in nodes:
        m = trimesh.load_mesh(mesh_dir / filename)
        m.apply_transform(fk[link])
        fk_errors[key] = float(np.max(abs(m.bounds * 1000 - meshes[key].bounds)))
    assert max(fk_errors.values()) < 5e-5, fk_errors

    # Keep repository/package checks reproducible instead of relying on a
    # one-off local validation after the build.
    for name, digest in report['source_sha256'].items():
        assert hashlib.sha256((INPUT / name).read_bytes()).hexdigest() == digest
    for mesh in ET.fromstring(portable).iter('mesh'):
        assert (PKG / 'urdf' / mesh.attrib['filename']).resolve().is_file()
    macro_path = str(PKG / 'urdf' / 'robotiq_digit_macro.xacro')
    integration = ET.Element('robot', {'name': 'integration', 'xmlns:xacro': 'http://www.ros.org/wiki/xacro'})
    ET.SubElement(integration, 'link', {'name': 'tool0'})
    ET.SubElement(integration, 'xacro:include', {'filename': macro_path})
    ET.SubElement(integration, 'xacro:robotiq_digit_assembly', {
        'parent': 'tool0', 'prefix': 'gripper_', 'xyz': '0.1 0.2 0.3', 'rpy': '0 0 0'})
    doc = xacro.parse(ET.tostring(integration, encoding='unicode'))
    xacro.process_doc(doc)
    embedded = URDF.from_xml_string(doc.toxml())
    assert embedded.get_root() == 'tool0' and len(embedded.links) == 10 and len(embedded.joints) == 9
    assert embedded.joint_map['gripper_hand_e_mount_joint'].origin.xyz == [0.1, 0.2, 0.3]
    ET.parse(PKG / 'package.xml')
    ast.parse((PKG / 'launch' / 'display.launch.py').read_text(encoding='utf-8'))

    report.update({
        'sensor_body_included': True,
        'sensor_source': json.loads((UPSTREAM / 'source.json').read_text()),
        'sensor_sha256': hashlib.sha256((UPSTREAM / 'digit.STL').read_bytes()).hexdigest(),
        'sensor_to_adapter_transform_mm': sensor_to_adapter.tolist(),
        'sensor_mount_axis_errors_mm': sensor_axis_errors.tolist(),
        'sensor_aabb_clearance_or_contact_mm': sensor_clearances,
        'sensor_left_gel_normal_cad': left_normal.tolist(),
        'sensor_right_gel_normal_cad': right_normal.tolist(),
        'sensor_mesh_watertight_upstream': bool(upstream_sensor.is_watertight),
        'sensor_collision_mesh': 'Watertight convex hull of upstream DIGIT mesh',
        'source_to_gripper_cad_transforms_mm': {k: t.tolist() for k, t in transforms.items()},
        'assembly_bounds_with_sensors_mm': full_mesh.bounds.tolist(),
        'urdf_forward_kinematics_bounds_errors_mm': fk_errors,
        'urdf_link_count': len(robot.links), 'urdf_joint_count': len(robot.joints),
        'urdf_joint_type': 'fixed', 'urdf_mesh_units': 'm',
        'package_checks': {'portable_mesh_paths': True, 'xacro_parent_prefix_integration': True,
                           'source_files_unchanged': True, 'package_xml_parse': True,
                           'launch_python_syntax': True, 'ros2_runtime_tested': False},
        'limitations': ['No fabricated hardware or load testing performed.',
                       'Threads are simplified in source STEP; confirm actual hardware.',
                       'Fixed assembly model; no actuator, mass or inertia values are inferred.',
                       'DIGIT cables and added mounting screws are not modeled.',
                       'Original Tacto visual mesh contains open internal surfaces; kept unchanged.'],
    })
    write(OUT / 'validation.json', json.dumps(report, indent=2))
    write(PKG / 'validation.json', json.dumps(report, indent=2))
    print('Full assembly STL, metre meshes, URDF and Xacro generated and validated.')
    print('Sensor axes maximum error (mm):', float(max(sensor_axis_errors)))
    print('URDF/CAD maximum bounds error (mm):', max(fk_errors.values()))

if __name__ == '__main__':
    main()
