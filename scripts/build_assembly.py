"""Rebuild the DIGIT adapter and the Hand-E CAD assembly (all CAD units: mm).

Dependencies: requirements-cad.txt at the repository root.
Run: python scripts/build_assembly.py
The three supplied source files are never modified.
"""
import hashlib
import json
import numpy as np
import cadquery as cq
import trimesh
from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing, BRepBuilderAPI_Transform
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Trsf

from project_paths import INPUT, OUT, INTERMEDIATE, ALIGNED

SOURCE_STL = INPUT / 'DIGIT-Robotiq Adapter-2F85-V1.STL'
SOURCE_SUPPORT = INPUT / 'HAND_E_FINGER_SUPPORT_20181026-Sep-06-2024-02-31-14-1482-PM.step'
SOURCE_GRIPPER = INPUT / 'ROBOTIQ_HAND-E_NO_FINGERTIPS_20190924-Sep-06-2024-02-31-06-0971-PM.step'

# Only the base (z <= 4) is replaced. The sensor plate and complete upper body
# are the original mesh geometry, not an approximate reconstruction.
BASE_LENGTH = 21.5
BASE_WIDTH = 20.2
BASE_THICKNESS = 4.0
HOLE_DIAMETER = 3.4
HOLE_X = (11.0, 18.0)
HOLE_PITCH_Y = 12.0
EDGE_CHAMFER = 0.2

def bounds(s):
    b = s.BoundingBox()
    return [b.xmin, b.ymin, b.zmin, b.xmax, b.ymax, b.zmax]

def volume(s):
    # Empty CAD compounds have no solids; don't interpret surface mass as volume.
    return sum(abs(o.Volume()) for o in s.Solids())

def solid_from_mesh(mesh):
    sew = BRepBuilderAPI_Sewing(1e-6)
    for tri in mesh.triangles:
        wire = cq.Wire.makePolygon([cq.Vector(*v) for v in tri], close=True)
        sew.Add(cq.Face.makeFromWires(wire).wrapped)
    sew.Perform()
    result = cq.Solid.makeSolid(cq.Shell(TopoDS.Shell_s(sew.SewedShape()))).clean()
    assert result.isValid() and len(result.Solids()) == 1
    return result

def mesh_of(shape, tolerance=0.025):
    v, f = shape.tessellate(tolerance, 0.10)
    return trimesh.Trimesh(vertices=[p.toTuple() for p in v], faces=f, process=True)

def transformed(shape, matrix):
    transform = gp_Trsf()
    transform.SetValues(*matrix[:3, :].reshape(-1).tolist())
    return cq.Shape.cast(BRepBuilderAPI_Transform(shape.wrapped, transform, True).Shape())

def rigid(rotation, translation):
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = translation
    assert np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3))
    assert np.isclose(np.linalg.det(t[:3, :3]), 1)
    return t

def main():
    INTERMEDIATE.mkdir(parents=True, exist_ok=True)
    original_mesh = trimesh.load_mesh(SOURCE_STL)
    original = solid_from_mesh(original_mesh)
    support = cq.importers.importStep(str(SOURCE_SUPPORT)).val()
    gripper = cq.importers.importStep(str(SOURCE_GRIPPER)).val()
    yc = float(original_mesh.bounds[:, 1].mean())
    xc = float(np.mean(HOLE_X))
    protected_box = cq.Solid.makeBox(100, 100, 100, cq.Vector(-30, -30, BASE_THICKNESS))
    upper = original.intersect(protected_box).clean()

    base = (cq.Workplane('XY')
            .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, centered=(False, True, False))
            .translate((0, yc, 0)))
    base = base.edges('|Z').chamfer(EDGE_CHAMFER)
    base = base.faces('<Z').edges().chamfer(EDGE_CHAMFER)
    # Keep the top rear edge square under the original wall; the three exposed
    # top sides receive an edge break. Side bevels lie outside the upper body.
    edges = [e for e in base.faces('>Z').edges().vals() if e.Center().x > 0.5]
    base = base.newObject(edges).chamfer(EDGE_CHAMFER)
    base_shape = base.val()
    hole_centers = [(x, yc + dy) for x in HOLE_X for dy in (-6.0, 6.0)]
    for x, y in hole_centers:
        bore = cq.Solid.makeCylinder(HOLE_DIAMETER / 2, 6, cq.Vector(x, y, -1))
        bottom = cq.Solid.makeCone(1.9, 1.7, 0.2, cq.Vector(x, y, 0))
        top = cq.Solid.makeCone(1.7, 1.9, 0.2, cq.Vector(x, y, 3.8))
        base_shape = base_shape.cut(bore.fuse(bottom, top))
    adapter = upper.fuse(base_shape).clean()
    assert adapter.isValid() and len(adapter.Solids()) == 1

    # Adapter bottom contacts the support y=6.5 face. Its four clearance holes
    # coincide with support (x,z) = (+/-6,+/-3.5). Adapter X is the closing axis.
    a_to_s = rigid([[0, -1, 0], [0, 0, 1], [-1, 0, 0]], [yc, 6.5, xc])
    # The support's underside groove floor is y=1.0. Seat it on the rack's
    # y=14.6 ridge; align the two base holes and central dowel at x=+/-14.25.
    s_left = rigid(np.eye(3), [-0.01, 13.6, 26.514])
    s_right = rigid([[-1, 0, 0], [0, 1, 0], [0, 0, -1]], [0.01, 13.6, -26.514])
    transforms = {
        'hand_e': np.eye(4),
        'finger_support_left': s_left,
        'finger_support_right': s_right,
        'digit_adapter_left': s_left @ a_to_s,
        'digit_adapter_right': s_right @ a_to_s,
    }
    local_shapes = {'hand_e': gripper, 'finger_support_left': support,
                    'finger_support_right': support, 'digit_adapter_left': adapter,
                    'digit_adapter_right': adapter}
    assembled = {name: transformed(local_shapes[name], t) for name, t in transforms.items()}
    print('Adapter built; checking preserved geometry and assembly...', flush=True)

    upper_after = adapter.intersect(protected_box)
    upper_lost = volume(upper.cut(upper_after))
    upper_added = volume(upper_after.cut(upper))
    assert upper_lost < 1e-6 and upper_added < 1e-6, (upper_lost, upper_added)
    overlaps = {}
    keys = list(assembled)
    for i, first in enumerate(keys):
        for second in keys[i+1:]:
            overlap = volume(assembled[first].intersect(assembled[second]))
            overlaps[f'{first}__{second}'] = overlap
            print('overlap', first, second, overlap, flush=True)
    assert max(overlaps.values()) < 1e-5, overlaps

    # Check a conservative head envelope, diameter 6 mm and height 3 mm.
    head_intersections = []
    for x, y in hole_centers:
        envelope = cq.Solid.makeCylinder(3, 3, cq.Vector(x, y, BASE_THICKNESS))
        head_intersections.append(volume(adapter.intersect(envelope)))
    assert max(head_intersections) < 1e-6
    expected_holes = np.array([[x, 6.5, z] for x in (-6, 6) for z in (-3.5, 3.5)])
    actual_holes = np.array([(a_to_s @ [x, y, 0, 1])[:3] for x, y in hole_centers])
    hole_errors = [float(np.min(np.linalg.norm(expected_holes - p, axis=1))) for p in actual_holes]
    assert max(hole_errors) < 1e-8

    cq.exporters.export(adapter, str(OUT / 'DIGIT-HandE-Adapter.step'))
    adapter_mesh = mesh_of(adapter, 0.01)
    adapter_mesh.export(OUT / 'DIGIT-HandE-Adapter.stl')
    exported_mesh = trimesh.load_mesh(OUT / 'DIGIT-HandE-Adapter.stl')
    assert exported_mesh.is_watertight and exported_mesh.is_winding_consistent
    assert len(exported_mesh.split()) == 1
    reimported = cq.importers.importStep(str(OUT / 'DIGIT-HandE-Adapter.step')).val()
    assert reimported.isValid() and len(reimported.Solids()) == 1
    assert abs(volume(reimported) - volume(adapter)) < 1e-5

    # Keep components in CAD STEP for selection/editing. A combined STL is an
    # assembly surface mesh, not a fused printable part; the adapter STL is printable.
    assembly = cq.Assembly(name='HandE_DIGIT_Assembly')
    palette = {'hand_e': (0.28, 0.31, 0.36), 'finger_support': (0.65, 0.69, 0.75),
               'digit_adapter': (0.08, 0.65, 0.39)}
    meshes = {}
    aligned_dir = ALIGNED
    aligned_dir.mkdir(exist_ok=True)
    for name, shape in assembled.items():
        color = next(c for key, c in palette.items() if name.startswith(key))
        assembly.add(shape, name=name, color=cq.Color(*color))
        meshes[name] = mesh_of(shape)
        meshes[name].export(aligned_dir / f'{name}.stl')
    assembly.export(str(INTERMEDIATE / 'HandE-DIGIT-Mounts-Assembly.step'))
    combined = trimesh.util.concatenate(list(meshes.values()))
    combined.export(INTERMEDIATE / 'HandE-DIGIT-Mounts-Assembly.stl')

    # Export BREP cache for independent verification/rendering, separate from inputs.
    cq.exporters.export(adapter, str(INTERMEDIATE / 'adapter.brep'))
    report = {
        'units': 'mm',
        'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in (SOURCE_STL, SOURCE_SUPPORT, SOURCE_GRIPPER)},
        'base': {'length': BASE_LENGTH, 'width': BASE_WIDTH, 'thickness': BASE_THICKNESS,
                 'clearance_hole_diameter': HOLE_DIAMETER, 'pattern': [7, 12],
                 'hole_centers_original_stl_xy': hole_centers},
        'adapter_bounds': bounds(adapter),
        'adapter_volume_mm3': volume(adapter),
        'protected_region': 'Original STL z >= 4.0 mm, including the complete DIGIT mounting geometry',
        'protected_region_removed_mm3': upper_lost, 'protected_region_added_mm3': upper_added,
        'adapter_stl_watertight': bool(exported_mesh.is_watertight),
        'adapter_stl_single_component': len(exported_mesh.split()) == 1,
        'adapter_step_valid': bool(reimported.isValid()),
        'clearance_hole_axis_errors_mm': hole_errors,
        'head_envelope_intersections_mm3': head_intersections,
        'assembly_pairwise_intersections_mm3': overlaps,
        'assembly_bounds': combined.bounds.tolist(),
        'adapter_to_support': a_to_s.tolist(),
        'source_to_gripper_cad_transforms_mm': {k: t.tolist() for k, t in transforms.items()},
        'gripper_reference_pose': 'Unchanged supplied STEP pose; no jaw displacement applied',
        'sensor_body_included': False,
        'limitations': ['DIGIT sensor body and cables were not supplied.',
                       'No fabricated hardware or load testing performed.',
                       'Threads are simplified in source STEP; confirm actual hardware.',
                       'No actuator, mass or inertia values are inferred.'],
    }
    (OUT / 'validation.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print('CAD exports and geometric checks complete.', flush=True)

if __name__ == '__main__':
    main()
