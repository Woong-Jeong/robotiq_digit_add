"""Render the actual CAD/mesh assembly without a display server."""
import numpy as np
import trimesh
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray

from project_paths import OUT, ALIGNED, UPSTREAM

def actor(mesh, color, transform=None):
    points = vtk.vtkPoints()
    points.SetData(numpy_to_vtk(mesh.vertices, deep=True))
    cells = vtk.vtkCellArray()
    cells.SetData(numpy_to_vtkIdTypeArray(np.arange(len(mesh.faces)+1, dtype=np.int64)*3, deep=True),
                  numpy_to_vtkIdTypeArray(np.asarray(mesh.faces.reshape(-1), dtype=np.int64), deep=True))
    poly = vtk.vtkPolyData(); poly.SetPoints(points); poly.SetPolys(cells)
    normals = vtk.vtkPolyDataNormals(); normals.SetInputData(poly); normals.SetFeatureAngle(35)
    normals.SplittingOn(); normals.Update()
    mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(normals.GetOutputPort())
    result = vtk.vtkActor(); result.SetMapper(mapper)
    prop = result.GetProperty(); prop.SetColor(*color); prop.SetAmbient(.22)
    prop.SetDiffuse(.75); prop.SetSpecular(.2); prop.SetSpecularPower(35)
    if transform is not None:
        matrix = vtk.vtkMatrix4x4()
        for i in range(4):
            for j in range(4): matrix.SetElement(i, j, float(transform[i, j]))
        result.SetUserMatrix(matrix)
    return result

def label(renderer, text, x, y, size=22, color=(.9,.94,.98)):
    a = vtk.vtkTextActor(); a.SetInput(text); a.SetPosition(x, y)
    p=a.GetTextProperty(); p.SetFontSize(size);p.SetFontFamilyToArial();p.SetColor(*color)
    renderer.AddViewProp(a)

def main():
    window = vtk.vtkRenderWindow();window.SetOffScreenRendering(1)
    window.SetSize(1800, 1100);window.SetMultiSamples(4)
    left=vtk.vtkRenderer();left.SetViewport(0,0,.61,1);left.SetBackground(.055,.075,.11)
    right=vtk.vtkRenderer();right.SetViewport(.61,0,1,1);right.SetBackground(.075,.10,.14)
    window.AddRenderer(left);window.AddRenderer(right)
    rotation=np.array([[1,0,0,0],[0,0,-1,0],[0,1,0,0],[0,0,0,1]],float)
    palette={'hand_e':(.27,.31,.37),'finger_support':(.65,.70,.78),'digit_adapter':(.05,.7,.43),'digit':(.78,.82,.86)}
    for p in sorted(ALIGNED.glob('*.stl')):
        if p.stem in ['digit_left', 'digit_right']:
            continue
        m=trimesh.load_mesh(p)
        key=next(key for key in palette if p.stem.startswith(key))
        left.AddActor(actor(m,palette[key],rotation))
    # Highlight the actual gel submesh without changing any exported geometry.
    import json
    report=json.loads((OUT/'validation.json').read_text())
    raw=trimesh.load_mesh(UPSTREAM/'digit.STL')
    components=raw.split(only_watertight=False)
    gel=max([c for c in components if c.bounds[0,0]>.016],key=lambda c:len(c.faces))
    body=trimesh.util.concatenate([c for c in components if c is not gel])
    gel.apply_scale(1000)
    body.apply_scale(1000)
    for side in ['left','right']:
        t=np.array(report['source_to_gripper_cad_transforms_mm'][f'digit_{side}'])
        left.AddActor(actor(body,(.78,.82,.86),rotation@t))
        left.AddActor(actor(gel,(.2,.6,.78),rotation@t))
    cam=left.GetActiveCamera();cam.SetPosition(205,-245,145);cam.SetFocalPoint(0,0,-6);cam.SetViewUp(0,0,1)
    cam.ParallelProjectionOn();cam.SetParallelScale(105)
    label(left,'HAND-E + 2 x DIGIT',36,1040,30)
    label(left,'Aligned CAD assembly  |  Tacto sensor meshes',36,1003,19,(.60,.70,.81))
    label(left,'Green: modified adapters   /   Silver: finger supports',36,55,18,(.6,.75,.75))
    label(left,'Provided STEP pose  |  Sensors face inward',36,25,18,(.6,.70,.80))
    m=trimesh.load_mesh(OUT/'DIGIT-HandE-Adapter.stl')
    base=m.triangles[:,:,2].max(axis=1)<=4.00001
    right.AddActor(actor(m.submesh([np.nonzero(~base)[0]],append=True),(.06,.72,.43)))
    right.AddActor(actor(m.submesh([np.nonzero(base)[0]],append=True),(.95,.51,.17)))
    cam=right.GetActiveCamera();cam.SetPosition(65,-70,48);cam.SetFocalPoint(9,8,10)
    cam.SetViewUp(0,0,1);cam.ParallelProjectionOn();cam.SetParallelScale(26)
    label(right,'MODIFIED ADAPTER',28,1040,27)
    label(right,'Original DIGIT mount preserved',28,1002,19,(.4,.86,.65))
    label(right,'Base: 21.5 x 20.2 x 4 mm',28,155,23)
    label(right,'4 x diameter 3.4 mm  /  pitch 12 x 7 mm',28,115,19,(.98,.68,.38))
    label(right,'Orange: revised base',28,65,18,(.98,.68,.38))
    label(right,'No CAD interference at the supplied pose',28,30,17,(.63,.73,.83))
    window.Render()
    capture=vtk.vtkWindowToImageFilter();capture.SetInput(window);capture.SetInputBufferTypeToRGB();capture.ReadFrontBufferOff();capture.Update()
    writer=vtk.vtkPNGWriter();writer.SetFileName(str(OUT/'assembly_preview.png'));writer.SetInputConnection(capture.GetOutputPort());writer.Write()
    print('Rendered', OUT/'assembly_preview.png')

if __name__=='__main__': main()
