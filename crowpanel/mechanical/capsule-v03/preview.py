"""Render the exported CAD, and make an offline rotatable assembly viewer."""
import os
os.environ.setdefault('XDG_CACHE_HOME','/tmp/crowpanel-cad-cache')
import json
import math
from pathlib import Path
import cadquery as cq
import numpy as np
import trimesh
import vtk
from vtk.util.numpy_support import numpy_to_vtk,numpy_to_vtkIdTypeArray
import build
from mesh_quality import triangulate

ROOT=Path(__file__).resolve().parent; OUT=ROOT/'output'
COLORS={'01_body':'#c4c9ca','02_rear_cover':'#869bb1','03_bottom_cover':'#727d89','04_cable_clamp':'#505966','steel_ballast':'#bd954a','07_display_carrier':'#cf8746'}

def mesh_shape(s):
    return triangulate(s)

def data():
    p=json.loads((ROOT/'parameters.json').read_text()); tilt=math.radians(p['tilt_back_deg'])
    origin=np.array(json.loads((OUT/'derived.json').read_text())['origin_mm'])
    meshes=[]
    for name in ('01_body','02_rear_cover','03_bottom_cover','04_cable_clamp','07_display_carrier'):
        s=cq.importers.importStep(str(OUT/'step'/f'{name}.step')).val()
        v,t,normals=mesh_shape(s)
        meshes.append(dict(name=name,v=v,t=t,normals=normals,color=COLORS[name]))
    def local_placed(s):
        return s.rotate((0,0,0),(1,0,0),90-p['tilt_back_deg']).translate(tuple(origin))
    for name,shape in build.module_reference.components().items():
        v,t,normals=mesh_shape(local_placed(shape))
        color=('#162432' if name=='display_glass' else '#252d37' if name=='encoder_ring'
               else '#235d4b' if name=='rear_pcb' else '#bc9a54' if name.startswith('m3_mount')
               else '#e4dec8' if name.startswith('connector') else '#65707a')
        meshes.append(dict(name='module_'+name,v=v,t=t,normals=normals,color=color))
    extras={'steel_ballast':build.box(p['ballast_width_mm'],p['ballast_depth_mm'],p['ballast_height_mm'],z=p['bottom_cover_mm'])}
    for i,(x,y) in enumerate(build.MAGNETS):extras[f'magnet_{i}']=build.cylinder(6,3,(x,y,.15))
    for i,(x,y) in enumerate(build.CUPS):
        extras[f'cup_{i}']=build.cylinder(3,6,(x,y,-1)).fuse(cq.Solid.makeCone(11,5,4,cq.Vector(x,y,-5)))
    extras['power_cable']=build.cylinder(1.5,45,(0,43,11.2),(0,1,0))
    extras.update(build.make_hardware(local_placed))
    for n,s in extras.items():
        v,t,normals=mesh_shape(s)
        c=COLORS.get(n,'#5d9fc3' if n.startswith('magnet') else '#cb9653' if n.startswith('cup') else '#c4cbd1' if n.startswith('screw_') else '#303944')
        meshes.append(dict(name=n,v=v,t=t,normals=normals,color=c))
    return meshes

def offset(name,mode):
    if mode!='exploded':return np.zeros(3)
    if name.startswith('module'):return np.array([0,-35,9.38])
    if name=='07_display_carrier' or name.startswith('screw_carrier'):return np.array([0,30,-8])
    if name.startswith('screw_rear'):return np.array([0,56,0])
    if name.startswith('screw_module'):return np.array([0,20,-5])
    if name.startswith('screw_bottom'):return np.array([0,0,-45])
    if name.startswith('screw_cable'):return np.array([0,0,-20])
    if name=='02_rear_cover':return np.array([0,56,0])
    if name=='03_bottom_cover' or name.startswith(('cup','magnet')):return np.array([0,0,-36])
    if name=='steel_ballast':return np.array([0,0,-20])
    if name=='04_cable_clamp':return np.array([0,0,-14])
    return np.zeros(3)

def render(meshes, views=('assembly','front','side','rear'), filename='cad-preview.png'):
    window=vtk.vtkRenderWindow();window.SetOffScreenRendering(1);window.SetSize(1800,1400);window.SetMultiSamples(4)
    modes=views
    positions=[(0,.5,.5,1),(.5,.5,1,1),(0,0,.5,.5),(.5,0,1,.5)]
    for mode,viewport in zip(modes,positions):
        renderer=vtk.vtkRenderer();renderer.SetViewport(*viewport);renderer.SetBackground(.975,.98,.985);window.AddRenderer(renderer)
        for m in meshes:
            name=m['name']
            if mode in ('service','bare','base_internal') and (name=='02_rear_cover' or name.startswith('screw_rear')):continue
            if mode in ('assembly','service','bare','base_internal') and name=='steel_ballast':continue
            if mode in ('bare','base_internal') and (name.startswith(('module','screw_module','screw_carrier')) or name=='07_display_carrier'):continue
            if mode=='base_internal' and name!='01_body':continue
            if mode=='carrier' and name!='07_display_carrier':continue
            v=m['v']+offset(name,mode)
            poly=vtk.vtkPolyData();pts=vtk.vtkPoints();pts.SetData(numpy_to_vtk(v,deep=True));poly.SetPoints(pts)
            cells=np.column_stack([np.full(len(m['t']),3),m['t']]).astype(np.int64).ravel()
            cell=vtk.vtkCellArray();cell.SetCells(len(m['t']),numpy_to_vtkIdTypeArray(cells,deep=True));poly.SetPolys(cell)
            ns=numpy_to_vtk(m['normals'],deep=True);ns.SetName('CAD surface normals');poly.GetPointData().SetNormals(ns)
            mapper=vtk.vtkPolyDataMapper();mapper.SetInputData(poly);
            if mode=='base_internal':
                plane=vtk.vtkPlane();plane.SetOrigin(0,0,18);plane.SetNormal(0,0,1);clip=vtk.vtkClipPolyData();clip.SetInputData(poly);clip.SetClipFunction(plane);clip.InsideOutOn();clip.Update();mapper.SetInputData(clip.GetOutput());
            actor=vtk.vtkActor();actor.SetMapper(mapper)
            actor.GetProperty().SetInterpolationToPhong()
            rgb=tuple(int(m['color'][i:i+2],16)/255 for i in (1,3,5));actor.GetProperty().SetColor(*rgb)
            actor.GetProperty().SetAmbient(.25);actor.GetProperty().SetDiffuse(.7);actor.GetProperty().SetSpecular(.15);actor.GetProperty().SetSpecularPower(25)
            renderer.AddActor(actor)
        camera=renderer.GetActiveCamera();camera.SetViewUp(0,0,1)
        if mode=='assembly':camera.SetPosition(200,-290,150);camera.SetFocalPoint(0,2,54)
        elif mode=='front':camera.SetPosition(0,-300,54);camera.SetFocalPoint(0,0,54)
        elif mode=='side':camera.SetPosition(300,0,54);camera.SetFocalPoint(0,0,54)
        elif mode=='rear':camera.SetPosition(0,300,54);camera.SetFocalPoint(0,0,54)
        elif mode in ('service','bare'):camera.SetPosition(-180,290,150);camera.SetFocalPoint(0,12,65)
        elif mode=='carrier':camera.SetPosition(90,220,90);camera.SetFocalPoint(0,10,61)
        elif mode=='base_internal':camera.SetPosition(45,-65,-190);camera.SetFocalPoint(0,0,8);camera.SetViewUp(0,1,0)
        elif mode=='bottom':camera.SetPosition(150,-160,-220);camera.SetFocalPoint(0,5,45);camera.SetViewUp(0,1,0)
        else:camera.SetPosition(220,-260,180);camera.SetFocalPoint(0,8,43)
        camera.ParallelProjectionOn();camera.SetParallelScale(118 if mode=='exploded' else 55 if mode=='carrier' else 61 if mode=='base_internal' else 76)
        renderer.ResetCameraClippingRange()
        label=vtk.vtkTextActor();label.SetInput({'assembly':'CAPSULE 0.3 / REMOVABLE CARRIER','front':'FRONT / BROAD BODY','side':'SIDE / 15 DEG BACK','rear':'REAR / CLOSED SHELL','bare':'BODY WITHOUT CARRIER','carrier':'FLAT-PRINT CARRIER / FOUR M3x12','base_internal':'BOTTOM BOSSES / THREE-SIDED ROOTS','service':'REAR COVER REMOVED','exploded':'EXPLODED ASSEMBLY','bottom':'UNDERSIDE / CUP ENVELOPES'}[mode]);label.SetPosition(25,25);label.GetTextProperty().SetFontSize(21);label.GetTextProperty().SetColor(.12,.19,.27);renderer.AddActor2D(label)
    window.Render();capture=vtk.vtkWindowToImageFilter();capture.SetInput(window);capture.ReadFrontBufferOff();capture.Update()
    writer=vtk.vtkPNGWriter();writer.SetFileName(str(OUT/filename));writer.SetInputConnection(capture.GetOutputPort());writer.Write();window.Finalize()

def html(meshes):
    rows=[]
    for m in meshes:
        rows.append(dict(name=m['name'],color=m['color'],v=np.round(m['v'],5).tolist(),t=m['t'].tolist(),n=np.round(m['normals'],6).tolist()))
    payload=json.dumps(rows,separators=(',',':'))
    template=(ROOT/'viewer.template.html').read_text()
    (OUT/'capsule-3d.html').write_text(template.replace('__MESHES__',payload))

if __name__=='__main__':
    meshes=data();html(meshes);render(meshes);render(meshes,('service','bare','carrier','base_internal'),'reinforcement-preview.png');render(meshes,('assembly','service','exploded','bottom'),'cad-construction.png')
