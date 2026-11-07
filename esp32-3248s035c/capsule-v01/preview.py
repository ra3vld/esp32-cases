"""Render exact exported geometry and an offline WebGL inspection viewer."""
import json
import os
from pathlib import Path
os.environ.setdefault('XDG_CACHE_HOME','/tmp/ldg-capsule-cad-cache')
os.environ.setdefault('MPLCONFIGDIR','/tmp/ldg-capsule-matplotlib')
import cadquery as cq
import numpy as np
import vtk
from vtk.util.numpy_support import numpy_to_vtk,numpy_to_vtkIdTypeArray
from mesh_quality import triangulate
from build import OUT,ROOT,P

COLORS={'01_front_frame':'#343b44','02_body':'#deded6','03_rear_cover':'#c7d0cf',
        '04_sole':'#a7b2b5','05_interface_cassette':'#d69350','06_cable_clamp':'#66757f'}

def meshes():
    rows=[]
    paths=list(sorted((OUT/'step').glob('0[1-6]*.step')))+list(sorted((OUT/'reference_step').glob('*.step')))
    for path in paths:
        name=path.stem
        if name in ('ref_components','ref_antenna_zone','ref_usb_plug'):continue
        shape=cq.importers.importStep(str(path)).val()
        v,t,n=triangulate(shape)
        color=COLORS.get(name,'#bf9b60' if name.startswith('insert') else '#9aabb2' if name.startswith('screw') else
            '#26915e' if name=='ref_pcb' else '#132a3d' if name=='ref_screen' else '#1b2126' if name in ('ref_glass','ref_cable','ref_usb') else
            '#78a5b4' if name.startswith('ref_cup') else '#ceb461' if name.startswith(('ref_led','ref_ldr')) else '#47616e')
        rows.append(dict(name=name,v=v,t=t,n=n,color=color))
    return rows

def offset(name,mode):
    if mode!='exploded':return np.zeros(3)
    if name.startswith(('01','ref_pcb','ref_glass','ref_screen','ref_button','ref_led','ref_ldr','ref_usb','screw_pcb','insert_pcb','screw_frame','insert_frame')):return np.array([0,-40,11])
    if name.startswith(('03','screw_rear')):return np.array([0,65,0])
    if name.startswith(('04','ref_cup','insert_cassette','insert_clamp','screw_sole')):return np.array([0,0,-28])
    if name.startswith(('05','ref_interface','screw_cassette')):return np.array([0,0,-13])
    if name.startswith(('06','screw_clamp')):return np.array([-12,0,-12])
    return np.zeros(3)

def render(rows,modes,filename):
    window=vtk.vtkRenderWindow();window.SetOffScreenRendering(1);window.SetSize(1800,1400);window.SetMultiSamples(4)
    viewports=[(0,.5,.5,1),(.5,.5,1,1),(0,0,.5,.5),(.5,0,1,.5)]
    details=filename in ('cad-surface-details.png','cad-front-joint.png','cad-skirts-and-roots.png')
    if details:
        window.SetNumberOfLayers(2)
        background=vtk.vtkRenderer();background.SetLayer(0);background.SetBackground(.91,.93,.94);window.AddRenderer(background)
        viewports=[(.008,.508,.492,.992),(.508,.508,.992,.992),(.008,.008,.492,.492),(.508,.008,.992,.492)]
    cameras={'assembly':(190,-250,150),'side':(250,40,44),'rear':(130,260,110),'bottom':(140,-140,-220),
             'service':(-160,260,140),'exploded':(210,-280,180),'cassette':(80,135,135),'frame':(-120,220,100),
             'joint_left':(-135,-120,-50),'joint_right':(135,-120,-50),'joint_rear':(140,200,-80),'rear_clean':(0,250,48),
             'joint_front_tl':(-145,-135,135),'joint_front_tr':(145,-135,135),
             'joint_front_bl':(-145,-135,25),'joint_front_br':(145,-135,25),
             'body_bare':(-135,230,-100),'frame_bare':(-110,230,140),
             'rear_bare':(-120,-200,145),'sole_bare':(120,-150,250)}
    focuses={'joint_left':(-46,15,13),'joint_right':(46,15,13),'joint_rear':(0,78,4),'rear_clean':(0,80,44),
             'joint_front_tl':(-54,22,77),'joint_front_tr':(54,22,77),
             'joint_front_bl':(-58,3,10),'joint_front_br':(58,3,10)}
    focuses.update(body_bare=(0,43,36),frame_bare=(0,18,46),rear_bare=(0,70,40),sole_bare=(0,42,10))
    scales={'joint_left':31,'joint_right':31,'joint_rear':48,'rear_clean':55,'body_bare':72,'frame_bare':72,'rear_bare':72,'sole_bare':75,
            'joint_front_tl':22,'joint_front_tr':22,'joint_front_bl':22,'joint_front_br':22}
    dims=json.loads((OUT/'dimensions.json').read_text())['enclosure_xyz_mm']
    labels={'assembly':'CAPSULE 0.1 / INTERNAL FRONT FASTENERS','side':'15 DEG BACK / '+' x '.join(f'{d:.1f}' for d in dims)+' mm',
            'rear':'REAR COVER / PLAIN RESET + BOOT HOLES','bottom':'UNDERSIDE / FOUR 20 mm CUPS',
            'service':'REAR COVER REMOVED / 40 x 25 x 25 mm MODULE',
            'exploded':'EXPLODED / SIX PRINTED PARTS','cassette':'TRAY + CABLE CLAMP / TWO TIES','frame':'FRONT FRAME / PCB POSTS + OPTICAL SLEEVES',
            'joint_left':'FRONT LEFT / COMMON SOLE CONTOUR','joint_right':'FRONT RIGHT / COMMON SOLE CONTOUR',
            'joint_rear':'REAR UNDERSIDE / CONTINUOUS SPLIT','rear_clean':'REAR / ANALYTIC R48 SKIN, NO LOFT CAP',
            'joint_front_tl':'FRONT / UPPER LEFT / SHARED SKIN',
            'joint_front_tr':'FRONT / UPPER RIGHT / SHARED SKIN',
            'joint_front_bl':'FRONT / LOWER LEFT / 0.3 mm SPLITS',
            'joint_front_br':'FRONT / LOWER RIGHT / 0.3 mm SPLITS',
            'body_bare':'MAIN BODY / PERIMETER RETAINED / OPEN CENTRE',
            'frame_bare':'FRONT COVER / CLOSED PERIMETER SKIRT',
            'rear_bare':'REAR COVER / CLOSED PERIMETER SKIRT',
            'sole_bare':'SOLE / CLOSED SKIRT + GUIDED CABLE CLAMP'}
    for mode,viewport in zip(modes,viewports):
        ren=vtk.vtkRenderer();ren.SetViewport(*viewport);ren.SetBackground(.968,.977,.983);window.AddRenderer(ren)
        if details:ren.SetLayer(1);ren.SetPreserveColorBuffer(False)
        for m in rows:
            name=m['name']
            if mode=='body_bare' and name!='02_body':continue
            if mode=='frame_bare' and name!='01_front_frame':continue
            if mode=='rear_bare' and name!='03_rear_cover':continue
            if mode=='sole_bare' and name not in ('04_sole','06_cable_clamp'):continue
            if mode.startswith('joint_') and name.startswith('ref_cup'):continue
            if mode=='service' and (name=='03_rear_cover' or name.startswith('screw_rear')):continue
            if mode=='cassette' and name not in ('04_sole','05_interface_cassette','06_cable_clamp','ref_interface','ref_cable'):continue
            if mode=='frame' and not name.startswith(('01','ref_pcb','ref_glass','ref_screen','ref_led','ref_ldr')):continue
            v=m['v']+offset(name,mode)
            if mode=='sole_bare' and name=='06_cable_clamp':v=v+np.array([0,0,8])
            pts=vtk.vtkPoints();pts.SetData(numpy_to_vtk(v,deep=True));poly=vtk.vtkPolyData();poly.SetPoints(pts)
            cells=np.column_stack([np.full(len(m['t']),3),m['t']]).astype(np.int64).ravel()
            c=vtk.vtkCellArray();c.SetCells(len(m['t']),numpy_to_vtkIdTypeArray(cells,deep=True));poly.SetPolys(c)
            ns=numpy_to_vtk(m['n'],deep=True);ns.SetName('Exact CAD normals');poly.GetPointData().SetNormals(ns)
            mapper=vtk.vtkPolyDataMapper();mapper.SetInputData(poly)
            actor=vtk.vtkActor();actor.SetMapper(mapper)
            prop=actor.GetProperty();prop.SetColor(*(int(m['color'][i:i+2],16)/255 for i in (1,3,5)))
            prop.SetInterpolationToPhong();prop.SetAmbient(.25);prop.SetDiffuse(.73);prop.SetSpecular(.12);prop.SetSpecularPower(30)
            ren.AddActor(actor)
        camera=ren.GetActiveCamera();camera.SetViewUp(0,0,1);camera.SetPosition(*cameras[mode]);camera.SetFocalPoint(0,42,38)
        if mode=='bottom':camera.SetViewUp(0,1,0)
        if mode=='cassette':camera.SetFocalPoint(0,45,15)
        if mode in focuses:camera.SetFocalPoint(*focuses[mode])
        camera.ParallelProjectionOn();camera.SetParallelScale(scales.get(mode,112 if mode=='exploded' else 77 if mode not in ('cassette','frame') else 69))
        ren.ResetCameraClippingRange()
        text=vtk.vtkTextActor();text.SetInput(labels[mode]);text.SetPosition(20,23);text.GetTextProperty().SetFontSize(18);text.GetTextProperty().SetColor(.12,.2,.27);ren.AddActor2D(text)
        if details:text.GetTextProperty().SetBackgroundColor(.97,.98,.99);text.GetTextProperty().SetBackgroundOpacity(.97)
    window.Render();capture=vtk.vtkWindowToImageFilter();capture.SetInput(window);capture.ReadFrontBufferOff();capture.Update()
    writer=vtk.vtkPNGWriter();writer.SetFileName(str(OUT/filename));writer.SetInputConnection(capture.GetOutputPort());writer.Write();window.Finalize()

def html(rows):
    payload=json.dumps([dict(name=m['name'],color=m['color'],v=np.round(m['v'],4).tolist(),t=m['t'].tolist(),n=np.round(m['n'],5).tolist()) for m in rows],separators=(',',':'))
    dims=json.loads((OUT/'dimensions.json').read_text())['enclosure_xyz_mm']
    document=(ROOT/'viewer.template.html').read_text().replace('__MESHES__',payload).replace('__DIMS__',' × '.join(f'{n:.1f}' for n in dims))
    (OUT/'capsule-3d.html').write_text(document)

if __name__=='__main__':
    rows=meshes();html(rows)
    render(rows,('assembly','side','rear','bottom'),'cad-preview.png')
    render(rows,('exploded','service','cassette','frame'),'cad-construction.png')
    render(rows,('joint_left','joint_right','joint_rear','rear_clean'),'cad-surface-details.png')
    render(rows,('joint_front_tl','joint_front_tr','joint_front_bl','joint_front_br'),'cad-front-joint.png')
    render(rows,('body_bare','frame_bare','rear_bare','sole_bare'),'cad-skirts-and-roots.png')
    print('CAD previews and offline viewer exported')
