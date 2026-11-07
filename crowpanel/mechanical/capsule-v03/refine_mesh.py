"""Re-export verified, unchanged STEP geometry with absolute STL tolerances."""
import hashlib
import itertools
import math
import json
from pathlib import Path
import numpy as np
import cadquery as cq
import trimesh
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRep import BRep_Tool
from OCP.BRepTools import BRepTools
from OCP.BRepClass import BRepClass_FaceClassifier
from OCP.GeomAPI import GeomAPI_ProjectPointOnSurf
from OCP.TopAbs import TopAbs_IN,TopAbs_ON
from OCP.gp import gp_Pnt,gp_Pnt2d
import build

ROOT=Path(__file__).resolve().parent;OUT=ROOT/'output'

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def periodic_distance(shape,point,distance):
    # Extrema can miss the equivalent UV branch on a trimmed periodic face.
    # Accept a correction only after exact projection AND trim classification.
    p=gp_Pnt(*point)
    def equivalent(value,low,high,period):
        if not period:return [value] if low-1e-8<=value<=high+1e-8 else []
        return [value+k*period for k in range(math.ceil((low-value)/period-1e-10),math.floor((high-value)/period+1e-10)+1)]
    for face in shape.Faces():
        surface=BRep_Tool.Surface_s(face.wrapped)
        if not (surface.IsUPeriodic() or surface.IsVPeriodic()):continue
        limits=BRepTools.UVBounds_s(face.wrapped)
        project=GeomAPI_ProjectPointOnSurf(p,surface)
        if not project.IsDone():continue
        for k in range(1,project.NbPoints()+1):
            u,v=project.Parameters(k)
            us=equivalent(u,limits[0],limits[1],surface.UPeriod() if surface.IsUPeriodic() else None)
            vs=equivalent(v,limits[2],limits[3],surface.VPeriod() if surface.IsVPeriodic() else None)
            for uu,vv in itertools.product(us,vs):
                if BRepClass_FaceClassifier(face.wrapped,gp_Pnt2d(uu,vv),1e-7).State() in (TopAbs_IN,TopAbs_ON):
                    distance=min(distance,surface.Value(uu,vv).Distance(p))
    return distance

def deviations(shape,mesh,count=256):
    # Deterministic area-weighted samples of facet interiors, in assembly mm.
    rng=np.random.default_rng(21021)
    ids=rng.choice(len(mesh.faces),size=min(count,len(mesh.faces)),replace=False,p=mesh.area_faces/mesh.area)
    values=[];corrections=0;raw_max=0.
    for point in mesh.triangles_center[ids]:
        tool=BRepExtrema_DistShapeShape(cq.Vertex.makeVertex(*point).wrapped,shape.wrapped)
        if not tool.IsDone():raise RuntimeError('Point-to-CAD distance failed')
        raw=tool.Value();distance=periodic_distance(shape,point,raw) if raw>.05 else raw
        corrections+=int(distance<raw-1e-7);raw_max=max(raw_max,raw);values.append(distance)
    return {'samples':len(values),'max_mm':max(values),'p95_mm':float(np.percentile(values,95)),
            'periodic_face_projection_corrections':corrections,'global_extrema_raw_max_mm':raw_max}

def main():
    p=json.loads((ROOT/'parameters.json').read_text());r=json.loads((OUT/'validation.json').read_text())
    if not r.get('cad_checks_passed',r['all_checks_passed']):raise RuntimeError('Run build.py successfully before refining meshes')
    hashes=r.get('geometry_step_sha256')
    if not hashes:raise RuntimeError('Missing STEP hashes for mechanical validation')
    assert all(digest(OUT/f)==sha for f,sha in hashes.items()),'STEP changed since mechanical validation'
    report={'revision':p['revision'],'linear_deflection_mm':p['mesh_linear_deflection_mm'],
            'relative':False,'angular_deflection_rad':p['mesh_angular_deflection_rad'],
            'geometry_step_sha256':hashes,'geometry_unchanged':True,'parts':{}}
    for name,prev in r['parts'].items():
        print('Refine',name,flush=True)
        s=cq.importers.importStep(str(OUT/'step'/f'{name}.step')).val()
        path=OUT/'stl'/f'{name}.stl';old=trimesh.load(path,force='mesh')
        before=deviations(s,old) if name=='01_body' else None
        s.exportStl(str(path),tolerance=p['mesh_linear_deflection_mm'],angularTolerance=p['mesh_angular_deflection_rad'],relative=False)
        oriented=build.as_printed(name,s,r['derived']['origin_mm'],p['tilt_back_deg'])
        pp=OUT/'print_stl'/f'{name}.stl'
        oriented.exportStl(str(pp),tolerance=p['mesh_linear_deflection_mm'],angularTolerance=p['mesh_angular_deflection_rad'],relative=False)
        pm=trimesh.load(pp,force='mesh');pm.apply_translation(-pm.bounds[0]);pm.export(pp)
        mesh=trimesh.load(path,force='mesh')
        quality={'triangles_before':len(old.faces),'triangles_after':len(mesh.faces),'watertight':bool(mesh.is_watertight),
                 'winding_consistent':bool(mesh.is_winding_consistent),'mesh_components':len(mesh.split(only_watertight=False)),
                 'print_watertight':bool(pm.is_watertight),'print_winding_consistent':bool(pm.is_winding_consistent),
                 'print_components':len(pm.split(only_watertight=False)),
                 'print_size_mm':pm.extents.tolist(),'print_min_xyz_mm':pm.bounds[0].tolist()}
        if before:quality['facet_to_CAD_before']=before;quality['facet_to_CAD_after']=deviations(s,mesh)
        report['parts'][name]=quality
        prev.update({k:quality[k] for k in ('watertight','winding_consistent','mesh_components','print_size_mm','print_min_xyz_mm','print_watertight')})
    report['geometry_unchanged']=all(digest(OUT/f)==sha for f,sha in hashes.items())
    report['file_sha256']={str(f.relative_to(OUT)):digest(f) for folder in ('stl','print_stl') for f in sorted((OUT/folder).glob('*.stl'))}
    checks=[q['watertight'] and q['winding_consistent'] and q['mesh_components']==1 and q['print_watertight'] and q['print_winding_consistent'] and q['print_components']==1 and max(q['print_size_mm'][:2])<=180 for q in report['parts'].values()]
    checks += [report['geometry_unchanged'],report['parts']['01_body']['facet_to_CAD_after']['max_mm']<=.05]
    report['all_checks_passed']=all(checks);r['revision']=p['revision'];r['mesh_quality_report']='mesh-validation.json'
    r['all_checks_passed']=r.get('cad_checks_passed',r['all_checks_passed']) and report['all_checks_passed']
    (OUT/'validation.json').write_text(json.dumps(r,indent=2)+'\n')
    (OUT/'mesh-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Mesh checks passed:',report['all_checks_passed'],flush=True)
    if not report['all_checks_passed']:raise RuntimeError('Mesh quality validation failed')

if __name__=='__main__':main()
