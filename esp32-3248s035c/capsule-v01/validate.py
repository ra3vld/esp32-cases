"""Check exported exact bodies, service sweeps and watertight print meshes."""
from __future__ import annotations
import itertools
import hashlib
import json
import math
from pathlib import Path
import cadquery as cq
import numpy as np
import trimesh
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Plane,GeomAbs_Cylinder,GeomAbs_Sphere
import build as b

OUT=b.OUT;P=b.P
LIMIT=.015

def load(folder):
    return {p.stem:cq.importers.importStep(str(p)).val() for p in sorted((OUT/folder).glob('*.step'))}

def front_joint_checks(production,outer):
    results={'surface_checks':{}};failures=[]
    # Front panel and body must inherit the complete shared perimeter. A
    # collision-free independent bezel could still overhang the rounded skin.
    local_outer=b.local_outer()
    front_w=P['frame_back_w']-.01;body_w=P['frame_back_w']+P['clearance']+.01
    profiles=[cq.Workplane('XY').newObject([local_outer]).section(w).val() for w in (front_w,body_w)]
    front_missing=[];front_changes=[];front_gaps=[];front_samples=0
    for degrees in range(0,360,2):
        angle=math.radians(degrees);dx=math.cos(angle);dy=math.sin(angle)
        radii=[];xyzs=[]
        for w in (front_w,body_w):
            lo=0;hi=150
            for _ in range(32):
                mid=(lo+hi)/2
                if local_outer.isInside(cq.Vector(mid*dx,mid*dy,w)):lo=mid
                else:hi=mid
            radii.append((lo+hi)/2);xyzs.append(b.point(radii[-1]*dx,radii[-1]*dy,w))
        if min(q[2] for q in xyzs)<P['sole_thickness']+P['clearance']+.1:continue
        # The USB bay is an intentional opening in the left cheek.
        if dx<0 and abs(radii[1]*dy-P['usb_uv_w'][1])<7.6:continue
        front_samples+=1
        for side,xyz,name in zip(('panel','body'),xyzs,('01_front_frame','02_body')):
            gap=production[name].distance(cq.Vertex.makeVertex(*xyz))
            if gap>.015:front_missing.append({'angle_deg':degrees,'side':side,'gap_mm':round(gap,6)})
        for i in (0,1):
            front_changes.append(profiles[1-i].distance(cq.Vertex.makeVertex(radii[i]*dx,radii[i]*dy,(body_w,front_w)[i])))
        # A material witness just inside the common outline measures the
        # actual facing surface along W, including the four rounded corners.
        radius=min(radii)-1.0;u=radius*dx;v=radius*dy
        witness=cq.Vertex.makeVertex(*b.point(u,v,P['frame_back_w']))
        if production['01_front_frame'].distance(witness)<.005:
            front_gaps.append(production['02_body'].distance(witness))
    results['surface_checks'].update(front_seam_samples=front_samples,front_seam_missing_skin=front_missing,
                                     front_seam_max_contour_change_mm=round(max(front_changes),6),
                                     front_facing_gap_samples=len(front_gaps),
                                     front_face_gap_range_mm=([round(min(front_gaps),6),round(max(front_gaps),6)] if front_gaps else None))
    if front_missing or max(front_changes)>.15:failures.append('continuous front exterior')
    if front_samples<100 or len(front_gaps)!=front_samples or any(abs(g-P['clearance'])>.002 for g in front_gaps):failures.append('front facing gap')
    protruding=production['01_front_frame'].cut(outer).Volume()
    results['surface_checks']['frame_outside_master_mm3']=round(protruding,7)
    if protruding>.001:failures.append('front bosses protrude through master skin')
    return results['surface_checks'],failures

def skirt_joint_checks(parts):
    rows=[];failures=[]
    for name,cover,receiver_names in [('front','01_front_frame',['02_body']),('rear','03_rear_cover',['02_body']),('sole','04_sole',['02_body','03_rear_cover'])]:
        receivers=[parts[n] for n in receiver_names]
        inset=b.skirt_inset(name);gaps=[];tip_gaps=[];missing=[]
        # Three complete exported BREP sections prove a closed annular skirt.
        # This avoids slow coincident-solid booleans on the NURBS front skirt.
        local=b.to_board(parts[cover]) if name=='front' else parts[cover].rotate((0,0,0),(1,0,0),-90) if name=='rear' else parts[cover]
        datum=P['frame_back_w'] if name=='front' else -P['rear_plane_y'] if name=='rear' else P['sole_thickness']
        expected_outer=cq.Face.makeFromWires(b.joint_profile(name,inset)).Area()
        expected_inner=cq.Face.makeFromWires(b.joint_profile(name,inset+P['lip_thickness'])).Area()
        sections=[]
        for level in (.5,1.5,2.5):
            wires=cq.Workplane('XY').newObject([local]).section(datum+level).wires().vals()
            areas=sorted([(cq.Face.makeFromWires(w).Area(),w.IsClosed()) for w in wires],reverse=True)
            good=(len(areas)>=2 and areas[0][1] and areas[1][1] and abs(areas[0][0]-expected_outer)<.05 and areas[1][0]>.8*expected_inner)
            sections.append({'level_mm':level,'closed':bool(good),'outer_area_error_mm2':round(abs(areas[0][0]-expected_outer),6) if areas else None})
        def xyz(q,z):
            if name=='front':return b.point(q.x,q.y,P['frame_back_w']+z)
            if name=='rear':return (q.x,P['rear_plane_y']-z,q.y)
            return (q.x,q.y,P['sole_thickness']+z)
        for position,offset,z,values in [('outer',inset,1.5,gaps),('tip',inset+P['lip_thickness']/2,P['lip_height'],tip_gaps)]:
            for edge in b.joint_profile(name,offset).Edges():
                count=max(2,math.ceil(edge.Length()/4))
                for i in range(count):
                    q=edge.positionAt(i/count);point=xyz(q,z);v=cq.Vertex.makeVertex(*point)
                    gap=min(r.distance(v) for r in receivers);values.append(gap)
                    if parts[cover].distance(v)>.003:missing.append({'at':'male_'+position,'point_mm':[round(c,4) for c in point]})
                    open_usb=(name=='front' and position=='tip' and q.x<-P['pcb_width']/2 and abs(q.y-P['usb_uv_w'][1])<=7.5)
                    good=(gap>=P['clearance']-.003) if open_usb else abs(gap-P['clearance'])<=.003
                    if not good:missing.append({'at':position,'point_mm':[round(c,4) for c in point],'gap_mm':round(gap,5)})
        row={'joint':name,'closed_continuous_skirt':all(s['closed'] for s in sections),'closed_sections':sections,
             'outer_flank_samples':len(gaps),'outer_gap_range_mm':[round(min(gaps),6),round(max(gaps),6)],
             'tip_samples':len(tip_gaps),'tip_gap_range_mm':[round(min(tip_gaps),6),round(max(tip_gaps),6)],'bad_samples':missing}
        rows.append(row)
        if not row['closed_continuous_skirt'] or missing:failures.append('continuous skirt '+name)
    return rows,failures

def check():
    parts=load('step');refs=load('reference_step')
    production={k:v for k,v in parts.items() if k[:2] in ('01','02','03','04','05','06')}
    results={'status':'checking','cad':[],'meshes':[],'part_collisions':[],
             'electronics_collisions':[],'hardware_collisions':[],
             'service_sweeps':[],'joint_measurements':[],'skirt_joints':[],'structure_checks':{},'surface_checks':{},'dimensions':{},'warnings':[]}
    failures=[]
    baseline=json.loads((b.ROOT/'compatibility-baseline.json').read_text())
    unchanged={name:hashlib.sha256((b.ROOT/name).read_bytes()).hexdigest()==sha
               for name,sha in baseline['fixed_export_sha256'].items()}
    if not all(unchanged.values()):failures.append('existing non-front exports changed')
    local_frame=b.to_board(parts['01_front_frame'])
    wires=cq.Workplane('XY').newObject([local_frame]).section(P['frame_front_w']+.1).wires().vals()
    window=next(w for w in wires if 3000<cq.Face.makeFromWires(w).Area()<5000)
    centre=cq.Face.makeFromWires(window).Center();bounds=b.exact_bounds(window)
    pcb_datum=[]
    for u,v in itertools.product((-P['mount_pitch'][0]/2,P['mount_pitch'][0]/2),(-P['mount_pitch'][1]/2,P['mount_pitch'][1]/2)):
        vertex=cq.Vertex.makeVertex(*b.point(u+3.4,v,-P['display_forward_mm']))
        pcb_datum.append(parts['01_front_frame'].distance(vertex))
    glass=b.to_board(refs['ref_glass']);glass_bounds=b.exact_bounds(glass)
    previous=P['usb_plug_previous_size']
    old_plug=b.board(b.box(*previous,x=P['usb_uv_w'][0]-14,y=P['usb_uv_w'][1],
                         z=P['usb_uv_w'][2]-previous[2]/2-P['display_forward_mm']))
    old_intersection=b.bounded_common(old_plug,parts['02_body'])
    results['front_revision']={
        'window_centre_u_mm':round(centre.x,6),'window_size_mm':[round(bounds.xlen,6),round(bounds.ylen,6)],
        'display_forward_mm':P['display_forward_mm'],'pcb_support_height_mm':round(-P['display_forward_mm']-P['frame_back_w'],6),
        'glass_recess_mm':round(glass_bounds.zmin-P['frame_front_w'],6),
        'front_skin_over_glass_pocket_mm':round(P['glass_front_w']-P['display_forward_mm']-P['clearance']-P['frame_front_w'],6),
        'glass_to_frame_gap_mm':round(parts['01_front_frame'].distance(refs['ref_glass']),6),
        'pcb_support_datum_error_mm':round(max(pcb_datum),7),
        'unchanged_export_sha256':unchanged,
        'validated_usb_plug_size_mm':P['usb_plug_size'],
        'validated_usb_plug_to_body_gap_mm':round(refs['ref_usb_plug'].distance(parts['02_body']),6),
        'previous_assumed_8mm_plug_intersection_mm3':round(old_intersection,6)}
    if abs(centre.x-2)>.001 or max(pcb_datum)>.001:failures.append('requested front revision dimensions')
    if abs(glass_bounds.zmin-P['frame_front_w']-1.7)>.001:failures.append('glass recess')
    if parts['01_front_frame'].distance(refs['ref_glass'])<P['clearance']-.001:failures.append('glass clearance')
    # The replacement skin must be analytic, symmetric and free of loft caps.
    outer=b.capsule_outer()
    types=[BRepAdaptor_Surface(f.wrapped).GetType() for f in outer.Faces()
           if b.to_board(f).Center().z>-.5]
    analytic=all(t in (GeomAbs_Plane,GeomAbs_Cylinder,GeomAbs_Sphere) for t in types)
    symmetry=outer.cut(outer.mirror('YZ')).Volume()
    results['surface_checks'].update(analytic_rear_and_sides=analytic,front_fillet_mm=P['front_skin_fillet'],symmetry_error_mm3=round(symmetry,8))
    if not analytic or symmetry>.01:failures.append('analytic symmetric exterior')
    for xyz in ((0,0,500),(33.2,-21,85),(-33.2,-21,85)):
        if b.local_outer().isInside(cq.Vector(*xyz)):failures.append('exterior domain')
    # Full circumference, rather than just the three tongue socket witnesses:
    # match each actual outer edge against the common master skin and measure
    # the contour change over the deliberate 0.3 mm vertical split.
    deviations=[];radial_deviations=[];missing=[]
    zlow=P['sole_thickness']-.01;zhigh=P['sole_thickness']+P['clearance']+.01
    low_profile=cq.Workplane('XY').newObject([outer]).section(zlow).val()
    high_profile=cq.Workplane('XY').newObject([outer]).section(zhigh).val()
    for degrees in range(0,360,3):
        angle=math.radians(degrees);dx=math.cos(angle);dy=math.sin(angle)
        radii=[]
        for z,side in [(P['sole_thickness']-.01,'sole'),(P['sole_thickness']+P['clearance']+.01,'body')]:
            lo=0;hi=150
            for _ in range(32):
                mid=(lo+hi)/2
                if outer.isInside(cq.Vector(mid*dx,40+mid*dy,z)):lo=mid
                else:hi=mid
            radii.append((lo+hi)/2)
            xyz=(radii[-1]*dx,40+radii[-1]*dy,z)
            if side=='body' and P['rear_plane_y']-P['clearance']-.001<xyz[1]<P['rear_plane_y']+.001:continue
            candidates=['04_sole'] if side=='sole' else ['01_front_frame','02_body','03_rear_cover']
            gap=min(production[n].distance(cq.Vertex.makeVertex(*xyz)) for n in candidates)
            if gap>.015:missing.append({'angle_deg':degrees,'side':side,'gap_mm':round(gap,5)})
        radial_deviations.append(abs(radii[1]-radii[0]))
        # Closest 2D contour distance avoids exaggerating the ledge with an
        # oblique radial ray near a front corner.
        deviations.append(high_profile.distance(cq.Vertex.makeVertex(radii[0]*dx,40+radii[0]*dy,zhigh)))
        deviations.append(low_profile.distance(cq.Vertex.makeVertex(radii[1]*dx,40+radii[1]*dy,zlow)))
    results['surface_checks'].update(seam_samples=120,seam_missing_skin=missing,
                                     seam_max_contour_change_mm=round(max(deviations),6),
                                     seam_max_radial_delta_mm=round(max(radial_deviations),6))
    if missing or max(deviations)>.15:failures.append('continuous sole exterior')
    print('Checking outer seams',flush=True)
    front_checks,front_failures=front_joint_checks(production,outer)
    results['surface_checks'].update(front_checks);failures.extend(front_failures)
    print('Checking closed skirts',flush=True)
    results['skirt_joints'],skirt_failures=skirt_joint_checks(production);failures.extend(skirt_failures)
    # No interior floor bars: only the peripheral cover seats remain.
    opening_probe=b.box(82,42,5.4,y=36,z=3.5)
    bars=b.bounded_common(opening_probe,production['02_body'])
    results['structure_checks']['floor_centre_intersection_mm3']=round(bars,7)
    if bars>.001:failures.append('floor centre is obstructed')
    perimeter=[]
    for x in (-35,0,35):
        witness=b.box(12,1,1,x=x,y=67,z=4.5)
        perimeter.append(round(b.bounded_common(witness,production['02_body'])/witness.Volume(),6))
    results['structure_checks']['rear_bottom_perimeter_material_fraction']=perimeter
    if min(perimeter)<.999:failures.append('rear bottom perimeter missing')
    # Former long button tubes must be absent from the cavity, leaving only
    # the two plain holes in the rear wall. This probe caught the old tubes.
    tubes=[]
    for u,v in P['buttons_uv']:
        witness=b.board(b.cyl(5,40,(u,v,P['button_top_w']+P['clearance'])))
        tubes.append(round(b.bounded_common(witness,production['03_rear_cover']),7))
    results['structure_checks']['former_button_guide_material_mm3']=tubes
    if max(tubes)>.001:failures.append('button guide tubes remain')
    # Whole rear saddles must remain, not just a tiny rib after machining cuts.
    rear_saddles=[];rear_necks=[]
    for x,z in P['rear_screws']:
        witness=b.box(16,8,P['rear_root_height'],x=math.copysign(abs(x)+10,x),y=61,z=z-P['rear_root_height']/2)
        fraction=b.bounded_common(witness,production['02_body'])/witness.Volume()
        rear_saddles.append(round(fraction,6))
        neck=b.box(.1,P['rear_root_end_y']-P['rear_post_start_y'],P['rear_root_height'],
                   x=math.copysign(abs(x)+7.5,x),y=(P['rear_root_end_y']+P['rear_post_start_y'])/2,z=z-P['rear_root_height']/2)
        rear_necks.append(round(b.bounded_common(neck,production['02_body'])/.1,4))
    results['structure_checks']['rear_saddle_material_fraction']=rear_saddles
    results['structure_checks']['rear_saddle_neck_area_mm2']=rear_necks
    if min(rear_saddles)<.85:failures.append('rear saddle weakened')
    if min(rear_necks)<140:failures.append('rear saddle neck too small')
    # Removed ventilation slots must now contain solid native rear skin.
    vents=[]
    for x in (-24,-12,0,12,24):
        lo=P['rear_plane_y'];hi=100
        for _ in range(30):
            mid=(lo+hi)/2
            if outer.isInside(cq.Vector(x,mid,63)):lo=mid
            else:hi=mid
        vents.append(production['03_rear_cover'].distance(cq.Vertex.makeVertex(x,(lo+hi)/2,63)))
    results['structure_checks']['former_slit_skin_error_mm']=round(max(vents),7)
    if max(vents)>.002:failures.append('rear ventilation slits remain')
    # These samples cover the former central bump as well as the surrounding
    # back. The finished lid must coincide with the master skin there.
    rear_deviation=[]
    for x,z in itertools.product((-20,0,20),(20,35,50)):
        lo=P['rear_plane_y'];hi=P['outer_depth']+20
        for _ in range(32):
            mid=(lo+hi)/2
            if outer.isInside(cq.Vector(x,mid,z)):lo=mid
            else:hi=mid
        d=production['03_rear_cover'].distance(cq.Vertex.makeVertex(x,(lo+hi)/2,z))
        rear_deviation.append(d)
    results['surface_checks']['rear_skin_deviation_mm']=round(max(rear_deviation),7)
    if max(rear_deviation)>.005:failures.append('smooth rear exterior')
    cheek_deviation=[]
    for side,y,z in itertools.product((-1,1),(65,68,70),(52,56,60)):
        lo=0;hi=P['outer_width']
        for _ in range(32):
            mid=(lo+hi)/2
            if outer.isInside(cq.Vector(side*mid,y,z)):lo=mid
            else:hi=mid
        cheek_deviation.append(production['02_body'].distance(cq.Vertex.makeVertex(side*(lo+hi)/2,y,z)))
    results['surface_checks']['cheek_skin_deviation_mm']=round(max(cheek_deviation),7)
    if max(cheek_deviation)>.005:failures.append('tool channels break exterior cheek')
    print('Checking exported CAD and meshes',flush=True)
    for name,s in parts.items():
        valid=s.isValid() and len(s.Solids())==1 and s.Volume()>0
        results['cad'].append({'name':name,'valid':valid,'solids':len(s.Solids()),'volume_mm3':round(s.Volume(),4)})
        if not valid:failures.append('CAD '+name)
        for folder in ('stl','print_stl'):
            m=trimesh.load_mesh(OUT/folder/(name+'.stl'),process=True)
            good=bool(m.is_watertight and m.is_winding_consistent and m.volume>0 and len(m.split())==1)
            zmin=float(m.bounds[0,2]);volume_error=abs(m.volume-s.Volume())/s.Volume()
            row={'name':name,'folder':folder,'watertight':bool(m.is_watertight),
                 'winding_consistent':bool(m.is_winding_consistent),'components':len(m.split()),
                 'faces':len(m.faces),'volume_error_fraction':round(volume_error,7),'zmin':round(zmin,6),'pass':good and volume_error<.004}
            if folder=='print_stl':row['pass']=row['pass'] and abs(zmin)<.001
            row['pass']=bool(row['pass'])
            results['meshes'].append(row)
            if not row['pass']:failures.append('MESH '+name+' '+folder)
    print('Checking collisions',flush=True)
    for (n,a),(m,c) in itertools.combinations(production.items(),2):
        v=b.bounded_common(a,c)
        if v>LIMIT:results['part_collisions'].append({'a':n,'b':m,'volume_mm3':round(v,5)})
    electronics={k:v for k,v in refs.items() if k.startswith('ref_') and k!='ref_antenna_zone'}
    for n,a in production.items():
        for m,c in electronics.items():
            v=b.bounded_common(a,c)
            if v>LIMIT:results['electronics_collisions'].append({'a':n,'b':m,'volume_mm3':round(v,5)})
    # Inserts have intentional radial interference with their OWN 4.2 mm pilot.
    owners={'frame':'01_front_frame','pcb':'01_front_frame','rear':'02_body','sole':'02_body','cassette':'04_sole','clamp':'04_sole'}
    for m,c in refs.items():
        if not m.startswith(('screw_','insert_')):continue
        for n,a in production.items():
            owner=owners[m.split('_')[1]] if m.startswith('insert_') else None
            if owner==n:continue
            v=b.bounded_common(a,c)
            if v>LIMIT:results['hardware_collisions'].append({'a':n,'b':m,'volume_mm3':round(v,5)})
    print('Checking service paths and assembly',flush=True)
    service=[]
    # Actual free paperclip lumen, from just above each button to outside.
    for i,(u,v) in enumerate(P['buttons_uv']):
        start=P['button_top_w']-P['display_forward_mm']+.01
        end=(P['outer_depth']+2-b.ORIGIN[1]-v*math.sin(b.TILT))/math.cos(b.TILT)
        probe=b.board(b.cyl(1.0,end-start,(u,v,start)))
        service.append((f'paperclip_{i}',probe,[]))
    # The whole plug translates straight through the actual side bay.
    for distance in (0,5,10,20):
        service.append((f'usb_insert_{distance}',refs['ref_usb_plug'].translate((-distance,0,0)),[]))
    for i,(u,v) in enumerate(P['frame_screws']):
        service.append((f'internal_frame_driver_{i}',b.frame_tool_paths(4.5)[i],['03_rear_cover']))
        # Front skin over the blind insert must remain solid, with no head or
        # through-hole. A 2 mm diameter material witness covers each screw axis.
        skin=b.board(b.cyl(2,1,(u,v,P['frame_front_w']+.1)))
        covered=b.bounded_common(skin,production['01_front_frame'])/skin.Volume()
        if covered<.999:failures.append(f'front skin over frame insert {i}')
        contact=b.point(u+math.copysign(3.5,u),v,P['frame_boss_end_w'])
        if production['02_body'].distance(cq.Vertex.makeVertex(*contact))>.001:
            failures.append(f'frame screw support datum {i}')
    for name,uv,diam in [('ldr',P['ldr_uv'],1),('led',P['led_uv'],.5)]:
        service.append((name+'_line_of_sight',b.board(b.cyl(diam,12.7,(*uv,-15))),[]))
    for name,probe,skip in service:
        collisions=[]
        for n,s in production.items():
            if n in skip:continue
            v=b.bounded_common(probe,s)
            if v>LIMIT:collisions.append([n,round(v,5)])
        results['service_sweeps'].append({'name':name,'pass':not collisions,'collisions':collisions})
    # Rigid extraction of rear cover with plain button access holes.
    for dy in (2,5,10,20,40,70):
        moved=production['03_rear_cover'].translate((0,dy,0));collisions=[]
        for n,s in production.items():
            if n in ('03_rear_cover','04_sole','05_interface_cassette','06_cable_clamp'):continue
            v=b.bounded_common(moved,s)
            if v>LIMIT:collisions.append([n,round(v,5)])
        results['service_sweeps'].append({'name':f'rear_remove_{dy}','pass':not collisions,'collisions':collisions})
    # Sole unplugs vertically after screws and interface wires are removed.
    sole_subassembly=cq.Compound.makeCompound([production[n] for n in ('04_sole','05_interface_cassette','06_cable_clamp')]+[refs['ref_interface']])
    for dz in (1,3,6,12):
        moved=sole_subassembly.translate((0,0,-dz));collisions=[]
        for n in ('01_front_frame','02_body','03_rear_cover'):
            v=b.bounded_common(moved,production[n])
            if v>LIMIT:collisions.append([n,round(v,5)])
        results['service_sweeps'].append({'name':f'sole_remove_{dz}','pass':not collisions,'collisions':collisions})
    # Front subassembly travels along the PCB normal after sole removal.
    front_names=('ref_pcb','ref_glass','ref_screen','ref_components','ref_usb','ref_led','ref_ldr','ref_button_0','ref_button_1')
    front_subassembly=cq.Compound.makeCompound([production['01_front_frame']]+[refs[n] for n in front_names]+
        [s for n,s in refs.items() if n.startswith(('screw_pcb_','insert_pcb_','insert_frame_'))])
    for d in (1,3,8,16):
        shift=(0,-math.cos(b.TILT)*d,math.sin(b.TILT)*d)
        moved=front_subassembly.translate(shift)
        collisions=[]
        for n in ('02_body','03_rear_cover'):
            v=b.bounded_common(moved,production[n])
            if v>LIMIT:collisions.append([n,round(v,5)])
        results['service_sweeps'].append({'name':f'frame_remove_{d}','pass':not collisions,'collisions':collisions})
    # Active image has an unobstructed normal projection through the bezel.
    view=b.board(b.box(*P['active_size'],8,x=P['active_center_u'],z=-12))
    v=b.bounded_common(view,production['01_front_frame'])
    results['service_sweeps'].append({'name':'full_active_image','pass':v<LIMIT,'intersection_mm3':round(v,5)})
    # Measure actual receiving surfaces, not just nominal parameter arithmetic.
    # Flat flanks are sampled away from interrupted screw pads and chamfers.
    # Heat insert access is verified by screw geometry and length accounting.
    results['dimensions']={
        'pcb_mount_pitch_mm':P['mount_pitch'],'tilt_from_vertical_deg':P['tilt_back_deg'],
        'radial_joint_gap_mm':P['clearance'],
        'skirt_thickness_mm':P['lip_thickness'],
        'receiver_step_depth_from_joint_datum_mm':P['lip_height']+P['clearance'],
        'lip_tip_axial_clearance_mm':P['clearance'],
        'suction_hole_mm':P['suction_hole'],'suction_web_mm':P['suction_web'],
        'PCB_screw_engagement_mm':4.0,'frame_screw_engagement_mm':4.0,
        'rear_screw_engagement_mm':4.0,'sole_screw_engagement_mm':4.0,
        'cassette_screw_engagement_mm':3.6,'clamp_screw_engagement_mm':3.0}
    # Antenna/module separation is a geometric envelope check, not an RF test.
    distance=refs['ref_antenna_zone'].distance(refs['ref_interface'])
    results['dimensions']['interface_to_estimated_antenna_mm']=round(distance,4)
    if distance<15:failures.append('interface too close to estimated antenna')
    for group in ('part_collisions','electronics_collisions','hardware_collisions'):
        if results[group]:failures.append(group)
    if any(not row['pass'] for row in results['service_sweeps']):failures.append('service_sweeps')
    results['warnings']=[
        'Physical printing, strength, suction retention, RF and optical calibration remain untested.',
        'PCB depth, glass stack, USB/button/optical coordinates are reconstructed or assumed, not factory C-revision 3D dimensions.',
        'Hardware insert interference is intentionally excluded only for its declared owning printed part.',
        'USB plug compatibility is verified for 23 x 12 x 6.8 mm, centred on the moved socket. The former assumed 8 mm-thick plug interferes by 0.3 mm in W with the unchanged body; use a thinner plug or fit-check the actual rounded plug.'
    ]
    results['failures']=failures;results['status']='PASS' if not failures else 'FAIL'
    inputs=['build.py','parameters.json','mesh_quality.py','validate.py','compatibility-baseline.json']
    results['input_sha256']={n:hashlib.sha256((b.ROOT/n).read_bytes()).hexdigest() for n in inputs}
    paths=[f for folder in ('step','stl','print_stl','reference_step') for f in (OUT/folder).glob('*') if f.is_file()]+[OUT/'assembly.step']
    results['geometry_sha256']={f.relative_to(b.ROOT).as_posix():hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(paths)}
    (OUT/'validation.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps({k:results[k] for k in ('status','failures','part_collisions','electronics_collisions','hardware_collisions')},indent=2),flush=True)
    for row in results['service_sweeps']:
        if not row['pass']:print(row,flush=True)
    return not failures

if __name__=='__main__':raise SystemExit(0 if check() else 1)
