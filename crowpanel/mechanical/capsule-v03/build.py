"""CrowPanel Capsule v0.3: reproducible PETG solids in mm, CadQuery 2.8.

Coordinates: X left/right, Y rearward, Z up. Local module XY is the
display plane, local +Z points toward the user. Module geometry is an independent simplified primitive reference.
"""
from __future__ import annotations
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
os.environ.setdefault('XDG_CACHE_HOME','/tmp/crowpanel-cad-cache')
os.environ.setdefault('MPLCONFIGDIR','/tmp/crowpanel-matplotlib')
import cadquery as cq
import numpy as np
import trimesh
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
import module_reference

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'output'
SOURCE=ROOT/'module_reference.py'
MOUNT=module_reference.MOUNT
CARRIER=[(x,y) for x in (-30.,30.) for y in (-13.,5.)]
COVER=[(-35.5,30),(35.5,30),(-32.5,82),(32.5,82)]
BASE_SCREWS=[(x,y) for x in (-16,16) for y in (-33.5,33.5)]
MAGNETS=[(x,y) for x in (-32,32) for y in (-24,24)]
CUPS=[(x,y) for x in (-25,25) for y in (-35.5,35.5)]

def cylinder(r,h,xyz=(0,0,0),direction=(0,0,1)):
    return cq.Solid.makeCylinder(r,h,cq.Vector(*xyz),cq.Vector(*direction))

def box(w,d,h,x=0,y=0,z=0):
    return cq.Workplane('XY').box(w,d,h,centered=(True,True,False)).translate((x,y,z)).val()

def ellipse(rx,ry,z,cy=0):
    return cq.Workplane('XY',origin=(0,cy,z)).ellipse(rx,ry).val()

def loft(rows):
    return cq.Solid.makeLoft([ellipse(rx,ry,z,cy) for z,rx,ry,cy in rows],False)

def tube_between(a,b,r):
    v=cq.Vector(*b)-cq.Vector(*a)
    return cylinder(r,v.Length,a,v.toTuple())

def rounded_rect(w,d,r,h,xy=(0,0),z=0):
    return cq.Workplane('XY').sketch().rect(w,d).vertices().fillet(r).finalize().extrude(h).translate((*xy,z)).val()

def common(a,b):
    aa=a.BoundingBox();bb=b.BoundingBox()
    if any(getattr(aa,n+'max')<getattr(bb,n+'min')-1e-7 or getattr(bb,n+'max')<getattr(aa,n+'min')-1e-7 for n in ('x','y','z')):
        return 0.
    return a._bool_op([a],[b],BRepAlgoAPI_Common(),parallel=False).Volume()

def rect_wire(w,front,back,r,z):
    return cq.Workplane('XY',origin=(0,(front+back)/2,z)).rect(w-2*r,back-front-2*r).offset2D(r).val()

def sculpt(rows):
    return cq.Solid.makeLoft([rect_wire(w,f,b,r,z) for z,w,f,b,r in rows],False)

def prism_xz(points,y,length):
    wire=cq.Wire.makePolygon([cq.Vector(x,y,z) for x,z in points],close=True)
    return cq.Solid.extrudeLinear(wire,[],cq.Vector(0,length,0))

def fan_points(x,y,direction,angle,reach=14,width=5):
    # Three broad radial webs fan toward the nearest outer wall.
    a=math.radians(angle);ux=math.cos(a)*direction[0]-math.sin(a)*direction[1];uy=math.sin(a)*direction[0]+math.cos(a)*direction[1]
    vx,vy=-uy,ux
    return [(x+d*ux+side*w*vx,y+d*uy+side*w*vy) for d,w,side in ((3.8,2.2,-1),(reach,width/2,-1),(reach,width/2,1),(3.8,2.2,1))]

def rear_boss(x,z,y,length,outer):
    boss=cylinder(6.5,length,(x,y,z),(0,1,0))
    for a in (-55,0,55):boss=boss.fuse(prism_xz(fan_points(x,z,(math.copysign(1,x),0),a),y,length))
    return boss.intersect(outer)

def bottom_boss(x,y,outer):
    boss=cylinder(6.5,11.2,(x,y,4))
    for a in (-55,0,55):
        rib=cq.Workplane('XY',origin=(0,0,5.8)).polyline(fan_points(x,y,(0,math.copysign(1,y)),a)).close().extrude(9.4).val()
        boss=boss.fuse(rib)
    # Preserve the cable-clamp seat on the inward fourth side.
    boss=boss.intersect(outer).cut(rounded_rect(23,9.6,2.3,4.4,(0,35),4.2))
    for hx in (-8,8):boss=boss.cut(cylinder(3.2,2,(hx,35,2.55)))
    return boss

def make_hardware(placed):
    def screw(x,y,z,length,direction=(0,0,1)):
        axis=cq.Vector(*direction);pt=cq.Vector(x,y,z)
        return cylinder(2.85,1.65,(pt-axis*1.65).toTuple(),direction).fuse(cylinder(1.5,length,pt.toTuple(),direction))
    result={}
    for i,(x,y) in enumerate(MOUNT):result[f'screw_module_{i+1}']=placed(screw(x,y,-24,8))
    for i,(x,y) in enumerate(CARRIER):result[f'screw_carrier_{i+1}']=placed(screw(x,y,-31.5,12))
    for i,(x,z) in enumerate(COVER):result[f'screw_rear_{i+1}']=screw(x,22.5,z,16,(0,-1,0))
    for i,(x,y) in enumerate(BASE_SCREWS):result[f'screw_bottom_{i+1}']=screw(x,y,2.1,8)
    for i,x in enumerate((-8,8)):result[f'screw_cable_{i+1}']=screw(x,35,4.5,10)
    return result

def make_parts(p):
    angle=90-p['tilt_back_deg'];tilt=math.radians(p['tilt_back_deg'])
    origin=(0,-14,p['front_lip_height_mm']+p['finger_clearance_mm']+39.5*math.cos(tilt)+1.6*math.sin(tilt))
    def placed(s):return s.rotate((0,0,0),(1,0,0),angle).translate(origin)
    front_z=-1.6-p['front_module_gap_mm']
    def plane_front(z):return -14+math.tan(tilt)*(z-origin[2])-front_z/math.cos(tilt)
    rows=[(4,94,-44,44,30),(8,94,-44,44,30),(12,92,-40,43,28),
          (16,90,-29.5,42,24),(20,87,-21.5,41,18),(28,85,plane_front(28)+1.2,39,12),
          (40,84,plane_front(40)+1.2,36,10),(52,85,plane_front(52),33,8),(68,85,plane_front(68),29,8),
          (82,78.5,plane_front(82),24,8),(94,65,plane_front(94),17,7),
          (102,44,plane_front(102),9,4.5),(107,17,plane_front(107),4,1.5),
          (108.5,2,plane_front(108.5),2.5,.5)]
    outer=sculpt(rows)
    inner_rows=[(-1,88,-41,41,27)]
    inner_rows += [(z,w-6,f+3,b-3,max(.5,r-3)) for z,w,f,b,r in rows[1:-2]]
    inner_rows += [(105,12,1.8,3.4,.5),(106,2,2.1,2.6,.2)]
    inner=sculpt(inner_rows)
    shell=outer.cut(inner)
    # A close-fitting continuous front lip; its PCB aperture is 51 mm.
    panel=cylinder(39,3,(0,0,front_z-3)).cut(cylinder(25.5,3.2,(0,0,front_z-3.1)))
    shell=shell.fuse(placed(panel))
    keepout=cylinder(25.5,20,(0,0,-22.6)).fuse(cylinder(40,16-front_z,(0,0,front_z)))
    shell=shell.cut(placed(keepout))
    ysplit=p['rear_split_y_mm'];zsplit=p['rear_cover_bottom_z_mm'];gap=p['part_clearance_mm']
    body=shell.cut(box(200,130,160,y=ysplit+65,z=zsplit))
    cap=shell.intersect(box(200,130,160,y=ysplit+gap+65,z=zsplit+gap))

    # Removable, flat-printing U carrier, four M3 screws into fixed pads.
    carrier=cylinder(12,12,(0,0,-31.5))
    def strap(a,b,width):
        dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy);px,py=-dy/length*width/2,dx/length*width/2
        poly=[(a[0]+px,a[1]+py),(b[0]+px,b[1]+py),(b[0]-px,b[1]-py),(a[0]-px,a[1]-py)]
        piece=cq.Workplane('XY',origin=(0,0,-31.5)).polyline(poly).close().extrude(8).val()
        return piece.fuse(cylinder(width/2,8,(*a,-31.5)),cylinder(width/2,8,(*b,-31.5)))
    for sign in (-1,1):
        branch=strap((sign*5,-7),(sign*24,-26),9)
        elbow=strap((sign*24,-26),(sign*30,-13),12)
        rail=box(14,18,8,x=sign*30,y=-4,z=-31.5)
        carrier=carrier.fuse(branch,elbow,rail)
    # Flat reliefs clear the lower rear-cover roots during withdrawal.
    for sign in (-1,1):
        toe=cq.Workplane('XY',origin=(0,0,-31.6)).polyline([(sign*9,-45),(sign*65,-45),(sign*65,11)]).close().extrude(12.3).val()
        carrier=carrier.cut(toe)
    # Keep the right JST port and its rear insertion window open.
    carrier=carrier.cut(box(20,11,12.3,x=19.8,z=-31.6))
    for x,y in CARRIER:
        carrier=carrier.fuse(cylinder(7,8,(x,y,-31.5)))
        carrier=carrier.cut(cylinder(1.7,8.2,(x,y,-31.6)))
        receiver=cylinder(7,18.3,(x,y,-23.5))
        # Three local webs carry the load into the front annulus and side wall.
        for a in (-55,0,55):
            poly=fan_points(x,y,(math.copysign(1,x),0),a,reach=15,width=5)
            web=cq.Workplane('XY',origin=(0,0,-23.5)).polyline(poly).close().extrude(18.3).val()
            receiver=receiver.fuse(web)
        receiver=receiver.cut(keepout)
        receiver=receiver.cut(cylinder(2.15,5.3,(x,y,-23.5)))
        receiver=receiver.cut(cylinder(1.7,11,(x,y,-23.5)))
        body=body.fuse(placed(receiver).intersect(outer))
    for x,y in MOUNT:
        carrier=carrier.cut(cylinder(1.7,13,(x,y,-31.6)))
        carrier=carrier.cut(cylinder(3.1,7.6,(x,y,-31.6)))
    mount_world=placed(carrier)
    # Four horizontal screws hold a full-height back shell, not a round cap.
    for x,z in COVER:
        boss=rear_boss(x,z,5.5,8.9,outer)
        body=body.fuse(boss)
        body=body.cut(cylinder(1.7,15,(x,0,z),(0,1,0)))
        body=body.cut(cylinder(2.15,5.3,(x,ysplit,z),(0,-1,0)))
        cb=rear_boss(x,z,ysplit+gap,30,outer)
        cap=cap.fuse(cb)
        cap=cap.cut(cylinder(1.7,45,(x,ysplit,z),(0,1,0)))
        cap=cap.cut(cylinder(3.2,50,(x,22.5,z),(0,1,0)))
    cap=cap.intersect(box(200,130,160,y=ysplit+gap+65,z=zsplit+gap))
    # The rear cable and its anchor stay on the fixed low base.
    body=body.fuse(box(28,12,2.2,y=36,z=15.8).intersect(outer))
    passage=rounded_rect(9,7,2,30).rotate((0,0,0),(1,0,0),90).translate((0,60,11.2))
    body=body.cut(passage)
    for x,y in BASE_SCREWS:
        body=body.fuse(bottom_boss(x,y,outer))
        body=body.cut(cylinder(1.7,6.7,(x,y,3.9)))
        body=body.cut(cylinder(2.15,5.3,(x,y,4)))
    for x in (-8,8):
        body=body.fuse(cylinder(4.5,10,(x,35,8)).intersect(outer))
        body=body.cut(cylinder(1.7,11,(x,35,8)))
        body=body.cut(cylinder(2.15,5.3,(x,35,8)))
    # Reopen cable space after adding the rear anchor bosses.
    body=body.cut(passage)

    bottom=rounded_rect(94,88,30,4)
    lip=rounded_rect(87.2,81.2,26.6,1.3,z=4).cut(rounded_rect(83.2,77.2,24.6,1.5,z=3.9))
    for x,y in BASE_SCREWS:lip=lip.cut(cylinder(7,2,(x,y,3.8)))
    bottom=bottom.fuse(lip)
    for x in (-37.75,37.75):bottom=bottom.fuse(rounded_rect(2.5,39,1,5,(x,0),4))
    for y in (-25,25):bottom=bottom.fuse(rounded_rect(60,3,1,5,(0,y),4))
    for x,y in BASE_SCREWS:
        bottom=bottom.cut(cylinder(1.7,6,(x,y,-.1)))
        bottom=bottom.cut(cylinder(3.2,2.2,(x,y,-.1)))
    for x,y in MAGNETS:bottom=bottom.cut(cylinder(p['magnet_pocket_diameter_mm']/2,p['magnet_pocket_depth_mm']+.1,(x,y,-.1)))
    for x,y in CUPS:
        bottom=bottom.cut(cylinder(p['suction_bore_mm']/2,14,(x,y,-.5)))
        bottom=bottom.cut(cq.Solid.makeCone(3.6,3,.6,cq.Vector(x,y,0)))
    # Recessed tie channels retain the ballast without a tall plinth.
    for y in (-15,15):
        bottom=bottom.cut(box(78,4.5,1.3,y=y,z=-.1))
        for x in (-38,38):bottom=bottom.cut(box(2,4.5,12,x=x,y=y,z=-.1))
    clamp=rounded_rect(22.4,9,2,3.5,(0,35),4.5)
    for x in (-3.5,3.5):clamp=clamp.cut(box(1.8,3.2,5,x=x,y=35,z=4))
    clamp=clamp.cut(box(9,3.2,1.4,y=35,z=4.4))
    for x in (-8,8):
        clamp=clamp.cut(cylinder(1.7,5,(x,35,4)))
        bottom=bottom.cut(cylinder(3.2,2,(x,35,2.55)))
    bottom=bottom.cut(box(27,11,5,y=35,z=4))

    coupon=rounded_rect(116,50,4,4)
    for x,d in zip((-40,-20,0,20,40),p['suction_bore_coupon_mm']):
        coupon=coupon.cut(cylinder(d/2,7,(x,11,-.5)))
        coupon=coupon.cut(cq.Workplane('XY',origin=(x,20,3.5)).text(str(d),3.5,1,combine=False,font='DejaVu Sans').val())
    for x,d in zip((-28,0,28),(12.1,12.25,12.4)):
        coupon=coupon.cut(cylinder(d/2,3.15,(x,-9,.85)))
        coupon=coupon.cut(cq.Workplane('XY',origin=(x,-20,3.5)).text(str(d),3.5,1,combine=False,font='DejaVu Sans').val())
    for x,d in zip((-48,48),(4.2,4.3)):
        coupon=coupon.fuse(cylinder(4.5,6,(x,-10,0))).cut(cylinder(d/2,5.3,(x,-10,.7)))
    fit=cylinder(10,4)
    for x,y in MOUNT:fit=fit.cut(cylinder(1.7,5,(x,y,-.5)))
    parts={'01_body':body.clean(),'02_rear_cover':cap.clean(),'03_bottom_cover':bottom.clean(),
           '04_cable_clamp':clamp.clean(),'05_fit_coupon':coupon.clean(),'06_mount_test':fit.clean(),'07_display_carrier':mount_world.clean()}
    for name,shape in parts.items():
        solids=shape.Solids()
        if len(solids)!=1:raise RuntimeError(f'{name}: disconnected CAD solids')
        # Fill air bubbles trapped between overlapping reinforcement roots.
        parts[name]=cq.Solid.makeSolid(solids[0].outerShell()).clean()
    module=module_reference.assembly()
    refs={'module_reference':placed(module),'steel_ballast':box(p['ballast_width_mm'],p['ballast_depth_mm'],p['ballast_height_mm'],z=4),
          'module_keepout':placed(keepout),'mount_fork':mount_world}
    refs.update(make_hardware(placed))
    for i,(x,y) in enumerate(MAGNETS):refs[f'magnet_{i+1}']=cylinder(6,3,(x,y,.15))
    derived={'origin_mm':origin,'module_mount_local_xy_mm':MOUNT,'factory_mount_face_local_z_mm':-19.5,
             'factory_m3_thread_depth_mm':6,'tilt_back_deg':p['tilt_back_deg'],'finger_clearance_to_front_lip_mm':22,
             'front_module_gap_mm':p['front_module_gap_mm'],'front_lip_height_mm':8,'base_nominal_width_mm':94,
             'base_nominal_depth_mm':88,'ballast_mass_g':p['ballast_width_mm']*p['ballast_depth_mm']*p['ballast_height_mm']*.00785,'magnet_centers_mm':MAGNETS,
             'cup_centers_mm':CUPS,'bottom_screw_centers_mm':BASE_SCREWS,'rear_split_y_mm':ysplit,
             'rear_cover_bottom_z_mm':zsplit,'ring_diameter_mm':79,'ring_sidewall_mm':14,'carrier_screw_local_xy_mm':CARRIER,'carrier_plate_mm':8,'carrier_center_height_mm':12,'lid_boss_diameter_mm':13,'gussets_per_boss':3}
    return parts,refs,derived,placed,placed


def as_printed(name,shape,origin,tilt=15):
    if name=='07_display_carrier':shape=shape.translate(tuple(-v for v in origin)).rotate((0,0,0),(1,0,0),tilt-90)
    if name=='02_rear_cover':shape=shape.rotate((0,0,0),(1,0,0),90)
    b=shape.BoundingBox()
    return shape.translate((-b.xmin,-b.ymin,-b.zmin))

def build():
    OUT.mkdir(exist_ok=True)
    for sub in ('stl','step','print_stl'):(OUT/sub).mkdir(exist_ok=True)
    p=json.loads((ROOT/'parameters.json').read_text())
    print('Construct revised shell',flush=True)
    parts,refs,d,placed,_=make_parts(p)
    print('Constructed',flush=True)
    validation={'source_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
                'module_reference':module_reference.validate(),
                'input_sha256':{name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in ('build.py','module_reference.py','parameters.json')},
                'cadquery_version':cq.__version__,'derived':d,'parts':{},'intersections_mm3':{},'module_intersections_mm3':{}}
    for name,s in parts.items():
        print('Export',name,flush=True)
        cq.exporters.export(s,str(OUT/'step'/f'{name}.step'))
        s.exportStl(str(OUT/'stl'/f'{name}.stl'),tolerance=p['mesh_linear_deflection_mm'],angularTolerance=p['mesh_angular_deflection_rad'],relative=False)
        oriented=as_printed(name,s,d['origin_mm'],p['tilt_back_deg'])
        print_path=OUT/'print_stl'/f'{name}.stl'
        oriented.exportStl(str(print_path),tolerance=p['mesh_linear_deflection_mm'],angularTolerance=p['mesh_angular_deflection_rad'],relative=False)
        print_mesh=trimesh.load(print_path,force='mesh')
        # OCCT bounding boxes can be conservative for B-splines. Put the
        # actual exported triangulation exactly on the slicer build plate.
        print_mesh.apply_translation(-print_mesh.bounds[0])
        print_mesh.export(print_path)
        mesh=trimesh.load(OUT/'stl'/f'{name}.stl',force='mesh')
        b=s.BoundingBox()
        validation['parts'][name]={'valid_brep':s.isValid(),'solids':len(s.Solids()),'volume_mm3':s.Volume(),
            'watertight':bool(mesh.is_watertight),'winding_consistent':bool(mesh.is_winding_consistent),
            'mesh_components':len(mesh.split(only_watertight=False)),'bounds_mm':[[b.xmin,b.ymin,b.zmin],[b.xmax,b.ymax,b.zmax]],
            'print_size_mm':print_mesh.extents.tolist(),'print_min_xyz_mm':print_mesh.bounds[0].tolist(),
            'print_watertight':bool(print_mesh.is_watertight)}
    active={n:s for n,s in parts.items() if n not in ('05_fit_coupon','06_mount_test')}
    for (a,sa),(b,sb) in itertools.combinations(active.items(),2):
        v=common(sa,sb)
        validation['intersections_mm3'][a+' / '+b]=v
    # Exact containment proof: the chassis outside the mounting fork has
    # no material in the reserved PCB/front module envelope.
    outside_mount=active['01_body'].cut(refs['mount_fork'])
    validation['reserved_envelope_proof_mm3']=common(outside_mount,refs['module_keepout'])
    module_local=module_reference.components()
    for a,sa in active.items():
        print('Check module',a,flush=True)
        v=0;clashes=[]
        local_sa=sa.translate(tuple(-v for v in d['origin_mm'])).rotate((0,0,0),(1,0,0),p['tilt_back_deg']-90)
        local_mount=refs['mount_fork'].translate(tuple(-v for v in d['origin_mm'])).rotate((0,0,0),(1,0,0),p['tilt_back_deg']-90)
        for component,factory in module_local.items():
            b=factory.BoundingBox()
            corner_r=math.hypot(max(abs(b.xmin),abs(b.xmax)),max(abs(b.ymin),abs(b.ymax)))
            if a=='01_body' and corner_r<=25.5-1e-5 and b.zmin>=-22.6-1e-5 and b.zmax<=15-1e-5:
                vv=0. # fixed body is excluded from the proven PCB keepout
            else:vv=common(local_sa,factory)
            v+=vv
            if vv>.005:clashes.append({'module_component':component,'volume_mm3':vv})
        validation['module_intersections_mm3'][a]=v
        if clashes:print('MODULE CLASHES',a,clashes,flush=True)
    validation['ballast_intersections_mm3']={n:common(s,refs['steel_ballast']) for n,s in active.items()}
    validation['screws_vs_printed_mm3']={n:sum(common(s,hw) for s in active.values()) for n,hw in refs.items() if n.startswith('screw_')}
    # 1 mm rearward travel is an enclosure reservation, not a measured
    # mechanical travel specification of the factory push mechanism.
    press=p['press_check_travel_mm']
    press_zone=placed(cylinder(40,17.6+press,(0,0,-1.6-press)).fuse(cylinder(15,10.6+press,(0,0,-11.2-press))))
    gap_envelope=placed(cylinder(40,16+1.6+p['front_module_gap_mm'],(0,0,-1.6-p['front_module_gap_mm'])))
    validation['front_gap_envelope_intersections_mm3']={n:common(s,gap_envelope) for n,s in active.items()}
    validation['press_reserved_travel_mm']=press
    validation['press_zone_intersections_mm3']={n:common(s,press_zone) for n,s in active.items()}
    validation['finger_probe']={'diameter_mm':16,'arc_deg':180,'positions':25,'max_intersection_mm3':0}
    print('Check finger arc',flush=True)
    for a in np.linspace(-90,90,25):
        rad=math.radians(a);sphere=cq.Solid.makeSphere(8,cq.Vector(47.5*math.sin(rad),-47.5*math.cos(rad),5.4),angleDegrees1=-90,angleDegrees2=90)
        sphere=placed(sphere)
        for s in active.values():
            validation['finger_probe']['max_intersection_mm3']=max(validation['finger_probe']['max_intersection_mm3'],common(s,sphere))
    print('Check rear cover removal',flush=True)
    validation['rear_cover_removal_mm3']={str(t):sum(common(active[n],active['02_rear_cover'].translate((0,t,0))) for n in ('01_body','07_display_carrier')) for t in (0,1,3,8,20,45)}
    front=placed(refs['module_keepout'].translate(tuple(-v for v in d['origin_mm'])).rotate((0,0,0),(1,0,0),p['tilt_back_deg']-90).translate((0,0,5)))
    validation['module_withdrawal_5mm_mm3']=sum(common(active[n],front) for n in ('01_body','07_display_carrier'))
    carrier_local=active['07_display_carrier'].translate(tuple(-v for v in d['origin_mm'])).rotate((0,0,0),(1,0,0),p['tilt_back_deg']-90)
    validation['carrier_removal_mm3']={str(t):common(active['01_body'],placed(carrier_local.translate((0,0,-t)))) for t in (0,1,2,3,5,8,12,20,30,45)}
    tools={}
    for i,(x,y) in enumerate(CARRIER):
        tool=placed(cylinder(2.5,50,(x,y,-83.15)))
        tools[f'carrier_{i+1}']=sum(common(active[n],tool) for n in ('01_body','07_display_carrier'))
    for i,(x,y) in enumerate(MOUNT):
        tool=placed(cylinder(2.5,50,(x,y,-75.65)))
        tools[f'module_{i+1}']=sum(common(active[n],tool) for n in ('01_body','07_display_carrier'))
    ports={}
    for name,bounds in module_reference.CONNECTORS.items():
        x0,y0,z0,x1,y1,z1=bounds;pad=.6
        access=placed(box(x1-x0+2*pad,y1-y0+2*pad,31,x=(x0+x1)/2,y=(y0+y1)/2,z=-45))
        ports[name]=sum(common(active[n],access) for n in ('01_body','07_display_carrier'))
    validation['connector_rear_access_pad_mm']=.6
    validation['connector_access_intersections_mm3']=ports
    validation['screwdriver_clearance_diameter_mm']=5
    validation['screwdriver_intersections_mm3']=tools
    b=cq.Compound.makeCompound(list(active.values())+[refs['module_reference']]).BoundingBox()
    d['assembly_size_without_cups_or_cable_mm']=[b.xlen,b.ylen,b.zlen]
    checks=[all(r['valid_brep'] and r['solids']==1 and r['watertight'] and r['print_watertight'] and r['winding_consistent'] and r['mesh_components']==1 for r in validation['parts'].values()),
            all(max(r['print_size_mm'][:2])<=180 for r in validation['parts'].values())]
    for k in ('intersections_mm3','module_intersections_mm3','ballast_intersections_mm3','press_zone_intersections_mm3','rear_cover_removal_mm3','screws_vs_printed_mm3','front_gap_envelope_intersections_mm3','carrier_removal_mm3','screwdriver_intersections_mm3','connector_access_intersections_mm3'):
        checks.append(all(v<.02 for v in validation[k].values()))
    checks.extend([validation['reserved_envelope_proof_mm3']<.02,validation['finger_probe']['max_intersection_mm3']<.02,validation['module_withdrawal_5mm_mm3']<.02])
    # Shape checks guard against a skinny pedestal and an open rear leg.
    outer=active['01_body'].fuse(active['02_rear_cover'])
    widths={}
    for z in (24,36,48,60):
        section=outer.intersect(box(140,160,.2,z=z))
        widths[str(z)]=section.BoundingBox().xlen
    validation['body_width_at_height_mm']=widths
    walls={}
    for x in (-25,0,25):
        for z in (24,32,40,48):
            # A thin horizontal ray must cross the rear wall in every sample.
            probe=cylinder(.3,60,(x,0,z),(0,1,0))
            walls[f'{x},{z}']=common(outer,probe)/(math.pi*.3**2)
    validation['rear_wall_ray_thickness_mm']=walls
    checks.append(min(widths.values())>=82.7)
    checks.append(min(walls.values())>=2)
    validation['all_checks_passed']=all(checks)
    validation['cad_checks_passed']=validation['all_checks_passed']
    asm=cq.Assembly(name='CrowPanel_Capsule_0_3')
    colors={'01_body':(.78,.79,.81),'02_rear_cover':(.58,.64,.72),'03_bottom_cover':(.32,.38,.45),'04_cable_clamp':(.3,.3,.35),'07_display_carrier':(.85,.56,.25)}
    for n,s in active.items():asm.add(s,name=n,color=cq.Color(*colors[n]))
    asm.add(refs['module_reference'],name='simplified_CrowPanel_reference',color=cq.Color(.12,.14,.18))
    asm.add(refs['steel_ballast'],name='steel_ballast',color=cq.Color(.75,.54,.21))
    for n,s in refs.items():
        if n.startswith(('screw_','magnet_')):asm.add(s,name=n,color=cq.Color(.67,.71,.75))
    asm.export(str(OUT/'assembly.step'))
    cq.exporters.export(module_reference.assembly(),str(OUT/'crowpanel-module-simplified.step'))
    validation['geometry_step_sha256']={str(f.relative_to(OUT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in [*sorted((OUT/'step').glob('*.step')),OUT/'assembly.step']}
    (OUT/'validation.json').write_text(json.dumps(validation,indent=2))
    (OUT/'derived.json').write_text(json.dumps(d,indent=2))
    print(json.dumps(validation,indent=2),flush=True)
    if not validation['all_checks_passed']:raise RuntimeError('CAD validation failed; inspect validation.json before printing')
    return parts,refs,d

if __name__=='__main__':build()
