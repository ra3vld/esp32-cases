"""Capsule 0.1. Reproducible exact solids; mm; CadQuery 2.8.

World X right, Y rearward, Z upward from underside of sole. Board U right,
V upward, W rearward from PCB front. W is normal to the tilted display.
Reference electronics are envelopes, not a manufacturer's STEP model.
"""
from __future__ import annotations
import hashlib
import itertools
import json
import math
import os
from functools import lru_cache
from pathlib import Path
os.environ.setdefault('XDG_CACHE_HOME', '/tmp/ldg-capsule-cad-cache')
import cadquery as cq
import numpy as np
import trimesh
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'output'
P = json.loads((ROOT / 'parameters.json').read_text())
TILT = math.radians(P['tilt_back_deg'])
ORIGIN = P['board_origin']

def box(w, d, h, x=0, y=0, z=0):
    return cq.Workplane('XY').box(w,d,h,centered=(True,True,False)).translate((x,y,z)).val()

def cyl(d, h, xyz=(0,0,0), axis=(0,0,1)):
    return cq.Solid.makeCylinder(d/2,h,cq.Vector(*xyz),cq.Vector(*axis))

def cone(d1,d2,h,xyz=(0,0,0),axis=(0,0,1)):
    return cq.Solid.makeCone(d1/2,d2/2,h,cq.Vector(*xyz),cq.Vector(*axis))

def rr(w,d,r,h,x=0,y=0,z=0):
    return cq.Workplane('XY').sketch().rect(w,d).vertices().fillet(r).finalize().extrude(h).translate((x,y,z)).val()

def ring(w,d,r,t,h,x=0,y=0,z=0):
    return rr(w,d,r,h,x,y,z).cut(rr(w-2*t,d-2*t,max(.2,r-t),h+2,x,y,z-1))

def point(u,v,w):
    return (u,ORIGIN[1]+v*math.sin(TILT)+w*math.cos(TILT),ORIGIN[2]+v*math.cos(TILT)-w*math.sin(TILT))

def board(s):
    # Mirror W, then rotate: this keeps circles and cylinders analytic.
    return s.mirror('XY').rotate((0,0,0),(1,0,0),90-P['tilt_back_deg']).translate(tuple(ORIGIN))

def on_rear(s):
    # Local x=X, local y=Z, local z=-Y. Normal axis is -Y.
    return s.rotate((0,0,0),(1,0,0),90)

def rear_ring(w,h,r,t,depth,y,zcenter):
    return on_rear(ring(w,h,r,t,depth,y=zcenter,z=-y))

def rear_rr(w,h,r,depth,y,zcenter):
    return on_rear(rr(w,h,r,depth,y=zcenter,z=-y))

def bounded_common(a,b):
    aa=a.BoundingBox();bb=b.BoundingBox()
    if any(getattr(aa,n+'max')<getattr(bb,n+'min')-1e-7 or getattr(bb,n+'max')<getattr(aa,n+'min')-1e-7 for n in ('x','y','z')):
        return 0.0
    return a.intersect(b).Volume()

def clean(s):
    return s.clean()

def exact_bounds(s):
    bounds=Bnd_Box()
    BRepBndLib.AddOptimal_s(s.wrapped,bounds,False,False)
    return cq.BoundBox(bounds)

def union(shapes):
    out=shapes[0]
    for s in shapes[1:]: out=out.fuse(s)
    return out

def ring_lip(size,depth,datum,orientation='board',cx=0,cy=0,interrupt=()):
    w,d,r=size;t=P['lip_thickness']
    s=ring(w,d,r,t,depth,x=cx,y=cy,z=datum)
    s=cq.Workplane(obj=s).edges('>Z').chamfer(P['lip_lead']).val()
    # Male interruptions are 0.3 mm larger than the female keep-out discs.
    for x,y in interrupt:s=s.cut(cyl(10+4*P['clearance'],depth+2,(x,y,datum-1)))
    return s

def mating_groove(size,depth,datum,cx=0,cy=0,interrupt=()):
    w,d,r=size;g=P['clearance'];t=P['lip_thickness']
    s=rr(w+2*g,d+2*g,r+g,depth,x=cx,y=cy,z=datum)
    s=s.cut(rr(w-2*t-2*g,d-2*t-2*g,r-t-g,depth+2,x=cx,y=cy,z=datum-1))
    for x,y in interrupt:s=s.cut(cyl(10+2*g,depth+2,(x,y,datum-1)))
    return s

def to_board(s):
    return s.translate(tuple(-v for v in ORIGIN)).rotate((0,0,0),(1,0,0),P['tilt_back_deg']-90).mirror('XY')

@lru_cache(maxsize=2)
def rounded_skin(inset=0):
    # Exact normal offset: planes, cylinders and spherical corner patches.
    # No interpolated sections, end cap or control points can create ripples.
    dimensions=[v-2*inset for v in P['skin_size']]
    return cq.Workplane('XY').box(*dimensions).edges().fillet(P['skin_radius']-inset).translate(tuple(P['skin_center'])).val()

@lru_cache(maxsize=1)
def capsule_outer():
    s=rounded_skin().intersect(board(box(400,400,300,z=P['frame_front_w'])))
    front_edges=[e for e in s.Edges() if abs(to_board(e).Center().z-P['frame_front_w'])<1e-5]
    # Constant-radius rolling fillet around the nose also rounds the two
    # footprint corners; both halves inherit it before the horizontal split.
    s=cq.Workplane(obj=s).newObject(front_edges).fillet(P['front_skin_fillet']).val()
    return clean(s.intersect(box(400,300,200,y=80)))

@lru_cache(maxsize=1)
def local_outer():
    return to_board(capsule_outer())

def cavity():
    return rounded_skin(P['wall']).intersect(board(box(400,400,300,z=.5))).intersect(box(400,300,200,y=80,z=3.5))

def sole_blank():
    # Both sides of the horizontal split inherit the very same outer skin.
    return capsule_outer().intersect(box(400,300,P['sole_thickness'],y=80))

@lru_cache(maxsize=24)
def joint_profile(name,inset=0):
    """Closed perimeter in a local XY plane. Positive inset points inward."""
    if name=='sole':
        w,d,r=P['sole_lip_size']
        shape=rr(w,d,r,.01,y=P['sole_lip_center_y'])
        face=min(shape.Faces(),key=lambda f:f.Center().z)
        wire=face.outerWire()
    else:
        shape=capsule_outer().intersect(box(400,300,200,y=80,z=P['sole_thickness']+P['clearance']))
        if name=='front':shape=to_board(shape);datum=P['frame_back_w']
        else:shape=shape.rotate((0,0,0),(1,0,0),-90);datum=-P['rear_plane_y']
        wires=cq.Workplane('XY').newObject([shape]).section(datum).wires().vals()
        wire=max(wires,key=lambda w:cq.Face.makeFromWires(w).Area()).translate((0,0,-datum))
    if inset:
        wire=max(wire.offset2D(-inset),key=lambda w:cq.Face.makeFromWires(w).Area())
        # STEP has no native offset-of-ellipse curve. Convert explicitly before
        # extruding; otherwise a valid in-memory solid can reload as an open
        # shell after the STEP translator drops these implicit curves.
        if name=='front':wire=wire.toNURBS()
    return wire

def skirt_inset(name):
    return 0 if name=='sole' else P['rear_lip_inset'] if name=='rear' else P['wall']

def profile_prism(name,inset,depth,datum):
    return cq.Solid.extrudeLinear(joint_profile(name,inset),[],cq.Vector(0,0,depth)).translate((0,0,datum))

def cover_lip(name):
    inset=skirt_inset(name);h=P['lip_height']
    lip=cq.Solid.extrudeLinear(joint_profile(name,inset),[joint_profile(name,inset+P['lip_thickness'])],cq.Vector(0,0,h))
    lip=cq.Workplane(obj=lip).edges('>Z').chamfer(P['lip_lead']).val()
    if name=='front':return board(lip.translate((0,0,P['frame_back_w'])))
    if name=='rear':return on_rear(lip.translate((0,0,-P['rear_plane_y'])))
    return lip.translate((0,0,P['sole_thickness']))

def receiver_opening(name):
    datum={'front':P['frame_back_w'],'rear':-P['rear_plane_y'],'sole':P['sole_thickness']}[name]
    inset=skirt_inset(name)-P['clearance']
    cut=profile_prism(name,inset,P['lip_height']+P['clearance'],datum)
    if name=='sole':
        # The inside is already open in the perimeter seat. Preserve the
        # bottom edge of the rear socket inside this loop, rather than sawing
        # it off with an unnecessary full-interior clearance cutter.
        cut=cut.cut(profile_prism(name,P['lip_thickness']+P['clearance'],P['lip_height']+2,datum-1))
    return board(cut) if name=='front' else on_rear(cut) if name=='rear' else cut

def frame():
    p=P;fb=p['frame_back_w'];pcb_w=-p['display_forward_mm']
    # Slice both sides of the front joint from the same finished skin. An
    # independent rounded rectangle cannot follow its oblique spherical cut.
    f=to_board(capsule_outer().intersect(board(box(400,400,fb-p['frame_front_w'],z=p['frame_front_w'])))
               .intersect(box(400,300,200,y=80,z=p['sole_thickness']+p['clearance'])))
    # Back-side reveal eases touch access, front opening stays 76.5 x 52.
    f=f.cut(rr(*p['window_size'],1.5,20,x=p['window_center_u'],z=-10))
    # Back-side pocket accepts the stock glass after moving the PCB forward.
    # Keep 0.3 mm normal and edge clearance; glass carries no frame load.
    glass_pocket_w=p['glass_front_w']-p['display_forward_mm']-p['clearance']
    f=f.cut(box(p['glass_size'][0]+2*p['clearance'],p['glass_size'][1]+2*p['clearance'],
                20,z=glass_pocket_w))
    for u in (-p['mount_pitch'][0]/2,p['mount_pitch'][0]/2):
        for v in (-p['mount_pitch'][1]/2,p['mount_pitch'][1]/2):
            f=f.fuse(cyl(8,pcb_w-fb,(u,v,fb)))
            f=f.cut(cyl(p['insert_pilot'],4.4,(u,v,pcb_w-4.4)))
    for u,v in p['frame_screws']:
        # Blind bosses: both the PCB and bezel screws are installed internally.
        boss=cyl(p['frame_boss_diameter'],p['frame_boss_end_w']-fb,(u,v,fb))
        # The PCB is closer to these fixed-position frame fasteners now.
        # Relieve only the inward flank of this boss, preserving PCB supports.
        keepout=box(p['pcb_width']+2*p['clearance'],p['pcb_height']+2*p['clearance'],
                    20,z=pcb_w-p['clearance'])
        f=f.fuse(boss.cut(keepout))
        f=f.cut(cyl(p['insert_pilot'],4.4,(u,v,p['frame_boss_end_w']-4.4)))
    # Optical channels are separate closed sleeves, not a shared light cavity.
    for uv,d in ((p['ldr_uv'],p['ldr_hole']),(p['led_uv'],p['led_hole'])):
        u,v=uv
        sleeve_end=fb+2.2-p['display_forward_mm']
        f=f.fuse(cyl(8,sleeve_end-fb,(u,v,fb)))
        f=f.cut(cone(d,6,sleeve_end-p['frame_front_w'],(u,v,p['frame_front_w'])))
    return clean(board(f).fuse(cover_lip('front')))

def button_access_paths():
    """Plain Ø2 bores through the rear skin; no projecting guide tubes."""
    result=[]
    for u,v in P['buttons_uv']:
        lo=P['button_top_w'];hi=P['outer_depth']+50
        for _ in range(28):
            mid=(lo+hi)/2
            if local_outer().isInside(cq.Vector(u,v,mid)):lo=mid
            else:hi=mid
        end=hi+1.5
        start=P['button_top_w']+P['clearance']
        result.append(board(cyl(P['button_access'],end-start+3,(u,v,start-1))))
    return result

def frame_tool_paths(diameter):
    # Slightly inward tool approach keeps the continuous outer cheek intact.
    # Use a 2 mm ball-end hex key; the approach angle stays below 10 degrees.
    paths=[]
    for u,v in P['frame_screws']:
        end=(P['rear_plane_y']+1-ORIGIN[1]-v*math.sin(TILT))/math.cos(TILT)
        start_xyz=point(u,v,P['frame_screw_seat_w']+1.7)
        end_xyz=point(math.copysign(P['frame_tool_entry_u'],u),v,end)
        delta=np.array(end_xyz)-np.array(start_xyz);length=float(np.linalg.norm(delta))
        paths.append(cyl(diameter,length,start_xyz,tuple(delta/length)))
    return paths

def body_and_rear(outer,void):
    p=P;rd=p['rear_plane_y'];sz=p['sole_thickness']+p['clearance']
    shell=outer.cut(void)
    passage=board(rr(*p['front_opening'],p['front_opening_radius'],1.2-p['frame_back_w'],z=p['frame_back_w']-.2))
    shell=shell.cut(passage)
    main=shell.intersect(box(400,rd-p['clearance'],200,y=(rd-p['clearance'])/2,z=sz))
    main=main.intersect(board(box(400,400,300,z=p['frame_back_w']+p['clearance'])))
    rear=shell.intersect(box(400,30,200,y=rd+15,z=sz))
    # Only perimeter seats remain at the bottom; the centre is entirely open.
    bottom=outer.intersect(box(400,300,p['bottom_receiver_top_z']-sz,y=80,z=sz))
    bottom=bottom.cut(profile_prism('sole',p['lip_thickness']+p['clearance'],20,sz-1))
    main=main.fuse(bottom.intersect(box(400,rd-p['clearance'],200,y=(rd-p['clearance'])/2,z=sz))
                   .intersect(board(box(400,400,300,z=p['frame_back_w']+p['clearance']))))
    rear=rear.fuse(bottom.intersect(box(400,30,200,y=rd+15,z=sz)))
    # The rear socket inherits the actual outer skin, including its bottom
    # corners. No floating inner rectangle and no floor cross-braces are used.
    inner=skirt_inset('rear')+p['lip_thickness']+p['clearance']
    receiver=outer.intersect(box(400,rd-p['clearance']-p['rear_receiver_start_y'],200,
                               y=(rd-p['clearance']+p['rear_receiver_start_y'])/2,z=sz))
    receiver=receiver.cut(on_rear(profile_prism('rear',inner,30,-rd-1)))
    main=main.fuse(receiver)
    # A full annular bridge joins the rear skirt to the curved lid skin.
    bridge=outer.intersect(box(400,30,200,y=rd+15,z=sz))
    bridge=bridge.cut(on_rear(profile_prism('rear',skirt_inset('rear')+p['lip_thickness'],40,-p['outer_depth']-1)))
    rear=rear.fuse(bridge).fuse(cover_lip('rear'))
    main=main.cut(receiver_opening('front')).cut(receiver_opening('rear')).cut(receiver_opening('sole'))
    rear=rear.cut(receiver_opening('sole'))
    for u,v in p['frame_screws']:
        end=p['frame_boss_end_w'];seat=p['frame_screw_seat_w']
        main=main.cut(board(cyl(p['frame_boss_diameter']+2*p['clearance'],end-p['frame_back_w'],(u,v,p['frame_back_w']))))
        root=box(24,6,seat-end,x=math.copysign(63,u),y=v,z=end)
        main=main.fuse(board(cyl(10,seat-end,(u,v,end)).fuse(root)).intersect(outer))
        main=main.cut(board(cyl(p['m3_clearance'],8,(u,v,end))))
    # Flat inner sides clear the PCB and its screw heads. The cover's blind
    # bosses stop 0.3 mm ahead of the PCB rather than entering its edge.
    pcb_support_width=max(p['pcb_width'],p['mount_pitch'][0]+8)
    main=main.cut(board(box(pcb_support_width+2*p['clearance'],p['pcb_height']+2*p['clearance'],
                            p['frame_screw_seat_w']+2,z=-p['clearance'])))
    # 14 mm posts with full-height saddles. Saddles stop 0.6 mm before the
    # skirt tip; the continuous skirt never passes through a screw root.
    for x,z in p['rear_screws']:
        post=cyl(p['rear_post_diameter'],rd-p['rear_post_start_y'],(x,p['rear_post_start_y'],z),(0,1,0))
        root=rr(38,p['rear_root_end_y']-p['rear_post_start_y'],2,p['rear_root_height'],
                x=math.copysign(abs(x)+12,x),y=(p['rear_root_end_y']+p['rear_post_start_y'])/2,z=z-p['rear_root_height']/2)
        main=main.fuse(post.fuse(root).intersect(outer))
        main=main.cut(cyl(p['insert_pilot'],4.4,(x,rd-4.4,z),(0,1,0)))
        rear=rear.fuse(cyl(p['rear_post_diameter'],p['outer_depth']-rd,(x,rd,z),(0,1,0)).intersect(outer))
    # Bottom screw saddles lie above the skirt, leaving it uninterrupted.
    for x,y in p['sole_screws']:
        post=cyl(12,9.3,(x,y,sz)).fuse(box(24,12,6,x=math.copysign(61,x),y=y,z=6.8))
        main=main.fuse(post.intersect(outer))
        main=main.cut(cyl(p['insert_pilot'],4.4,(x,y,sz)))
    usb=board(box(55,15,11,x=p['usb_uv_w'][0]-25,y=p['usb_uv_w'][1],z=-1.5))
    main=main.cut(usb);rear=rear.cut(usb)
    for path in button_access_paths():rear=rear.cut(path)
    # No long extraction slots, floor sweeps or rear ventilation slits.
    for u,v in p['frame_screws']:
        main=main.cut(board(cyl(6.2,1.9,(u,v,p['frame_screw_seat_w']))))
    for path in frame_tool_paths(p['frame_tool_clearance']):main=main.cut(path)
    for x,z in p['rear_screws']:
        rear=rear.cut(cyl(p['m3_clearance'],30,(x,rd-1,z),(0,1,0)))
        rear=rear.cut(cyl(6.2,25,(x,p['rear_screw_seat_y'],z),(0,1,0)))
    cx,cz=p['cable_center_xz']
    cable=cyl(p['cable_diameter']+.6,50,(cx,60,cz),(0,1,0))
    main=main.cut(cable);rear=rear.cut(cable)
    for x,y in p['suction_centers']:
        main=main.cut(cyl(p['suction_head_clearance'],8,(x,y,2)))
        rear=rear.cut(cyl(p['suction_head_clearance'],8,(x,y,2)))
    return clean(main),clean(rear)

def clamp_channel():
    cx,cz=P['cable_center_xz'];cy=P['clamp_center_y'];depth=P['clamp_size'][1]
    return cyl(P['cable_diameter'],depth+2,(cx,cy-depth/2-1,cz),(0,1,0)).fuse(
        cone(5.8,5,.6,(cx,cy-depth/2,cz),(0,1,0))).fuse(
        cone(5.8,5,.6,(cx,cy+depth/2,cz),(0,-1,0)))

def sole():
    p=P;s=cq.Workplane(obj=sole_blank()).edges('<Z').fillet(.9).val()
    s=s.fuse(cover_lip('sole'))
    for x,y in p['sole_screws']:
        s=s.fuse(cyl(9,.3,(x,y,3.2))).cut(cyl(p['m3_clearance'],15,(x,y,-1))).cut(cyl(6.2,1.8,(x,y,-.1)))
    for x,y in p['suction_centers']:
        s=s.cut(cyl(p['suction_hole'],15,(x,y,-1))).cut(cyl(p['suction_head_clearance'],10,(x,y,p['suction_web'])))
    for x,y in p['cassette_screws']:
        s=s.fuse(cyl(9,2.4,(x,y,3.2))).cut(cyl(p['insert_pilot'],4.4,(x,y,1.2)))
    cx,cz=p['cable_center_xz'];cy=p['clamp_center_y'];w,d,_=p['clamp_size']
    base=rr(w,d,2,p['clamp_base_top_z']-3.2,x=cx,y=cy,z=3.2).cut(clamp_channel())
    s=s.fuse(base)
    for x,y in p['clamp_screws']:s=s.cut(cyl(p['insert_pilot'],4.4,(x,y,p['clamp_base_top_z']-4.4)))
    for x in (cx-5.5,cx+5.5):s=s.cut(cyl(p['clamp_pin_diameter']+.6,2.2,(x,cy,p['clamp_cap_bottom_z']-2.3)))
    return clean(s)

def cassette():
    p=P;cx,cy=p['interface_center_xy'];bottom=p['interface_bottom_z']-2.4
    width=p['interface_size'][0]+4.2;depth=p['interface_size'][1]+4.2
    s=rr(width,depth,2.5,2.4,x=cx,y=cy,z=bottom)
    rim=ring(width,depth,2.5,1.6,3,x=cx,y=cy,z=bottom+2.4)
    s=s.fuse(rim)
    for x,y in p['cassette_screws']:
        tab=cyl(9,2.4,(x,y,bottom))
        # Narrow connecting arms stay below the PCB/module support plane.
        arm=box(abs(x-cx)+1,7,2.4,x=(x+cx)/2,y=y,z=bottom)
        s=s.fuse(tab).fuse(arm)
        s=s.cut(cyl(p['m3_clearance'],9,(x,y,4)))
    # Two 2.5 mm ties, recessed on the underside, go outside the module edges.
    for x in (cx-p['interface_size'][0]*.3,cx+p['interface_size'][0]*.3):
        for yy in (cy-depth/2+1.3,cy+depth/2-1.3):
            s=s.cut(box(3.2,1.4,8,x=x,y=yy,z=5))
        s=s.cut(box(3.2,depth-1.2,1.3,x=x,y=cy,z=bottom-.1))
    return clean(s)

def clamp():
    p=P;cx,cz=p['cable_center_xz'];cy=p['clamp_center_y'];w,d,h=p['clamp_size'];bottom=p['clamp_cap_bottom_z']
    s=rr(w,d,2,h,x=cx,y=cy,z=bottom).cut(clamp_channel())
    for x,y in p['clamp_screws']:
        s=s.cut(cyl(p['m3_clearance'],h+2,(x,y,bottom-1))).cut(cyl(6.2,3,(x,y,p['clamp_seat_z'])))
    for x in (cx-5.5,cx+5.5):
        pin=cyl(p['clamp_pin_diameter'],2,(x,cy,bottom-2))
        pin=cq.Workplane(obj=pin).edges('<Z').chamfer(.2).val();s=s.fuse(pin)
    return clean(s)

def references():
    p=P;pcb=rr(p['pcb_width'],p['pcb_height'],2,p['pcb_thickness'])
    for x in (-p['mount_pitch'][0]/2,p['mount_pitch'][0]/2):
        for y in (-p['mount_pitch'][1]/2,p['mount_pitch'][1]/2):pcb=pcb.cut(cyl(p['pcb_hole_diameter'],5,(x,y,-1)))
    refs={'ref_pcb':board(pcb),'ref_glass':board(box(*p['glass_size'],-.8-p['glass_front_w'],z=p['glass_front_w'])),
          'ref_screen':board(box(*p['active_size'],.12,x=p['active_center_u'],z=p['glass_front_w']-.12)),
          'ref_components':board(box(p['pcb_width']-14.5,p['pcb_height']-19.9,p['component_back_w']-p['pcb_thickness'],z=p['pcb_thickness'])),
          'ref_interface':box(*p['interface_size'],x=p['interface_center_xy'][0],y=p['interface_center_xy'][1],z=p['interface_bottom_z']),
          'ref_usb':board(box(5,7.8,3.2,x=p['usb_uv_w'][0],y=p['usb_uv_w'][1],z=1.6)),
          'ref_usb_plug':board(box(*p['usb_plug_size'],x=p['usb_uv_w'][0]-14,y=p['usb_uv_w'][1],z=p['usb_uv_w'][2]-p['usb_plug_size'][2]/2)),
          'ref_cable':cyl(p['cable_diameter'],32,(p['cable_center_xz'][0],61,p['cable_center_xz'][1]),(0,1,0))}
    for i,(u,v) in enumerate(p['buttons_uv']):
        refs[f'ref_button_{i}']=board(box(3,4,3,x=u,y=v,z=1.6))
    for name,uv,size in [('led',p['led_uv'],5),('ldr',p['ldr_uv'],3.6)]:
        refs['ref_'+name]=board(box(size,size,2.2,x=uv[0],y=uv[1],z=-2.2))
    for i,(x,y) in enumerate(p['suction_centers']):
        cup=cone(20,7,5,(x,y,-7)).fuse(cyl(4,3,(x,y,-2))).fuse(cyl(6,4,(x,y,0))).fuse(cone(8,6,1.5,(x,y,4)))
        refs[f'ref_cup_{i}']=cup
    # Estimated antenna occupied region, ONLY a keep-out for added module.
    refs['ref_antenna_zone']=board(box(7,18,10,x=47.25,z=-.5))
    # Move the complete PCB stack along its normal; the housing stays fixed.
    shift=(0,-p['display_forward_mm']*math.cos(TILT),p['display_forward_mm']*math.sin(TILT))
    moving={'ref_pcb','ref_glass','ref_screen','ref_components','ref_usb','ref_usb_plug','ref_led','ref_ldr','ref_antenna_zone'}
    for name in refs:
        if name in moving or name.startswith('ref_button_'):refs[name]=refs[name].translate(shift)
    return refs

def hardware():
    hw={};p=P
    def screw(length,xyz,axis):
        a=cq.Vector(*axis);pt=cq.Vector(*xyz)
        return cyl(3,length,xyz,axis).fuse(cyl(5.7,1.65,(pt-a*1.65).toTuple(),axis))
    def insert(xyz,axis):
        return cyl(4.6,4,xyz,axis).cut(cyl(3,4,xyz,axis))
    for i,(u,v) in enumerate(p['frame_screws']):
        hw[f'screw_frame_{i}']=board(screw(p['frame_screw_length'],(u,v,p['frame_screw_seat_w']),(0,0,-1)))
        hw[f'insert_frame_{i}']=board(insert((u,v,p['frame_boss_end_w']),(0,0,-1)))
    for i,(u,v) in enumerate(itertools.product((-47.25,47.25),(-23.95,23.95))):
        hw[f'screw_pcb_{i}']=board(screw(6,(u,v,1.6),(0,0,-1)))
        hw[f'insert_pcb_{i}']=board(insert((u,v,-4),(0,0,1)))
    for i,(x,z) in enumerate(p['rear_screws']):
        hw[f'screw_rear_{i}']=screw(12,(x,p['rear_screw_seat_y'],z),(0,-1,0))
        hw[f'insert_rear_{i}']=insert((x,p['rear_plane_y']-4,z),(0,1,0))
    for i,(x,y) in enumerate(p['sole_screws']):
        hw[f'screw_sole_{i}']=screw(6,(x,y,1.7),(0,0,1))
        hw[f'insert_sole_{i}']=insert((x,y,3.5),(0,0,1))
    for i,(x,y) in enumerate(p['cassette_screws']):
        hw[f'screw_cassette_{i}']=screw(6,(x,y,8),(0,0,-1))
        hw[f'insert_cassette_{i}']=insert((x,y,1.6),(0,0,1))
    for i,(x,y) in enumerate(p['clamp_screws']):
        hw[f'screw_clamp_{i}']=screw(8,(x,y,p['clamp_seat_z']),(0,0,-1))
        hw[f'insert_clamp_{i}']=insert((x,y,p['clamp_base_top_z']-4),(0,0,1))
    shift=(0,-p['display_forward_mm']*math.cos(TILT),p['display_forward_mm']*math.sin(TILT))
    for name in hw:
        if name.startswith(('screw_pcb_','insert_pcb_')):hw[name]=hw[name].translate(shift)
    return hw

def fit_coupon():
    # Three test pieces joined ONLY by thin, easily cut break-away bridges.
    s=rr(102,20,3,2.4,y=15)
    for i,d in enumerate((5.8,6.0,6.2)):
        x=-37+i*14
        s=s.cut(cyl(d,8,(x,15,-1)))
        s=s.cut(cyl(11,4,(x,15,2)))
    for i,d in enumerate((4.0,4.2,4.4)):
        x=10+i*14;s=s.fuse(cyl(10,6,(x,15,2.4)))
        s=s.cut(cyl(d,5,(x,15,3.4)))
    for x in (-29,29):
        for xx in (x-12,x+12):s=s.fuse(box(2,4.4,.6,x=xx,y=3))
    male=rr(42,22,3,2.4,x=-29,y=-10).fuse(ring_lip([35,15,3],3,2.4,cx=-29,cy=-10))
    socket=rr(42,22,3,5.7,x=29,y=-10)
    socket=socket.cut(rr(35.6,15.6,3.3,3.3,x=29,y=-10,z=2.4))
    socket=socket.cut(rr(31.2,11.2,1.1,8,x=29,y=-10,z=-1))
    s=s.fuse(male).fuse(socket)
    return clean(s)

def board_coupon(f):
    # Actual mounting and optical positions; thin partial shell includes USB bay.
    coupon=f
    u,v,w=P['usb_uv_w']
    patch=board(box(6,19,15,x=-58,y=v,z=-4.8))
    patch=patch.cut(board(box(20,15,11,x=-58,y=v,z=-1.5)))
    coupon=coupon.fuse(patch)
    return clean(coupon)

def make():
    outer=capsule_outer();void=cavity()
    print('outer/cavity',outer.isValid(),void.isValid(),flush=True)
    f=frame();print('frame',len(f.Solids()),flush=True)
    b,r=body_and_rear(outer,void);print('body/rear',len(b.Solids()),len(r.Solids()),flush=True)
    parts={'01_front_frame':f,'02_body':b,'03_rear_cover':r,'04_sole':sole(),
           '05_interface_cassette':cassette(),'06_cable_clamp':clamp(),
           '07_fit_coupon':fit_coupon(),'08_board_coupon':board_coupon(f)}
    return parts,references(),hardware()

def print_pose(name,s):
    if name in ('01_front_frame','08_board_coupon'):
        s=s.translate(tuple(-v for v in ORIGIN)).rotate((0,0,0),(1,0,0),P['tilt_back_deg']-90).mirror('XY')
    elif name=='03_rear_cover':s=s.rotate((0,0,0),(1,0,0),-90)
    elif name=='06_cable_clamp':s=s.rotate((0,0,0),(1,0,0),180)
    bb=exact_bounds(s);return s.translate((-(bb.xmin+bb.xmax)/2,-(bb.ymin+bb.ymax)/2,-bb.zmin))

def run(front_only=False):
    for folder in ('step','stl','print_stl','reference_step'): (OUT/folder).mkdir(parents=True,exist_ok=True)
    if front_only:
        parts={f.stem:cq.importers.importStep(str(f)).val() for f in sorted((OUT/'step').glob('*.step'))}
        parts['01_front_frame']=frame();parts['08_board_coupon']=board_coupon(parts['01_front_frame'])
        refs,hw=references(),hardware()
    else:parts,refs,hw=make()
    manifest=[]
    for name,s in parts.items():
        valid=s.isValid();solids=len(s.Solids());print(name,valid,solids,round(s.Volume(),2),flush=True)
        if front_only and name not in ('01_front_frame','08_board_coupon'):
            bb=exact_bounds(s);manifest.append({'name':name,'valid':valid,'solids':solids,'volume_mm3':s.Volume(),'bounds_mm':[bb.xlen,bb.ylen,bb.zlen]})
            continue
        cq.exporters.export(s,str(OUT/'step'/f'{name}.step'))
        reloaded=cq.importers.importStep(str(OUT/'step'/f'{name}.step')).val()
        assert reloaded.isValid() and len(reloaded.Solids())==1,(name,'STEP is not one valid solid')
        assert abs(reloaded.Volume()-s.Volume())/s.Volume()<.0001,(name,'STEP round-trip volume')
        for directory,shape in [('stl',s),('print_stl',print_pose(name,s))]:
            from mesh_quality import triangulate
            vertices,faces,_=triangulate(shape,P['stl_linear_tolerance'],P['stl_angular_tolerance'])
            mesh=trimesh.Trimesh(vertices,faces,process=True)
            # Weld numerical duplicates below 0.00001 mm; no hole filling or
            # smoothing is used, and the BREP surfaces are never changed.
            mesh.merge_vertices(digits_vertex=5)
            mesh.update_faces(mesh.nondegenerate_faces())
            mesh.update_faces(mesh.unique_faces())
            mesh.remove_unreferenced_vertices()
            mesh.export(str(OUT/directory/f'{name}.stl'))
        bb=exact_bounds(s);manifest.append({'name':name,'valid':valid,'solids':solids,'volume_mm3':s.Volume(),'bounds_mm':[bb.xlen,bb.ylen,bb.zlen]})
    assembly=cq.Assembly(name='LDG_capsule_v01')
    colors=[(.18,.21,.25),(.86,.86,.83),(.74,.77,.78),(.74,.77,.78),(.82,.49,.24),(.3,.34,.38)]
    for (name,s),c in zip(list(parts.items())[:6],colors):assembly.add(s,name=name,color=cq.Color(*c))
    for name,s in refs.items():
        if name in ('ref_components','ref_antenna_zone','ref_usb_plug'):continue
        color=(.14,.48,.33) if name=='ref_pcb' else (.05,.09,.13) if name in ('ref_glass','ref_screen') else (.35,.46,.48)
        assembly.add(s,name=name,color=cq.Color(*color))
    for name,s in hw.items():assembly.add(s,name=name,color=cq.Color(*((.76,.55,.23) if name.startswith('insert') else (.6,.62,.65))))
    assembly.save(str(OUT/'assembly.step'))
    for name,s in {**refs,**hw}.items():cq.exporters.export(s,str(OUT/'reference_step'/f'{name}.step'))
    (OUT/'parts.json').write_text(json.dumps(manifest,indent=2)+'\n')
    bounds=exact_bounds(cq.Compound.makeCompound(list(parts.values())[:6]))
    (OUT/'dimensions.json').write_text(json.dumps({
        'enclosure_xyz_mm':[round(bounds.xlen,3),round(bounds.ylen,3),round(bounds.zlen,3)],
        'bounds_mm':{k:round(getattr(bounds,k),3) for k in ('xmin','xmax','ymin','ymax','zmin','zmax')},
        'front_exposed_screws':0,'additional_glass_overlay':False,
        'tilt_back_deg':P['tilt_back_deg']},indent=2)+'\n')
    print('EXPORT COMPLETE',flush=True)

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--front-only',action='store_true',help='Replace only front frame and fit coupon; preserve all other exported parts')
    args=parser.parse_args()
    run(args.front_only or (ROOT/'compatibility-baseline.json').exists())
