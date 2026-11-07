"""CC BY 4.0: independent primitive reference for the CrowPanel 2.1 module.

Coordinates: display-plane X/Y, +Z toward the user, millimetres.
Only dimensions and mounting coordinates are retained from the mechanical
reference. No imported vendor surfaces, topology or detailed PCB artwork.
"""
import cadquery as cq

MOUNT = [(0., 5.95), (-5.1528511525, -2.975), (5.1528511525, -2.975)]
DIAMETER = 79.0
BACK_Z = -21.02
FRONT_Z = 12.4
MOUNT_FACE_Z = -19.5
THREAD_DEPTH = 6.0
# Simple connector envelopes: xmin, ymin, zmin, xmax, ymax, zmax.
CONNECTORS = {
    'upper_left_socket': (-8.224924, 14.085, -18.92, -1.874924, 17.835, -14.5),
    'upper_left_plug': (-8.5, 14.9, -21.02, -1.6, 17.7, -16.02),
    'upper_right_socket': (1.875076, 14.085, -18.92, 8.225076, 17.835, -14.5),
    'upper_right_plug': (1.6, 14.9, -21.02, 8.5, 17.7, -16.02),
    'left_lower': (-18.255004, -7.75, -19.5, -13.744996, -2.25, -14.5),
    'left_upper': (-18.255004, 2.25, -19.5, -13.744996, 7.75, -14.5),
    'right_socket': (10.61, -3.175076, -18.92, 14.36, 3.174924, -14.5),
    'right_plug': (11.425, -3.45, -21.02, 14.225, 3.45, -16.02),
    'right_board_port': (16.7, -5.7, -16.51, 21.16, 4.7, -14.5),
    'lower_port': (-5.840051, -20.5, -16.5, 4.729949, -15.6, -14.48),
}


def cylinder(radius, height, z, x=0., y=0.):
    return cq.Solid.makeCylinder(radius, height, cq.Vector(x, y, z))


def envelope(bounds):
    x0, y0, z0, x1, y1, z1 = bounds
    return cq.Solid.makeBox(x1-x0, y1-y0, z1-z0, cq.Vector(x0, y0, z0))


def components():
    ring = cylinder(39.5, 14., -1.6).cut(cylinder(35.625, 14.2, -1.7))
    rear = cylinder(14., 9.5, -11.1).fuse(cylinder(38.95, 5.1, -1.6))
    parts = {
        'encoder_ring': ring,
        'display_glass': cylinder(35.625, .25, 11.4),
        'rear_housing': rear,
        'rear_pcb': cylinder(24., 1., -14.5),
        'centre_hub': cylinder(8.45, 16.9, -15.),
    }
    for i, (x, y) in enumerate(MOUNT):
        post = cylinder(2.35, 8., MOUNT_FACE_Z, x, y)
        bore = cylinder(1.5, THREAD_DEPTH+.01, MOUNT_FACE_Z-.01, x, y)
        parts[f'm3_mount_{i+1}'] = post.cut(bore)
    parts.update({f'connector_{name}': envelope(bounds) for name, bounds in CONNECTORS.items()})
    return parts


def assembly():
    return cq.Compound.makeCompound(list(components().values()))


def validate():
    parts = components()
    bounds = assembly().BoundingBox()
    dimensions = [bounds.xlen, bounds.ylen, bounds.zlen]
    expected = [DIAMETER, DIAMETER, FRONT_Z-BACK_Z]
    assert all(abs(a-b)<1e-5 for a,b in zip(dimensions,expected)), dimensions
    assert all(s.isValid() and len(s.Solids())==1 for s in parts.values())
    for i,(x,y) in enumerate(MOUNT):
        part=parts[f'm3_mount_{i+1}']
        assert abs(part.BoundingBox().zmin-MOUNT_FACE_Z)<1e-6
        assert part.distance(cq.Vertex.makeVertex(x,y,MOUNT_FACE_Z))>1.49
    return {'kind':'simplified_primitive_reference','dimensions_mm':dimensions,
            'mount_centres_xy_mm':MOUNT,'mount_face_z_mm':MOUNT_FACE_Z,
            'm3_bore_mm':3.,'thread_depth_mm':THREAD_DEPTH,'parts':len(parts)}
