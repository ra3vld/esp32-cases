"""Absolute OCCT tessellation and exact, oriented CAD surface normals.

Only triangulations are replaced; the underlying BREP is never edited.
Sharp edges retain separate normal sets on their adjacent CAD faces.
"""
import cadquery as cq
import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepGProp import BRepGProp_Face
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepTools import BRepTools
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Sphere,GeomAbs_Cylinder
from OCP.TopAbs import TopAbs_REVERSED
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Pnt,gp_Vec

LINEAR_MM=.025
ANGULAR_RAD=.07

def triangulate(shape,linear_mm=LINEAR_MM,angular_rad=ANGULAR_RAD):
    BRepTools.Clean_s(shape.wrapped)
    BRepMesh_IncrementalMesh(shape.wrapped,linear_mm,False,angular_rad,True)
    vertices=[];triangles=[];normals=[];offset=0
    for face in shape.Faces():
        loc=TopLoc_Location();poly=BRep_Tool.Triangulation_s(face.wrapped,loc)
        if poly is None:continue
        tr=loc.Transformation()
        vs=np.array([poly.Node(i).Transformed(tr).Coord() for i in range(1,poly.NbNodes()+1)],dtype=float)
        ts=np.array([t.Get() for t in poly.Triangles()],dtype=np.int64)-1
        if face.wrapped.Orientation()==TopAbs_REVERSED:ts=ts[:,[0,2,1]]
        ns=np.zeros_like(vs)
        adaptor=BRepAdaptor_Surface(face.wrapped)
        kind=adaptor.GetType()
        weighted=np.cross(vs[ts[:,1]]-vs[ts[:,0]],vs[ts[:,2]]-vs[ts[:,0]])
        if kind in (GeomAbs_Sphere,GeomAbs_Cylinder):
            # UV derivatives vanish at sphere poles. Analytic geometric normals
            # avoid isolated black triangles at the plane/cylinder/sphere joins.
            primitive=adaptor.Sphere() if kind==GeomAbs_Sphere else adaptor.Cylinder()
            ns=vs-np.array(primitive.Location().Coord())
            if kind==GeomAbs_Cylinder:
                axis=np.array(primitive.Axis().Direction().Coord())
                ns-=np.outer(ns@axis,axis)
            if np.sum(np.einsum('ij,ij->i',ns[ts].mean(axis=1),weighted))<0:ns=-ns
        elif poly.HasUVNodes():
            surface=BRepGProp_Face(face.wrapped);point=gp_Pnt();normal=gp_Vec()
            for i in range(poly.NbNodes()):
                uv=poly.UVNode(i+1);surface.Normal(uv.X(),uv.Y(),point,normal)
                ns[i]=normal.Coord()
        lens=np.linalg.norm(ns,axis=1);missing=lens<1e-12
        if np.any(missing):
            # Singular surface poles use incident triangle normals on that face.
            accum=np.zeros_like(vs)
            for col in range(3):np.add.at(accum,ts[:,col],weighted)
            ns[missing]=accum[missing];lens=np.linalg.norm(ns,axis=1)
        ns/=np.maximum(lens[:,None],1e-30)
        vertices.append(vs);triangles.append(ts+offset);normals.append(ns);offset+=len(vs)
    return np.vstack(vertices),np.vstack(triangles),np.vstack(normals)
