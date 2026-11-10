"""Pinned D415 nominal description, distinct from device-calibrated extrinsics."""
import ast
import hashlib
import json
import operator
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
from .s2_geometry import origin


def nominal_model(config_path):
    path=Path(config_path);metadata=json.loads(path.read_text())
    source=path.parent/'official_sources'/metadata['urdf']['file']
    if hashlib.sha256(source.read_bytes()).hexdigest()!=metadata['urdf']['sha256']:
        raise ValueError('D415 official URDF hash mismatch')
    root=ET.parse(source).getroot();properties={}
    def number(expr):
        node=ast.parse(expr,mode='eval').body
        def visit(n):
            if isinstance(n,ast.Constant) and type(n.value) in (int,float):return n.value
            if isinstance(n,ast.Name):return properties[n.id]
            if isinstance(n,ast.UnaryOp) and isinstance(n.op,ast.USub):return -visit(n.operand)
            ops={ast.Add:operator.add,ast.Sub:operator.sub,ast.Mult:operator.mul,ast.Div:operator.truediv}
            if isinstance(n,ast.BinOp) and type(n.op) in ops:return ops[type(n.op)](visit(n.left),visit(n.right))
            raise ValueError('unsupported numeric Xacro expression')
        return float(visit(node))
    def expand(value):return re.sub(r'\$\{([^}]+)\}',lambda m:str(number(m[1])),value)
    def transform(element):return origin({'origin':{k:expand(v) for k,v in element.attrib.items()}})
    for p in root.iter('{http://ros.org/wiki/xacro}property'):
        properties[p.get('name')]=number(expand(p.get('value')))
    transforms={'${name}_bottom_screw_frame':np.eye(4)}
    pending=[j for j in root.iter('joint') if j.find('origin') is not None]
    while pending:
        before=len(pending)
        for joint in pending[:]:
            parent=joint.find('parent').get('link');child=joint.find('child').get('link')
            if parent not in transforms:continue
            transforms[child]=transforms[parent]@transform(joint.find('origin'));pending.remove(joint)
        if len(pending)==before:raise ValueError('unresolved D415 nominal joint chain')
    visual=next(e for e in root.iter('link') if e.get('name')=='${name}_link').find('visual')
    mesh_visual=next(e for e in visual if e.get('value')=='${use_mesh}')
    screw_from_mesh=transforms['${name}_link']@transform(mesh_visual.find('origin'))
    screw_from_color=transforms['${name}_color_optical_frame']
    color_from_mesh=np.linalg.inv(screw_from_color)@screw_from_mesh
    height=properties['d415_cam_height'];depth=properties['d415_cam_depth'];width=properties['d415_cam_width']
    # Official STL X spans width, Y points up, and its front surface is Z=0.
    rear_top=[0,height/2,-depth,1]
    return dict(source=metadata['source'],width_m=width,height_m=height,depth_m=depth,
                color_optical_from_mesh=color_from_mesh.tolist(),
                color_optical_from_depth_optical=(np.linalg.inv(screw_from_color)@transforms['${name}_depth_optical_frame']).tolist(),
                screw_from_color_optical=screw_from_color.tolist(),
                rear_top_center_in_color_optical_m=(color_from_mesh@rear_top)[:3].tolist(),
                body_geometry='official D415 peripheral STL and Xacro',
                optical_geometry='official nominal Xacro; not device factory calibration',
                mesh_sha256=metadata['mesh']['sha256'],physical_accuracy_verified=False)
