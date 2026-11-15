"""Place an observed held shape only on observed empty support cells.

Unknown cells and new clutter remain unavailable. The attachment is measured,
not inferred from the centre of the object or a nominal grasp.
"""
from dataclasses import dataclass
import numpy as np
from .assistant.alignment_contracts import HeldObject
from .assistant.contracts import Stamp
from .s2_geometry import rigid


@dataclass(frozen=True)
class SupportGrid:
    stamp: Stamp
    surface_id: str
    scene_revision: str
    map_from_surface: object
    origin_xy: tuple
    resolution_m: float
    empty: object

    def __post_init__(self):
        pose=rigid(self.map_from_surface).copy()
        if not np.allclose(pose[:3,2],[0,0,1],atol=.01): raise ValueError('horizontal support required')
        empty=np.asarray(self.empty)
        if empty.dtype!=bool or empty.ndim!=2 or not 1<=empty.size<=1000000:
            raise ValueError('bounded observed-empty boolean grid required')
        if not .002<=self.resolution_m<=.05 or len(self.origin_xy)!=2 or not np.isfinite(self.origin_xy).all():
            raise ValueError('support grid geometry required')
        pose.setflags(write=False); empty=empty.copy(); empty.setflags(write=False)
        object.__setattr__(self,'map_from_surface',pose); object.__setattr__(self,'empty',empty)


def place_candidates(grid, held, *, now_s, yaw_angles=(0.,np.pi/2,np.pi,3*np.pi/2), maximum_results=16):
    if not isinstance(grid,SupportGrid) or not isinstance(held,HeldObject): raise ValueError('observed support and measured attachment required')
    grid.stamp.check(held.context,now_s,2.)
    held.check(held.context,held.object_id,now_s,held.model_revision)
    if not 1<=maximum_results<=64: raise ValueError('bounded placement search required')
    from scipy.ndimage import binary_erosion, distance_transform_edt
    from scipy.spatial.transform import Rotation
    half=np.array(held.size_m)/2
    corners=np.array([[x,y,z] for x in (-half[0],half[0]) for y in (-half[1],half[1]) for z in (-half[2],half[2])])
    if held.payload_mesh is not None: corners=np.asarray(held.payload_mesh['vertices'],float)
    results=[]
    for yaw in yaw_angles:
        if not np.isfinite(yaw): raise ValueError('finite placement yaw required')
        r=Rotation.from_euler('z',yaw).as_matrix()
        vertices=corners@r.T
        # Conservative raster erosion accounts for the full rotated payload,
        # measured attachment uncertainty and half a pixel on either boundary.
        radius=np.max(np.abs(vertices[:,:2]),axis=0)+held.translation_uncertainty_m+.003
        cells=np.ceil(radius/grid.resolution_m+.5).astype(int)
        footprint=np.ones((2*cells[1]+1,2*cells[0]+1),bool)
        valid=binary_erosion(grid.empty,structure=footprint,border_value=0)
        distance=distance_transform_edt(grid.empty)*grid.resolution_m
        yy,xx=np.nonzero(valid)
        order=sorted(range(len(xx)),key=lambda i:(-distance[yy[i],xx[i]],int(yy[i]),int(xx[i])))
        used=[]
        for i in order:
            xy=np.array(grid.origin_xy)+(np.array([xx[i],yy[i]])+.5)*grid.resolution_m
            if any(np.linalg.norm(xy-q)<.05 for q in used):continue
            used.append(xy)
            local=np.eye(4);local[:3,:3]=r;local[:3,3]=[*xy,-vertices[:,2].min()+.001]
            obj=grid.map_from_surface@local
            tcp=obj@np.linalg.inv(held.tcp_from_object)
            results.append(dict(map_from_object=obj,map_from_tcp=tcp,clearance_m=float(distance[yy[i],xx[i]]),
                support_capture_id=grid.stamp.capture_id,object_id=held.object_id,yaw_rad=float(yaw)))
            if len(used)>=maximum_results:break
    results.sort(key=lambda p:-p['clearance_m'])
    return results[:maximum_results]
