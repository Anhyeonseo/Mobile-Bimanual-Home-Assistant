"""Persistent, evidence-limited furniture changes on the reviewed navigation map.

The source map is immutable. Reidentified furniture replaces its own historical
geometry and its registered source laser cells; unassociated objects remain.
Without confirmed identity, laser cells require repeated free-space evidence.
A 2-D laser never clears an unassociated upper obstacle.
"""
import hashlib
import itertools
import json
import math
from pathlib import Path

import numpy as np

from .assistant.contracts import document
from .recorded_semantics import register_native_depth
from .s2_geometry import rigid


def confirmed_geometry_replacement(location,config):
    ids=location.get('evidence_capture_ids',[])
    return (location.get('identity_verified') is True and
            location.get('furniture_id')==config['furniture_id'] and
            location.get('reference_revision')==config['reference_revision'] and
            location.get('method')=='appearance_geometry_reidentification' and
            len(ids)>=3 and all(isinstance(v,str) and v for v in ids) and len(set(ids))==len(ids))


def furniture_laser_membership(points,centers,voxel_size,resolution):
    """Intersect source furniture voxel volumes with laser map cell volumes.

    Testing only the cell centre leaves a row of old table returns when the
    RGB-D lattice and the SLAM map have different origins. Both half-cell sizes
    participate; this is source association, not extra navigation clearance.
    """
    from scipy.spatial import ConvexHull
    points=np.asarray(points)[:,:2];hull=ConvexHull(points)
    radius=(voxel_size+resolution)/2
    limits=radius*np.abs(hull.equations[:,:2]).sum(1)
    planes=np.all(centers@hull.equations[:,:2].T+hull.equations[:,2]<limits-1e-9,axis=-1)
    bounds=np.all((centers>points.min(0)-radius+1e-9)&(centers<points.max(0)+radius-1e-9),axis=-1)
    return planes&bounds


def visible_free_boxes(boxes,depth,intrinsics,camera_to_map,margin=.025):
    """All projected pixels must see beyond all box corners; holes veto clearing."""
    import cv2
    tf=np.asarray(camera_to_map,float);rigid(tf)
    offsets=np.array(list(itertools.product((0.,1.),repeat=3)))
    result=[];h,w=depth.shape
    for low,high in boxes:
        corners=np.asarray(low)+offsets*(np.asarray(high)-low)
        cam=(corners-tf[:3,3])@tf[:3,:3]
        if np.any(cam[:,2]<=.1):result.append(False);continue
        uv=cam[:,:2]/cam[:,2:]*[intrinsics['fx'],intrinsics['fy']]+[intrinsics['ppx'],intrinsics['ppy']]
        # Require the entire projection, not a point sampled through an occluder.
        if np.any(uv<1) or np.any(uv[:,0]>=w-1) or np.any(uv[:,1]>=h-1):result.append(False);continue
        lo=np.floor(uv.min(0)).astype(int);hi=np.ceil(uv.max(0)).astype(int)
        if np.prod(hi-lo+1)>20000:result.append(False);continue
        mask=np.zeros((hi[1]-lo[1]+1,hi[0]-lo[0]+1),np.uint8)
        cv2.fillConvexPoly(mask,cv2.convexHull(np.rint(uv-lo).astype(np.int32)),1)
        observed=depth[lo[1]:hi[1]+1,lo[0]:hi[0]+1][mask>0]
        result.append(bool(len(observed)>=4 and np.all(np.isfinite(observed)) and
                           np.all(observed>cam[:,2].max()+margin)))
    return np.asarray(result,bool)


def laser_free_cells(scan,lidar_xy_yaw,shape,origin,resolution):
    """Reuse depth DDA; masked/invalid/missing rays never become free evidence."""
    from .navigation_depth import DepthVolume
    result=np.zeros(shape,np.int16);hits=np.zeros(shape,bool)
    x,y,yaw=lidar_xy_yaw
    for i,value in enumerate(scan['ranges']):
        if value is None or not math.isfinite(value) or not .25<=value<=3.5:continue
        angle=scan['angle_min']+i*scan['angle_increment']+yaw
        end=np.array([x+value*math.cos(angle),y+value*math.sin(angle)])
        for a,b,_ in DepthVolume._ray(None,(*((np.array([x,y])-origin)),0.),(*((end-origin)),0.),resolution_m=resolution):
            if 0<=a<shape[1] and 0<=b<shape[0]:result[b,a]+=1
        a,b=np.floor((end-origin)/resolution).astype(int)
        if 0<=a<shape[1] and 0<=b<shape[0]:hits[b,a]=True
    return (result>=2)&~hits


def transformed_voxels(voxels,matrix,resolution):
    tf=np.asarray(matrix,float);rigid(tf)
    corners=np.array(list(itertools.product((0.,1.),repeat=3)))
    cells=set()
    for cell in voxels:
        points=(np.asarray(cell)+corners)*resolution
        points=points@tf[:3,:3].T+tf[:3,3]
        lo=np.floor(np.round(points.min(0)/resolution,9)).astype(int)
        hi=np.ceil(np.round(points.max(0)/resolution,9)).astype(int)
        cells.update(itertools.product(*(range(a,b) for a,b in zip(lo,hi))))
    return cells


def cached_voxel_centres(cells,kernel,shape,origin,resolution):
    """Apply the original height/yaw kernel without rebuilding its geometry."""
    anchors=np.floor(((np.asarray(cells)+.5)*resolution-origin)/resolution).astype(int)
    yy,xx=np.nonzero(kernel);cy,cx=np.array(kernel.shape)//2
    offsets=np.column_stack((xx-cx,yy-cy))
    result=np.zeros(shape,bool)
    # Bound temporary storage even for a large mapped furniture surface.
    for start in range(0,len(anchors),256):
        target=(anchors[start:start+256,None,:]+offsets[None,:,:]).reshape(-1,2)
        target=target[(target[:,0]>=0)&(target[:,0]<shape[1])&(target[:,1]>=0)&(target[:,1]<shape[0])]
        result[target[:,1],target[:,0]]=True
    return result


class FurnitureNavigationLayer:
    def __init__(self,folder,store):
        from scipy.spatial import ConvexHull
        from tools.diagnostics.review_s4_sensor_capture import endpoint_obstacles
        folder=Path(folder);self.config=json.loads((folder/'binding.json').read_text())
        for name,key in [('baseline.npz','baseline_sha256'),('occupancy.json','map_sha256')]:
            if hashlib.sha256((folder/name).read_bytes()).hexdigest()!=self.config[key]:raise ValueError('navigation evidence hash mismatch')
        self.binding=hashlib.sha256(document(self.config).encode()).hexdigest();self.store=store
        self.base=json.loads((folder/'occupancy.json').read_text());self.source=dict(np.load(folder/'baseline.npz'))
        self.grid=np.asarray(self.base['data']).reshape(self.base['height'],self.base['width'])
        self.origin=np.asarray(self.base['origin']);self.resolution=self.base['resolution'];self.size=self.config['voxel_size_m']
        if abs(self.resolution-self.size)>1e-9:raise ValueError('matching evidence lattices required')
        self.kernels=None
        cache=folder/'geometry-cache.json'
        if cache.exists():
            manifest=json.loads(cache.read_text());archive=folder/'geometry-cache.npz'
            if (manifest['binding_sha256']!=hashlib.sha256((folder/'binding.json').read_bytes()).hexdigest() or
                manifest['cache_sha256']!=hashlib.sha256(archive.read_bytes()).hexdigest()):
                raise ValueError('navigation geometry cache binding mismatch')
            self.kernels=dict(np.load(archive))
        points=np.asarray(self.config['reference_geometry']);keys=set(map(tuple,np.floor(points/self.size+1e-8).astype(int)))
        self.owned={tuple(map(int,v)) for v in self.source['voxels'] if tuple(v) in keys}
        if not self.owned:raise ValueError('registered furniture has no owned baseline voxels')
        # Semantic labels can omit the edge of a measured surface. A separately
        # bound replay of the ORIGINAL RGB-D exposures completes that ownership;
        # proximity to the robot or today's missing depth never does.
        ownership_path=folder/'ownership.json';ownership_revision=None
        if ownership_path.exists():
            ownership=json.loads(ownership_path.read_text())
            if (ownership['binding_sha256']!=hashlib.sha256((folder/'binding.json').read_bytes()).hexdigest() or
                    ownership['reference_revision']!=self.config['reference_revision'] or
                    ownership['furniture_id']!=self.config['furniture_id'] or
                    ownership['method']!='original_rgbd_surface_membership'):
                raise ValueError('furniture ownership evidence binding mismatch')
            members={tuple(v['voxel']) for v in ownership['members']}
            baseline=set(map(tuple,self.source['voxels']))
            sources={v['view'] for v in ownership['sources']}
            if (not members<=baseline or any(not v['views'] or not set(v['views'])<=sources
                    for v in ownership['members'])):
                raise ValueError('furniture ownership requires original observed baseline cells')
            self.owned|=members
            ownership_revision=hashlib.sha256(ownership_path.read_bytes()).hexdigest()
            points=np.vstack((points,(np.asarray(sorted(members)).reshape(-1,3)+.5)*self.size))
        self.unowned=np.asarray([v for v in self.source['voxels'] if tuple(v) not in self.owned],int).reshape(-1,3)
        self.other_depth,_=endpoint_obstacles(self.unowned,self.size,self.grid.shape,self.origin,self.resolution,
            bottom=.1,top=self.config['body']['height_above_floor_m']+self.config['body']['clearance_m'])
        yy,xx=np.indices(self.grid.shape)
        centers=np.stack((self.origin[0]+(xx+.5)*self.resolution,self.origin[1]+(yy+.5)*self.resolution),axis=-1)
        inside=furniture_laser_membership(points,centers,self.size,self.resolution)
        self.lidar_owned={tuple(map(int,v)) for v in np.argwhere(inside&(self.grid>=65))[:,::-1]}
        previous=store.record('furniture_navigation',self.config['furniture_id'])
        if previous and previous['binding']!=self.binding:raise ValueError('navigation layer binding changed')
        if previous and previous.get('ownership_revision') and ownership_revision is None:
            raise ValueError('persisted furniture ownership evidence missing')
        self.state=previous or dict(binding=self.binding,history_voxels=sorted(map(list,self.owned)),
            retired_voxels=[],retired_lidar_cells=[],active_matrix=None,revision='original',evidence=[])
        remembered=store.record('furniture_last_location',self.config['furniture_id'])
        if not previous and remembered:
            if (remembered.get('identity_verified') is not True or
                    remembered.get('reference_revision')!=self.config['reference_revision']):
                raise ValueError('remembered furniture belongs to another reference')
            self.state['active_matrix']=remembered['matrix']
            self.state['history_voxels']=sorted(map(list,self.owned|transformed_voxels(self.owned,remembered['matrix'],self.size)))
            self.state['revision']='fnav-'+hashlib.sha256(document(self.state).encode()).hexdigest()[:24]
            self.store.save_record('furniture_navigation',self.config['furniture_id'],self.state)
        if ownership_revision and self.state.get('ownership_revision')!=ownership_revision:
            history=set(map(tuple,self.state['history_voxels']))|self.owned
            if self.state['active_matrix'] is not None:
                history|=transformed_voxels(self.owned,self.state['active_matrix'],self.size)
            self.state.update(history_voxels=sorted(map(list,history)),ownership_revision=ownership_revision)
            self.state['revision']='fnav-'+hashlib.sha256(document({k:v for k,v in self.state.items() if k!='revision'}).encode()).hexdigest()[:24]
            self.store.save_record('furniture_navigation',self.config['furniture_id'],self.state)
        if (remembered and confirmed_geometry_replacement(remembered,self.config) and
                self.state['active_matrix'] is not None and
                np.allclose(remembered['matrix'],self.state['active_matrix'],atol=.05,rtol=0)):
            current=transformed_voxels(self.owned,self.state['active_matrix'],self.size)
            retired=set(map(tuple,self.state['history_voxels']))-current
            retired_lidar=set(map(tuple,self.state['retired_lidar_cells']))|self.lidar_owned
            if (retired!=set(map(tuple,self.state['retired_voxels'])) or retired_lidar!=set(map(tuple,self.state['retired_lidar_cells'])) or
                    self.state.get('geometry_policy')!='replace_confirmed_identity'):
                self.state.update(retired_voxels=sorted(map(list,retired)),retired_lidar_cells=sorted(map(list,retired_lidar)),geometry_policy='replace_confirmed_identity')
                self.state['evidence']=(self.state['evidence']+[dict(kind='confirmed_identity_replacement',capture_id=remembered['capture_id'],evidence_capture_ids=remembered['evidence_capture_ids'])])[-64:]
                self.state['revision']='fnav-'+hashlib.sha256(document({k:v for k,v in self.state.items() if k!='revision'}).encode()).hexdigest()[:24]
                self.store.save_record('furniture_navigation',self.config['furniture_id'],self.state)
        self.votes={};self.last_stamp=None;self.last_capture=None;self.last_bundle=None

    def observe(self,*,saved_location,native,metadata,camera_to_map,scan,lidar_xy_yaw,
                capture_id,observed_s,now_s):
        from tools.diagnostics.review_s4_sensor_capture import endpoint_obstacles
        if (not math.isfinite(observed_s) or not 0<=now_s-observed_s<=5 or
            self.last_stamp is not None and observed_s<=self.last_stamp or capture_id==self.last_capture):
            raise ValueError('fresh unique map observation required')
        if (saved_location.get('identity_verified') is not True or
            saved_location.get('reference_revision')!=self.config['reference_revision'] or
            saved_location.get('furniture_id')!=self.config['furniture_id'] or
            saved_location.get('capture_id')!=capture_id or saved_location.get('observed_s')!=observed_s):
            raise ValueError('current registered furniture association required')
        depth=register_native_depth(native,metadata)
        state=json.loads(json.dumps(self.state));matrix=np.asarray(saved_location['matrix']);rigid(matrix)
        active=state['active_matrix']
        if active is None or np.linalg.norm(matrix[:2,3]-np.asarray(active)[:2,3])>.05 or abs(math.atan2(matrix[1,0],matrix[0,0])-math.atan2(active[1][0],active[0][0]))>.06:
            active=matrix.tolist();state['active_matrix']=active
        active_cells=transformed_voxels(self.owned,active,self.size)
        history=set(map(tuple,state['history_voxels']))|active_cells
        retired=set(map(tuple,state['retired_voxels']))-active_cells
        if confirmed_geometry_replacement(saved_location,self.config):
            # One identified object has one current shape. Do not leave phantom
            # copies just because its previous surface is outside today's view.
            retired=history-active_cells
            state['geometry_policy']='replace_confirmed_identity'
        candidates=sorted(history-active_cells-retired)
        boxes=[(np.array(v)*self.size,(np.array(v)+1)*self.size) for v in candidates]
        free=visible_free_boxes(boxes,depth,metadata['color_intrinsics'],camera_to_map)
        votes={}
        def confirmed(key):
            previous=self.votes.get(key)
            first=previous[0] if previous and observed_s-previous[1]<=1.5 else observed_s
            votes[key]=(first,observed_s)
            return observed_s-first>=.25
        for v,clear in zip(candidates,free):
            if clear and confirmed(('voxel',*v)):retired.add(v)
        retired_lidar=set(map(tuple,state['retired_lidar_cells']))
        if confirmed_geometry_replacement(saved_location,self.config):
            # Retire the object's registered original laser contribution too.
            # The current transformed shape and independent live sensor layers
            # are applied after this subtraction, so overlap is not cleared.
            retired_lidar|=self.lidar_owned
        # A laser cell is at laser height: repeated synchronized rays through
        # it prove it empty without also demanding the high camera see the floor.
        # Only cells inside the registered old furniture are eligible; other
        # lidar cells and all unowned upper depth geometry remain untouched.
        if scan is not None and abs(scan['observed_s']-observed_s)<=.2:
            free_laser=laser_free_cells(scan,lidar_xy_yaw,self.grid.shape,self.origin,self.resolution)
            active_projection,_=endpoint_obstacles(np.asarray(sorted(active_cells)),self.size,self.grid.shape,
                self.origin,self.resolution,bottom=.1,top=1.2)
            for x,y in sorted(self.lidar_owned-retired_lidar):
                if free_laser[y,x] and not active_projection[y,x] and confirmed(('laser',x,y)):
                    retired_lidar.add((x,y))
        state.update(history_voxels=sorted(map(list,history)),retired_voxels=sorted(map(list,retired)),
            retired_lidar_cells=sorted(map(list,retired_lidar)))
        changed=any(state[k]!=self.state[k] for k in ('active_matrix','history_voxels','retired_voxels','retired_lidar_cells'))
        if changed:
            state['evidence']=(state['evidence']+[dict(capture_id=capture_id,observed_s=observed_s,
                previous_revision=self.state['revision'],retired_voxels=len(retired),retired_lidar_cells=len(retired_lidar))])[-64:]
            state['revision']='fnav-'+hashlib.sha256(document({k:v for k,v in state.items() if k!='revision'}).encode()).hexdigest()[:24]
            self.store.save_record('furniture_navigation',self.config['furniture_id'],state)
            self.state=state
        self.votes=votes;self.last_stamp=observed_s;self.last_capture=capture_id
        return changed

    def compile(self):
        """Recompute only affected centres using the original STL slab algorithm."""
        from scipy.ndimage import binary_dilation
        from tools.diagnostics.review_s4_sensor_capture import endpoint_obstacles,footprint_kernel,voxel_body_centres
        state=self.state;grid=self.grid.copy();body=self.config['body'];clearance=body['clearance_m']
        for x,y in state['retired_lidar_cells']:grid[y,x]=0
        remaining=set(map(tuple,state['history_voxels']))-set(map(tuple,state['retired_voxels']))
        own=np.asarray(sorted(remaining),int).reshape(-1,3)
        old_new=np.asarray(sorted(self.owned|set(map(tuple,state['history_voxels']))),int)
        changed_mask,_=endpoint_obstacles(old_new,self.size,grid.shape,self.origin,self.resolution,bottom=0,top=2)
        for x,y in state['retired_lidar_cells']:changed_mask[y,x]=True
        radius=float(np.linalg.norm(np.asarray(body['footprint_xy_m']),axis=1).max())+clearance+.1
        steps=int(math.ceil(radius/self.resolution));affected=binary_dilation(changed_mask,iterations=steps)
        all_voxels=np.vstack((self.unowned,own))
        # Only nearby voxels can influence the affected region; exact overlap
        # calculation still uses the original voxel lattice and height slices.
        y,x=np.nonzero(affected);lo=self.origin+np.array([x.min(),y.min()])*self.resolution-radius
        hi=self.origin+np.array([x.max()+1,y.max()+1])*self.resolution+radius
        xyz=(all_voxels+.5)*self.size
        near=all_voxels[np.all((xyz[:,:2]>=lo)&(xyz[:,:2]<=hi),axis=1)]
        feasible=self.source['feasible_by_yaw'].copy()
        for i in range(len(feasible)):
            yaw=i*2*math.pi/len(feasible)
            kernel=(self.kernels['base_'+str(i)] if self.kernels is not None else
                    footprint_kernel([body['footprint_xy_m']],self.resolution,yaw,clearance))
            blocked=binary_dilation(grid>=65,structure=kernel)
            unknown=binary_dilation(grid<0,structure=kernel,border_value=1)
            for z in np.unique(near[:,2]):
                polygons=self.config['slabs'].get(str(z),[])
                if not polygons:continue
                cells=near[near[:,2]==z,:2]
                if self.kernels is not None:
                    blocked|=cached_voxel_centres(cells,self.kernels[f'slab_{i}_{z}'],grid.shape,self.origin,self.resolution)
                else:
                    blocked|=voxel_body_centres(cells,polygons,voxel_size=self.size,
                        grid_shape=grid.shape,origin=self.origin,resolution=self.resolution,yaw=yaw,clearance=clearance)
            valid=(grid==0)&~blocked&~unknown
            feasible[i,affected]=valid[affected]
        rgbd,_=endpoint_obstacles(own,self.size,grid.shape,self.origin,self.resolution,
            bottom=.1,top=body['height_above_floor_m']+clearance)
        rgbd|=self.other_depth
        area={k:v.copy() for k,v in self.source.items() if k!='voxels'}
        area.update(feasible_by_yaw=feasible,rgbd_obstacles=rgbd,
            candidate_centers=feasible.any(0)&~area['uncertain_centers'],
            review_free_by_yaw=feasible&~area['uncertain_centers'][None,:,:])
        doc=dict(self.base,data=grid.ravel().tolist())
        raw=(json.dumps(doc,indent=2)+'\n').encode()
        area['source_map_sha256']=np.array(hashlib.sha256(raw).hexdigest())
        fused=grid.copy();fused[rgbd]=100
        self.last_bundle=dict(revision=state['revision'],map=doc,map_bytes=raw,area=area,fused=fused,
            changed_map_cells=int(np.count_nonzero(fused!=np.where(self.source['rgbd_obstacles'],100,self.grid))),
            cleared_depth_cells=int(np.count_nonzero(self.source['rgbd_obstacles']&~rgbd)),
            added_depth_cells=int(np.count_nonzero(~self.source['rgbd_obstacles']&rgbd)),
            retired_lidar_cells=len(state['retired_lidar_cells']),
            unknown_old_voxels=len(remaining-active_set(self.owned,state['active_matrix'],self.size)))
        return self.last_bundle


def active_set(owned,matrix,size):
    return set() if matrix is None else transformed_voxels(owned,matrix,size)
