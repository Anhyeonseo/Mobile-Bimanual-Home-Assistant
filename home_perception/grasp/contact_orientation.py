"""Observed straight-across alternatives; original network poses remain valid.

The footprint supplies orientation only, never invisible contact surfaces or a
nominal width. Every new pinch line must be measured again from target points.
"""
import numpy as np


def footprint_short_axis(reference):
    if reference is None:return None
    vertices=reference['vertices']-reference['center']
    following=np.roll(vertices,-1,axis=0)
    area=np.abs(np.cross(vertices,following)@reference['normal'])/2
    # Exact area second moment of the centroid fan; independent of how densely
    # the RGB-D samples one end of the object.
    outer=lambda a,b:a[:,:,None]*b[:,None,:]
    moment=(outer(vertices,vertices)+outer(following,following))/6
    moment+=(outer(vertices,following)+outer(following,vertices))/12
    values,vectors=np.linalg.eigh(np.sum(area[:,None,None]*moment,axis=0)/area.sum())
    if values[1]<=1e-12 or values[2]/values[1]<1.5**2:return None
    axis=np.cross(reference['normal'],vectors[:,2]);axis/=np.linalg.norm(axis)
    return axis


def orientation_variants(row, short_axis):
    yield row,'network_orientation'
    if short_axis is None:return
    r=row[4:13].reshape(3,3)
    closing=np.asarray(short_axis).copy()
    if closing@r[:,1]<0:closing=-closing
    if abs(closing@r[:,1])>np.cos(np.deg2rad(5)):return
    approach=r[:,0]-closing*(r[:,0]@closing)
    if np.linalg.norm(approach)<1e-6:return
    approach/=np.linalg.norm(approach)
    rotation=np.column_stack((approach,closing,np.cross(approach,closing)))
    tip=row[13:16]+row[3]*r[:,0]
    proposal=row.copy();proposal[4:13]=rotation.ravel()
    proposal[13:16]=tip-row[3]*approach
    yield proposal,'observed_straight_across'
