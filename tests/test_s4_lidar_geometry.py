"""Ray checks use surfaces and preserve the blind zone, not skeleton distance."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/diagnostics'))
from s4_lidar_visualize import ray_hits


def test_first_surface_and_parallel_backward_misses():
    far=np.array([[1,-1,-1],[1,1,-1],[1,0,1]],dtype=float)
    near=far.copy();near[:,0]=.08
    hit=ray_hits(np.array([far,near]),np.zeros(3),np.array([[1,0,0],[-1,0,0],[0,1,0]]))
    assert np.isclose(hit[0],.08)  # Do not skip an occluder inside the 12 cm range limit.
    assert hit[1:] == [None,None]
