import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def test_floor_picking_rejects_block_surfaces_and_preserves_map_coordinates():
    module = ROOT / "ros2_ws/src/home_robot_tasks/home_robot_tasks/web/map.js"
    script = """const assert=require('node:assert/strict');const fs=require('node:fs');
    const api=require(process.argv[1]);const map=JSON.parse(fs.readFileSync(process.argv[2]));
    const g=api.geometry(map,800,420);
    let checked=0;
    for(const s of g.surfaces){
      if(s.x===undefined)continue;
      const p=g.project(s.x+.5,s.y+.5,s.blocked?.9:0),hit=api.pick(g,...p);
      if(s.blocked)assert.equal(hit,null);
      else if(hit){assert.deepEqual(hit,{x:s.x,y:s.y});checked++;
        const w=api.world(map,hit);assert.equal(w.z,map.floor_z_m);
        assert.equal(w.x,map.origin.x+(s.x+.5)*map.resolution_m);}
    }
    assert(checked>20);assert.equal(api.pick(g,-1,-1),null);
    """
    subprocess.run(
        [
            "node",
            "-e",
            script,
            str(module),
            str(ROOT / "config/navigation_map.simulation.json"),
        ],
        check=True,
    )
