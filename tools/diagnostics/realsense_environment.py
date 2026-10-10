#!/usr/bin/env python3
"""Read-only deployment preflight, also usable before installing Python packages."""

import argparse
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys


def command(args, timeout=30):
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
        return {"returncode": result.returncode, "stdout": result.stdout.strip(), "stderr": result.stderr.strip()}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"error": str(error)}


def read_optional(path):
    try:
        return Path(path).read_text().replace("\x00", "").strip()
    except OSError:
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    disk = shutil.disk_usage(Path.cwd())
    report = {
        "hostname": platform.node(), "architecture": platform.machine(),
        "python": sys.version, "python_executable": sys.executable,
        "kernel": platform.release(), "os_release": read_optional("/etc/os-release"),
        "jetson_model": read_optional("/proc/device-tree/model"),
        "l4t_release": read_optional("/etc/nv_tegra_release"),
        "display": os.environ.get("DISPLAY"), "memory": command(["free", "-m"]),
        "disk_free_gib": round(disk.free / 2**30, 2),
        "jetpack_packages": command(["dpkg-query", "-W", "nvidia-l4t-core", "nvidia-jetpack"]),
        "usb": command(["lsusb"]), "usb_topology": command(["lsusb", "-t"]),
    }
    probes = {
        "vision_packages": '''import json, importlib.metadata as m
versions={}
for name in ['numpy','opencv-python','opencv-python-headless','transformers','pyrealsense2','Pillow']:
 try: versions[name]=m.version(name)
 except m.PackageNotFoundError: versions[name]=None
import cv2
versions['cv2_module']=cv2.__file__
versions['cv2_version']=cv2.__version__
versions['cv2_gui']=[line.strip() for line in cv2.getBuildInformation().splitlines() if 'GUI:' in line]
print(json.dumps(versions))''',
        "torch_gpu": '''import torch,json
r={'torch':torch.__version__,'torch_module':torch.__file__,'cuda_build':torch.version.cuda,'cuda_available':torch.cuda.is_available()}
if r['cuda_available']:
 r['gpu']=torch.cuda.get_device_name(0)
 r['gpu_memory_bytes']=torch.cuda.get_device_properties(0).total_memory
 r['cuda_tensor_check']=(torch.ones(1,device='cuda')+1).item()
print(json.dumps(r))''',
        "torchvision_ops": '''import torch,torchvision,json
from torchvision.ops import nms
boxes=torch.tensor([[0.,0.,2.,2.],[0.,0.,2.,2.]])
print(json.dumps({'torchvision':torchvision.__version__,'nms_kept':nms(boxes,torch.tensor([.9,.8]),.5).tolist()}))''',
        "realsense": '''import pyrealsense2 as rs,json
c=rs.context();result=[]
for d in c.query_devices():
 result.append({label:d.get_info(key) for label,key in [('name',rs.camera_info.name),('serial',rs.camera_info.serial_number),('usb',rs.camera_info.usb_type_descriptor)] if d.supports(key)})
print(json.dumps(result))''',
    }
    report["probes"] = {name: command([sys.executable, "-c", source], timeout=45) for name, source in probes.items()}
    text = json.dumps(report, indent=2) + "\n"
    print(text, end="")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")


if __name__ == "__main__":
    main()
