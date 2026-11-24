"""Pinned inputs used to run the reference RealSense network in the station kit."""
from hashlib import sha256

BASELINE_REVISION = '280c215129f759ed8649cb4e89fc5dfee55f4f80'
CHECKPOINT_SHA256 = '60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868'
BASELINE_FILES = (
    'LICENSE', 'models/backbone.py', 'models/graspnet.py', 'models/loss.py', 'models/modules.py',
    'pointnet2/pointnet2_modules.py', 'pointnet2/pointnet2_utils.py', 'pointnet2/pytorch_utils.py',
    'utils/collision_detector.py', 'utils/label_generation.py', 'utils/loss_utils.py',
)
BASELINE_DIGEST = 'aa9f46b925e755c0ff097b1c9ab0496c41c01f3fd36dd02adeb07b50286791d1'


def verify_baseline(root):
    digest = sha256()
    for name in BASELINE_FILES:
        digest.update(sha256((root / name).read_bytes()).digest())
    if digest.hexdigest() != BASELINE_DIGEST:
        raise ValueError(f'expected inference sources from GraspNet {BASELINE_REVISION} in {root}')
