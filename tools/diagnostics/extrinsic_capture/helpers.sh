# Calibration capture helpers. On the Jetson, inside the session folder that holds these files:
#   REL=/home/hyper/robot/releases/<release confirmed with the controller owner> source ~/cal_YYYYMMDD/helpers.sh
# CAL = the folder this file is in (one folder per visit). REL must be given explicitly.
CAL="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export CAL
PY=/home/hyper/robot/shared/venvs/room-scan-bag/bin/python
if [ -z "$REL" ] || [ ! -f "$REL/tools/run/stationary_pick.py" ]; then
  echo "set REL to the installed release folder first, e.g.  REL=/home/hyper/robot/releases/<name> source $CAL/helpers.sh"
  return 1
fi
cd "$REL" || return
echo "session folder: $CAL"
echo "release:        $REL"

# camshot NAME : camera only (floor, table, checkerboard, depth targets, lift-plate marker)
camshot() {
  if [ $# -ne 1 ]; then echo "usage: camshot NAME   e.g. camshot CB-2"; return 2; fi
  "$PY" $CAL/cam_shot.py $CAL/"$1"/cam
}

# pose : controller firmware + which joints are inside the allowed range (read-only)
pose() { "$PY" $CAL/pose_check.py; }

# rawshot NAME LIFT_M [left|right ATTACHMENT] : photo + raw joints (read-only, torque must be off)
#   arm marker:   rawshot M-L-01 0.35 left M1      gripper tip:  rawshot TCP-L-A01 0.00
rawshot() {
  if [ $# -ne 2 ] && [ $# -ne 4 ]; then echo "usage: rawshot NAME LIFT_M [left|right ATTACHMENT]"; return 2; fi
  "$PY" $CAL/raw_shot.py $CAL/"$1" "${@:2}"
}

# livecam : browser preview + type shot names in this terminal (q to quit); port LIVECAM_PORT (8766)
livecam() { "$PY" $CAL/live_cam.py; }

echo "ready: pose | camshot NAME | rawshot NAME LIFT_M [left|right ATT] | livecam"
echo "S3 or S4 controller; depth exposure is fixed at 16 ms in every shot (capture_common.py)"

# sweep / tour need the S3 stationary firmware (legacy AP hold, stationary_platform).
# The S4 unified controller rejects them; they stay here only for an S3 session.
sweep() {
  if [ $# -lt 3 ]; then echo "usage: sweep NAME left|right LIFT_M [--camera --attachment TL1]   (S3 firmware only)"; return 2; fi
  n=$1; a=$2; l=$3; shift 3
  "$PY" $CAL/sweep_log.py $CAL/"$n" --arm "$a" --lift-m "$l" "$@"
}
tour() {
  if [ $# -lt 5 ]; then echo "usage: tour TOUR.json NAME LIFT_M LEFT_ATT RIGHT_ATT [--auto]   (S3 firmware only, MOVES ARMS)"; return 2; fi
  t=$1; n=$2; l=$3; la=$4; ra=$5; shift 5
  "$PY" $CAL/calib_tour.py "$t" $CAL/"$n" --lift-m "$l" --left-attachment "$la" --right-attachment "$ra" "$@"
}
