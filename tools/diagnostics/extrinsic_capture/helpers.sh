# Calibration capture helpers. On the Jetson:  source ~/cal_20261113/helpers.sh
CAL=~/cal_20261113
PY=/home/hyper/robot/shared/venvs/room-scan-bag/bin/python
cd /home/hyper/robot/releases/s3-stationary-9d2fd662e718 || return

# shot NAME LIFT_M : synchronized photo + joint values (arm marker shots)
shot() {
  if [ $# -ne 2 ]; then echo "usage: shot NAME LIFT_M   e.g. shot L1-01 0.20"; return 2; fi
  f=$CAL/"$1".json
  if [ -f "$f" ] && grep -q '"REJECTED"' "$f"; then rm "$f"; fi
  "$PY" tools/run/stationary_pick.py capture --fixed-lift-m "$2" --output "$f"
}

# camshot NAME : camera only (floor, table, checkerboard)
camshot() {
  if [ $# -ne 1 ]; then echo "usage: camshot NAME   e.g. camshot CB-2"; return 2; fi
  "$PY" $CAL/cam_shot.py $CAL/"$1"/cam
}

# pose : which joints are inside the allowed range (read-only)
pose() { "$PY" $CAL/pose_check.py; }

echo "ready: shot NAME LIFT_M | camshot NAME | pose"

# rawshot NAME LIFT_M : photo + raw joints, no range gate (read-only, torque must be off)
rawshot() {
  if [ $# -ne 2 ]; then echo "usage: rawshot NAME LIFT_M   e.g. rawshot L1-01 0.20"; return 2; fi
  "$PY" $CAL/raw_shot.py $CAL/"$1" "$2"
}
echo "also: rawshot NAME LIFT_M  (use this if shot keeps saying 'outside recorded range')"

# livecam : browser preview + type shot names in this terminal (q to quit)
livecam() { "$PY" $CAL/live_cam.py; }
echo "also: livecam  (live preview; take shots by typing names)"

# sweep NAME ARM LIFT_M [--camera --attachment TL1] : hand-guided free-space log (torque off, read-only)
sweep() {
  if [ $# -lt 3 ]; then echo "usage: sweep NAME left|right LIFT_M [--camera --attachment TL1]"; return 2; fi
  n=$1; a=$2; l=$3; shift 3
  "$PY" $CAL/sweep_log.py $CAL/"$n" --arm "$a" --lift-m "$l" "$@"
}
# tour TOUR.json NAME LIFT_M LEFT_ATT RIGHT_ATT [--auto] : automatic tour for one lift height (MOVES THE ARMS)
tour() {
  if [ $# -lt 5 ]; then echo "usage: tour TOUR.json NAME LIFT_M LEFT_ATT RIGHT_ATT [--auto]"; return 2; fi
  t=$1; n=$2; l=$3; la=$4; ra=$5; shift 5
  "$PY" $CAL/calib_tour.py "$t" $CAL/"$n" --lift-m "$l" --left-attachment "$la" --right-attachment "$ra" "$@"
}
echo "also: sweep NAME ARM LIFT_M  (hand-guided log, torque off) | tour TOUR.json NAME LIFT_M LATT RATT  (MOVES ARMS)"
