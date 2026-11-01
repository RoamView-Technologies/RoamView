#!/bin/bash
# Memory of the simulation's big processes every few seconds: Gazebo's server (physics and sensors), the Gazebo
# window, RViz, and how much memory the machine (or WSL) has left. The (+N) is the change since the line before:
# while the world loads it jumps, then it should stay near +0. A renderer that leaks shows +N on every line; on WSL
# that ends with WSL out of memory and every window frozen, so stop the simulation before it gets there.
#
#   ros2 run roamview_sim watch_memory.sh           # every 5 s, warn above 6000 MB
#   ros2 run roamview_sim watch_memory.sh 2 4000    # every 2 s, warn above 4000 MB
#
# Ctrl+C stops the watching; the simulation keeps running.
INTERVAL=${1:-5}
LIMIT_MB=${2:-6000}

sample() {   # prints: server_MB window_MB rviz_MB
  ps -eo rss=,args= | awk '
    { args = substr($0, index($0, $2)) }
    args ~ /awk|watch_memory/ || args ~ /^(\/bin\/)?sh -c/ { next }
    args ~ /gz-sim-gui-client/ || ((args ~ /gz-sim-main/ || args ~ /gz sim/) && args ~ / -g( |$)/) { w += $1; next }
    args ~ /gz-sim-main/ || args ~ /gz sim/ { s += $1; next }
    args ~ /rviz2/ { r += $1 }
    END { printf "%d %d %d\n", s / 1024, w / 1024, r / 1024 }'
}

echo "every ${INTERVAL} s; warning above ${LIMIT_MB} MB per process. Ctrl+C to stop watching."
s0=0; w0=0; r0=0
while true; do
  read -r s w r < <(sample)
  avail=$(awk '/MemAvailable/ { printf "%d", $2 / 1024 }' /proc/meminfo)
  printf '%s  server %5d MB (%+d)  window %5d MB (%+d)  rviz %5d MB (%+d)  | free for use %6d MB\n' \
    "$(date +%T)" "$s" $((s - s0)) "$w" $((w - w0)) "$r" $((r - r0)) "$avail"
  [ $((s + w + r)) -eq 0 ] && echo "          (no Gazebo or RViz running)"
  for pair in "server $s" "window $w" "rviz $r"; do
    set -- $pair
    [ "$2" -gt "$LIMIT_MB" ] && echo "!! the $1 uses $2 MB: if it keeps growing, stop the simulation (Ctrl+C in its terminal)"
  done
  [ "$avail" -lt 1500 ] && echo "!! under 1.5 GB left: stop the simulation now, before the system freezes"
  s0=$s; w0=$w; r0=$r
  sleep "$INTERVAL"
done
