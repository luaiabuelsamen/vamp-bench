#!/bin/bash
# VAMP pick-place on the A5 cells (CPU, outside GPU slots). The machine is shared: a load log (1-min loadavg and the
# top CPU consumers, every 5 min) is written next to the results so the timings can be read against contention.
cd ~/projects/vamp-bench
LOAD=results/v2/raw/vamp_pickplace.load.log
( while true; do echo "$(date '+%m-%d %T') loadavg $(cut -d' ' -f1-3 /proc/loadavg) memavail_kB $(awk '/MemAvailable/{print $2}' /proc/meminfo)"
    ps -eo pcpu,args --sort=-pcpu | sed -n 2,5p | cut -c1-110 | sed 's/^/  /'; sleep 300; done ) >> $LOAD &
LP=$!; trap "kill $LP" EXIT
.venv/bin/python bench/v2_pickplace_vamp.py --freeze 6aea6eecf76a173d7ebdf548104aaec854b6b6c7 --seeds 5
echo "rc=$? $(date '+%m-%d %T')"
