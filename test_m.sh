#!/bin/bash
( set -m; sleep 100 ) &
PID=$!
sleep 1
ps -o pid,pgid,comm -p $(pgrep -f "sleep 100")
