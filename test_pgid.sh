#!/bin/bash
setsid sleep 100 &
PID=$!
sleep 0.5
PGID=$(ps -o pgid= -p $PID | tr -d ' ')
echo "Child PID: $PID, PGID: $PGID"
ps -o pid,pgid,comm -g $PGID
