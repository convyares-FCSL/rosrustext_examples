#!/usr/bin/env bash
set -e

echo "========================================"
echo " ROS 2 Graph Diagnostics"
echo "========================================"

echo
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-<default>}"
echo "ROS_DISTRO   : ${ROS_DISTRO:-<unknown>}"

echo
echo "----------------------------------------"
echo " Active ROS Nodes"
echo "----------------------------------------"
ros2 node list || echo "No nodes found"

echo
echo "----------------------------------------"
echo " Node Instance Counts (duplicates)"
echo "----------------------------------------"
ros2 node list | sort | uniq -c | sort -nr

echo
echo "----------------------------------------"
echo " Lesson-related Nodes"
echo "----------------------------------------"
ros2 node list | grep -E "lesson_" || echo "No lesson nodes found"

echo
echo "----------------------------------------"
echo " Topics with Publishers/Subscribers"
echo "----------------------------------------"
for t in $(ros2 topic list); do
    pubs=$(ros2 topic info "$t" 2>/dev/null | grep "Publisher count" | awk '{print $3}')
    subs=$(ros2 topic info "$t" 2>/dev/null | grep "Subscription count" | awk '{print $3}')
    printf "%-40s pubs=%s subs=%s\n" "$t" "${pubs:-0}" "${subs:-0}"
done

echo
echo "----------------------------------------"
echo " Suspicious Conditions"
echo "----------------------------------------"

# Duplicate lesson nodes
DUPES=$(ros2 node list | grep lesson_ | sort | uniq -c | awk '$1 > 1')
if [ -n "$DUPES" ]; then
    echo "⚠ Duplicate node names detected:"
    echo "$DUPES"
else
    echo "✓ No duplicate lesson nodes"
fi

# Zombie-like publishers (topics with subs but no pubs or vice versa)
echo
echo "Potential DDS residue (topics with 0 pubs or 0 subs):"
for t in $(ros2 topic list); do
    pubs=$(ros2 topic info "$t" 2>/dev/null | grep "Publisher count" | awk '{print $3}')
    subs=$(ros2 topic info "$t" 2>/dev/null | grep "Subscription count" | awk '{print $3}')
    if [[ "$pubs" == "0" || "$subs" == "0" ]]; then
        printf "  %s (pubs=%s subs=%s)\n" "$t" "$pubs" "$subs"
    fi
done

echo
echo "----------------------------------------"
echo " OS-level ROS Processes"
echo "----------------------------------------"
ps aux | grep -E "lesson_|ros2 run|rosrustext_lifecycle_proxy|rosbridge" | grep -v grep || echo "No ROS lesson processes running"

echo
echo "========================================"
echo " Diagnostic Complete"
echo "========================================"
