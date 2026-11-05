#!/bin/sh
# Install (or with --remove, uninstall) the high-five server as a login service. Log: ~/Library/Logs/highfive.log
set -e
DST=~/Library/LaunchAgents/com.highfive.robot.plist
launchctl bootout gui/$(id -u)/com.highfive.robot 2>/dev/null || true
if [ "$1" = "--remove" ]; then
  rm -f "$DST"; echo "high-five service removed"; exit 0
fi
pkill -f "tools/highfive.py" 2>/dev/null || true      # a copy started by hand would hold port 8091
REPO=$(cd "$(dirname "$0")/../.." && pwd)
sed -e "s|__REPO__|$REPO|g" -e "s|__HOME__|$HOME|g" "$(dirname "$0")/com.highfive.robot.plist.template" > "$DST"
launchctl bootstrap gui/$(id -u) "$DST"
echo "high-five service installed: http://localhost:8091, log ~/Library/Logs/highfive.log"
