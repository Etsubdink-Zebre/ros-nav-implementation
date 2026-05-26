#!/usr/bin/env bash
# deliver_medicine.sh — Quick-start medicine delivery via Hermes Agent
#
# Usage:
#   ./deliver_medicine.sh [from] [to] [payload] [priority]
#
# Defaults:
#   from=reception  to=patient_room1  payload=medicine  priority=normal

set -euo pipefail

FROM="${1:-reception}"
TO="${2:-patient_room1}"
PAYLOAD="${3:-medicine}"
PRIORITY="${4:-normal}"
ROBOT="aws_robomaker_hospital_world"

echo "╔══════════════════════════════════════════════════╗"
echo "║  Hermes Hospital Delivery Robot                  ║"
echo "╠══════════════════════════════════════════════════╣"
echo "║  From     : $FROM"
echo "║  To       : $TO"
echo "║  Payload  : $PAYLOAD"
echo "║  Priority : $PRIORITY"
echo "║  Robot    : $ROBOT"
echo "╚══════════════════════════════════════════════════╝"
echo ""

ros2 run aws_robomaker_hospital_world hermes_agent.py deliver \
    --robot "$ROBOT" \
    --from "$FROM" \
    --to "$TO" \
    --payload "$PAYLOAD" \
    --priority "$PRIORITY"
