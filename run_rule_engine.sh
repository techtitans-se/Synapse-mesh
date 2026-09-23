#!/bin/bash
# Launch AMR Rule Engine with preloaded data
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"

echo "=========================================================="
echo " Starting AMR Warehouse Rule Engine & Dispatch Framework..."
echo " Loaded: inventory_data.json, warehouse_stations.yaml, amr_specs.json"
echo "=========================================================="

ros2 run synapse_amr_warehouse amr_rule_engine.py
