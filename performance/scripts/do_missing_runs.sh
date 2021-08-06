#!/bin/bash
#ensure computer doesn't overload doing too many runs too quickly 
echo "Waiting to start..."
sleep 180
timeout 3m bash /performance/scripts/record_waypoints.sh 6500 8000 unsecure

echo "Waiting to start..."
sleep 180
timeout 3m bash /performance/scripts/record_waypoints.sh 6500 8000 unsecure

echo "Waiting to start..."
sleep 180
timeout 3m bash /performance/scripts/record_waypoints.sh 8500 10000 unsecure

echo "Waiting to start..."
sleep 180
timeout 3m bash /performance/scripts/record_waypoints.sh 14500 16000 unsecure

echo "Waiting to start..."
sleep 180
timeout 3m bash /performance/scripts/record_waypoints.sh 16500 18000 unsecure

echo "Waiting to start..."
sleep 180
timeout 3m bash /performance/scripts/record_waypoints.sh 24500 26000 unsecure

echo "Waiting to start..."
sleep 180
timeout 3m bash /performance/scripts/record_waypoints.sh 500 2000 secure

echo "Waiting to start..."
sleep 180
timeout 3m bash /performance/scripts/record_waypoints.sh 4500 6000 secure

echo "Waiting to start..."
sleep 180
timeout 3m bash /performance/scripts/record_waypoints.sh 12500 14000 secure

echo "Waiting to start..."
sleep 180
timeout 3m bash /performance/scripts/record_waypoints.sh 22500 24000 secure
