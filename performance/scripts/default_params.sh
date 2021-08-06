#!/bin/bash
#Restores amcl parameters back to default using backup.yaml file and removes gazebo log to ensure next run can find the correct log file
cp backup.yaml /ros_ws/install/nav2_bringup/share/nav2_bringup/params/nav2_params.yaml

rm -r /root/.gazebo/log/*
