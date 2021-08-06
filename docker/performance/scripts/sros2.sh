#!/bin/sh

export ROS_SECURITY_ENABLE=$1
export ROS_SECURITY_STRATEGY=Enforce
export ROS_SECURITY_KEYSTORE=/performance/keystore
