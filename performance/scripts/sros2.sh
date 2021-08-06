#!/bin/sh
#sets security environment variables
#arg1: true enables security, anything else disables security
export ROS_SECURITY_ENABLE=$1
export ROS_SECURITY_STRATEGY=Enforce
export ROS_SECURITY_KEYSTORE=/performance/keystore
