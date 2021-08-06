#!/bin/bash
#This script was found here: https://askubuntu.com/questions/923922/how-to-save-cpu-logs-or-gpu-usage-values 
# NAME: ~/bin/cpu-top-summary
# DATE: June 13, 2017
# DESC: Call `top` command every second to obtain CPU(s) stats for
#       us, sy, ni, id, wa, hi, si, and st. Log to /tmp/top-cpu-summary.log
#       with time stamp in hh:mm:ss 24 hour format.
# PARM: $1 number of seconds to run, ie 2 hours = 7200

now="$(date +'%d/%m/%Y')"
printf "top CPU(s) summary for %s\n" "$now" > /tmp/top-cpu-summary.log
for ((i=0; i<$1; i++))
do
    TimeStamp=`date +"%H:%M:%S"`
    printf "$TimeStamp - " >> /tmp/top-cpu-summary.log
    top -n 1 -b | head -n 3 | tail -n 1 >> /tmp/top-cpu-summary.log
    sleep 1
done
