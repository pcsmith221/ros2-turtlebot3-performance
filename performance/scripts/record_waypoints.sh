#!/bin/bash
#default args are 500 2000 unsecure
#arg $1: min_particle_count
#arg $2: max_particle_count 
#arg $3: security [unsecure,secure]

#enable or disable security settings based on arg 3
if [ $3 = "secure" ]; then 
	source /performance/scripts/sros2.sh 'true'
else
	source /performance/scripts/sros2.sh 'false'
fi 

#function that secures byobu windows if security is enabled
secure_byobu () {
	if [ $1 = "secure" ]; then
		byobu send-keys "source /performance/scripts/sros2.sh true" Enter
	else
		byobu send-keys "source /performance/scripts/sros2.sh false" Enter
	fi
}

#create directories to store runs and rosbag logs if they do not exist
[[ -d /performance/simulations/runs ]] || mkdir /performance/simulations/runs
[[ -d /performance/simulations/rosbag ]] || mkdir /performance/simulations/rosbag

#name directory to store files
RUN_DIR="$3$1-$2"

#check if run directory exists, if it does then append a number to name to avoid duplicates
i=1
while [ -d "/performance/simulations/runs/$RUN_DIR" ]
do
	RUN_DIR="$3$1-$2-$i"
	i=$(( $i + 1 ))	
done

byobu new-session -d -s $USER

# configure particle count parameters 
sed -i "s/min_particles: 500/min_particles: $1/" /ros_ws/install/nav2_bringup/share/nav2_bringup/params/nav2_params.yaml
sed -i "s/max_particles: 2000/max_particles: $2/" /ros_ws/install/nav2_bringup/share/nav2_bringup/params/nav2_params.yaml

# launch turtlebot burger in gazebo world 
byobu rename-window -t $USER:0 'turtlebot'
secure_byobu $3 
byobu send-keys 'ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py' Enter

# launch RViz2 navigation
byobu new-window -t $USER:1 -n 'navigation'
secure_byobu $3
byobu send-keys 'ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/root/map.yaml' Enter

#Wait for simulations to load 
sleep 25
echo "Waiting for simulation ready..."

#send initial pose  
/performance/scripts/initial-pose.sh

#start gazebo logging 
byobu new-window -t $USER:2 -n 'gazebo-log'
secure_byobu $3 
byobu send-keys 'gz log -d 1' Enter

#start rosbag2 logging
byobu new-window -t $USER:3 -n 'rosbag'
secure_byobu $3
byobu send-keys "ros2 bag record /odom /rosout /tf -o /performance/simulations/rosbag/$RUN_DIR" Enter 

#start cpu performance recording
byobu new-window -t $USER:4 -n 'cpu'
secure_byobu $3
byobu send-keys "/performance/scripts/top-cpu-summary.sh 300" Enter

#send waypoints. This will hold up the script until navigation completes or aborts
/performance/scripts/follow-waypoints.sh 

#stop rosbag recording 
byobu select-window -t $USER:3 
byobu send-keys C-c Enter

#stop CPU monitor recording 
byobu select-window -t $USER:4
byobu send-keys C-c Enter

#stop gazebo recording 
byobu select-window -t $USER:2
byobu send-keys 'gz log -d 0' Enter

#kill byobu session (comment out if you want to attach to the session after the script finishes)
byobu kill-session -t root 

#restore nav2_bringup params to defaults 
sed -i "s/min_particles: $1/min_particles: 500/" /ros_ws/install/nav2_bringup/share/nav2_bringup/params/nav2_params.yaml
sed -i "s/max_particles: $2/max_particles: 2000/" /ros_ws/install/nav2_bringup/share/nav2_bringup/params/nav2_params.yaml

#create directory to store log files 
mkdir /performance/simulations/runs/$RUN_DIR

#find and deserialize gazebo recording 
echo "Deseriailizing gazebo recording..."
#gazebo log directory does not have a static name so the log file must be copied using the find command
find /root -name state.log -exec cat > state.log {} \;
gz log -e -f state.log > gazebo_log.txt
mv gazebo_log.txt /performance/simulations/runs/$RUN_DIR
echo "Done!"

#remove serialized gazebo recordings to avoid conflict with next run 
rm -r /root/.gazebo/log/*
rm state.log

#deserialize rosbag recordings 
echo "Deserializing rosbag recording..."
python3 /performance/scripts/nav_bagfile_parser.py /performance/simulations/rosbag/$RUN_DIR/${RUN_DIR}_0.db3 /performance/simulations/runs/$RUN_DIR/
echo "Done!"

#move cpu summary to directory with other logs 
mv /tmp/top-cpu-summary.log /performance/simulations/runs/$RUN_DIR/top-cpu-summary.txt

#log performance data (logDir, outputFile, outputFileHeader)
echo "Logging performance data..."
python3 /performance/scripts/record_performance.py /performance/simulations/runs/$RUN_DIR/ /performance/simulations/RunSummaries.txt $RUN_DIR

echo "Done!"

# Uncomment this to keep simulation running and open byobu terminal(flip between windows with F3 -left and F4 -right)
#byobu attach-session -t $USER
