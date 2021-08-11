# ROS 2 Secure TurtleBot3 Performance
This repository investigates the research question: “Does enabling security configurations add overhead to the performance of a TurtleBot?” by straining the performance of a simulated TurtleBot to the point where additional overhead can have a measurable impact on performance. To accomplish this, we simulated the TurtleBot attempting to navigate a set of waypoints with increasing particle counts, which are used to localize the robot’s position. After each simulation, behavioral metrics such as navigation time and robot jitter were recorded from log files. The log files for each run were then parsed and grouped together for analysis/visualization using a Jupyter Notebook. 

## Running the experiments 
Download the repository and build the Dockerfile.

```
cd path/to/ros2-turtlebot3-performance/docker
docker build -t image_name .
```

Run the docker container with port 5900 open 

```
docker run -p 5900:5900 --rm -it image_name
```

Inside the container, start the VNC server and visualize the container with a VNC viewer. If prompted, the password is `password`. 

```
/startup-vnc.sh
```
 
Once on the desktop, you should change the display settings to prevent the desktop from turning off while running simulations by clicking `Applications` in the top left corner, then `Settings -> Power Manager -> Display` and change the `switch off after` slider to 0. 

Once you open Terminal Emulator, all needed scripts are found in the /performance/scripts directory. 

```
cd /performance/scripts
```

To run the simulation a single time with default settings: 

```
./record_waypoints.sh 500 2000 unsecure 
```

Argument 1 is the minimum particle count and argument 2 is the max particle count, they should be positive integers. The third argument should be either "unsecure" or "secure" to disable or enable security configurations. Note that the script currently has no parameter validation. 

To run the simulation multiple times with increasing particle counts, first with no security then with security enabled, use the `run_experiments.sh` script, for example: 

```
./run_experiments.sh 40000 5 
```

This script simply runs the `record_waypoints.sh` script multiple times with different parameters. Argument 1 is the highest max particle count the script will simulate, and argument 2 is the number of times each configuration will be simulated, both unsecure and secure. The default step value for min and max particles in the script is 2000, you must edit the script to change this. Running the script on a 2017 Macbook pro with a 2.2 GHz Core i7 processor with the docker containter allocated 8GB of RAM and 6 CPU cores, a max particle count of 30000 is when the TurtleBot began to perform significantly worse, and at about 40000 the simulation would typically crash seconds after beginning navigation. 

Both the `record_waypoints.sh` and `run_experiments.sh` scripts log performance data to the `/performance/simulations` directory. A summary of performance behavior for each run is appended to `/performance/simulations/RunSummaries.txt`. The log files for each run can be found in `/performance/simulations/runs`, and the Rosbag recordings can be found in `/performance/simulations/rosbag`. 

### Modifying the experiments 
To modify the waypoints the TurtleBot navigates to, change or add new goals to the `follow-waypoints.sh` script. However, if you change the final goal, you must also change the final goal in the Main function of the `record_performance.py` script to calulate the correct final distance from goal after navigation completes.

To change the /topics recorded by Rosbag2, modify line 70 of the `record_waypoints.sh` script. You will also need to modify the `nav_bagfile_parser.py` script to deserialize the recordings; at the bottom simply change or add additional calls to the *LogTopic()* function. 

To change the data appended to *RunSummaries.txt*, you must modify the *LogExperimentData()* function in the `record_performance.py` script. 

## Visualizing the data in Jupyter Notebook 
To visualize key performance metrics using `ViewExperimentDataNotebook.ipynb`. copy the file from the docker container and replace the string in the first cell of the Jupyter Notebook with the path to your *RunSummaries.txt* file. To copy the file from the docker containter to your local machine: 

```
docker cp container_name:/performance/simulations/RunSummaries.txt /path/on/local/machine
```

An example *RunSummaries.txt* file is also included in this repository. Note that if you would like to add more summaries to this file, all unsecure runs should be listed before the secure runs for the notebook to parse the log properly. The notebook does not graph every performance metric, but they can all be accessed in the created DataFrames. 
