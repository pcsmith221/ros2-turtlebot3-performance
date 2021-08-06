To run unsecure experiments: /performance/scripts/run_experiments.sh max_particles_to_step_to n 
This will run the navigation simulation starting from default parameters of min: 500, max: 2000, up to max_particles, repeating each configuration n times. By default the particle counts increment by 2000. 

The same logic applies to running navigation simulations with security enabled: /performance/scripts/run_secure_experiments.sh max_particles n

To run the simulation a single time: /performance/scripts/record_waypoints.sh min_particles max_particles secure||unsecure 

To change the waypoints, edit the goal poses in /performance/scripts/follow-waypoints.sh

The rest of the scripts are executed within record_waypoints.sh to record and log experimental data. 