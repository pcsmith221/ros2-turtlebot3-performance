#!/bin/bash
#run record_waypoints many times with varied particle count 
#$1 is upper bound for particle count loop 
#$2 is number of experiments to run for each configuration
min_particles=500
max_particles=2000
i=1

#run unsecure
while [ $max_particles -le $1 ]
do
	while [ $i -le $2 ]
	do 
		#ensure computer doesn't overload doing too many runs too quickly 
		echo "Waiting to start..."
		sleep 180
		timeout 3m bash /performance/scripts/record_waypoints.sh $min_particles $max_particles unsecure

		#increment run counter 
		i=$(( $i + 1 ))
	        
		#double checks previous session is cleared just in case the script timed out
		tmux kill-session -t root 
		/performance/scripts/default_params.sh 	
	done

	i=1
	min_particles=$(( $min_particles + 2000 ))
	max_particles=$(( $max_particles + 2000 ))
done

min_particles=500
max_particles=2000
i=1

#run secure
while [ $max_particles -le $1 ]
do
        while [ $i -le $2 ]
        do
                #ensure computer doesn't overload doing too many runs too quickly 
                echo "Waiting to start..."
                sleep 180
                timeout 3m bash /performance/scripts/record_waypoints.sh $min_particles $max_particles secure

                #increment run counter 
                i=$(( $i + 1 ))

                #double checks previous session is cleared just in case the script timed out
                tmux kill-session -t root
                /performance/scripts/default_params.sh
        done

        i=1
        min_particles=$(( $min_particles + 2000 ))
        max_particles=$(( $max_particles + 2000 ))
done
