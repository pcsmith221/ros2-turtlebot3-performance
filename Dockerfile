from therobotcooperative/turtlebot3-ros2:foxy

#copy needed files into container
COPY performance /performance
COPY gazebo /root/.gazebo

#install byobu and python libraries
RUN apt-get update \
 && apt-get install -y idle3 \
 && pip3 install pandas \
 && pip3 install scipy \
 && apt-get install -y python3-matplotlib \
 && apt-get install -y byobu

#generate security artifacts from policy file
RUN . /ros_ws/install/setup.sh && \
    ros2 security generate_artifacts -k /performance/keystore -p /performance/policies/tb3_gazebo_policy.xml

#add environment variables to setup.bash
RUN echo "export TURTLEBOT3_MODEL=\"burger\"" >> /ros_ws/install/setup.bash \
 && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /ros_ws/install/setup.bash

#add executable permission to shell scripts and create simulations directory to store log files for each run
RUN chmod +x /performance/scripts/*.sh \
 && mkdir /performance/simulations 


      

