#### Running ROS across multiple machines ####
#ROS Links
http://wiki.ros.org/ROS/Tutorials/MultipleMachines
http://wiki.ros.org/ROS/NetworkSetup

#ROS on multiple machines (under same local network)
* master setup 
    export ROS_MASTER_URI=http://`[master_ip]`:11311
    export ROS_IP=`[master_ip]`
* slaves setup
    export ROS_MASTER_URI=http://`[master_ip]`:11311
    export ROS_IP=`[slave_ip]`

#The follow packages are required in order to interact with Robot built-in messages and service:
sudo apt-get install:
    * ros-kinetic-navigation
    * ros-kinetic-robot-pose-publisher
    * ros-kinetic-openslam-gmapping
    * ros-kinetic-hector-nav-msgs
    * ros-kinetic-ecl
    * ros-kinetic-geographic-msgs
    * ros-kinetic-tf2-geometry-msgs

Or `sudo sh ros_source_packages_install_script.sh` for the installment



