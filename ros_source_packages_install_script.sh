#!/bin/bash
echo "#################################"
echo "START ROS PACKAGES INSTALLMENT..........."
echo "#################################"
echo "[navigation] installing..."
echo "y" | sudo apt install ros-kinetic-navigation
echo "[navigation] installed!"
sleep 2s
echo "[robot-pose-publisher] installing..."
echo "y" | sudo apt install ros-kinetic-robot-pose-publisher
echo "[robot-pose-publisher] installed!"
sleep 2s
echo "[openslam-gmapping] installing..."
echo "y" | sudo apt install ros-kinetic-openslam-gmapping
echo "[openslam-gmapping] installed!"
sleep 2s
echo "[hector-nav-msgs] installing..."
echo "y" | sudo apt install ros-kinetic-hector-nav-msgs
echo "[hector-nav-msgs] installed!"
sleep 2s
#yocs_velocity_smoother package
echo "[ecl] installing..."
echo "y" | sudo apt install ros-kinetic-ecl
echo "[ecl] installed!"
sleep 2s
#robot_localization package
echo "[geographic-msgs] installing..."
echo "y" | sudo apt install ros-kinetic-geographic-msgs
echo "[geographic-msgs] installed!"
sleep 2s
echo "[tf2-geometry-msgs] installing..."
echo "y" | sudo apt install ros-kinetic-tf2-geometry-msgs
echo "[tf2-geometry-msgs] installed!"
sleep 2s
echo "#################################"
echo "All ROS PACKAGES ARE INSTALLED!"
echo "#################################"
