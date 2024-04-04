#!/bin/bash

# Compute APE for orginal PID Stanley
evo_ape bag ./outputs/pid_stanley_original/traj.bag /odometry/filtered /gazebo/ground_truth/state --plot --plot_mode xy --align 

# Compute APE for improved PID Stanley
evo_ape bag ./outputs/pid_stanley_improved/traj.bag /odometry/filtered /gazebo/ground_truth/state --plot --plot_mode xy --align

# Compute APE for PID Pure Pursuit
evo_ape bag ./outputs/pid_purepursuit/traj.bag /odometry/filtered /gazebo/ground_truth/state --plot --plot_mode xy --align