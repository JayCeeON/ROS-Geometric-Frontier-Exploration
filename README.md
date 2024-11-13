# Frontier Exploration with ROS1 and Gazebo

This repository contains a project for autonomous frontier exploration using ROS1 and Gazebo. The system allows a mobile robot to explore unknown environments by identifying frontier cells and navigating to them, expanding its knowledge of the map.

## Overview

The exploration algorithm is designed to:
1. **Identify Frontier Cells**: Unmapped cells adjacent to mapped areas are marked as frontiers.
2. **Group Frontier Cells**: Cells are clustered into frontier lines to form exploration targets.
3. **Navigate to Frontier Goals**: The robot selects and navigates to the centroid of the largest frontier group.
4. **Evaluate Exploration Progress**: The percentage of newly mapped areas is calculated to assess exploration completion.

