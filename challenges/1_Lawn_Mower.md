# Challenge 1: Lawnmower Pattern

## Objective
Modify the Orca4 underwater vehicle simulation to implement a lawnmower search pattern instead of the current looping pattern.

## Background
The Orca4 project simulates an autonomous underwater vehicle (AUV) using ROS2 and Gazebo. Currently, the AUV follows a loop pattern. A lawnmower pattern is a more efficient search strategy for covering a rectangular area.

## Task Description
Your task is to modify the Orca4 codebase to implement a lawnmower pattern. This will involve changes to the mission planning and potentially to the navigation system.

## Requirements
1. The AUV should follow a lawnmower pattern, zig-zagging over a rectangular area.
2. The pattern should be configurable, allowing for different:
   - Number of "lanes"
   - Lane spacing
   - Area dimensions
4. The existing functionality for diving to the operating depth should be preserved.

## Suggested Approach
1. Start by examining the `mission_runner.py` file in the `orca_bringup/scripts/` directory. This is likely where you'll need to make the most significant changes.

2. Look at how waypoints are currently defined in the `mission_runner.py` file. You'll need to modify this to generate waypoints for a lawnmower pattern.

3. You may need to create a new function to generate the lawnmower pattern waypoints based on the desired area and lane spacing.

4. Consider adding new parameters to configure the lawnmower pattern, such as area dimensions and lane spacing.

5. Modify the main mission execution logic to use your new lawnmower pattern waypoints.

6. Test your changes in the simulation environment to ensure the AUV follows the new pattern correctly.

## Files to Modify
- `orca_bringup/scripts/mission_runner.py`: This is the main file you'll need to change to implement the new pattern.
- `orca_bringup/launch/sim_launch.py`: You might need to modify this if you're adding new parameters for the lawnmower pattern.
- `orca_nav2/src/straight_line_planner_3d.cpp`: If you need to make any changes to how the path is planned between waypoints.

## Testing
Use the existing simulation environment to test your changes. You should be able to visualize the new pattern in Rviz.

## Hints
- Review the [orca_nav2 sequence diagram](https://github.com/whoigit/orca4/tree/main/orca_nav2) to better understand movement control flow.
- Consider using nested loops to generate the lawnmower pattern waypoints.
- Remember to handle the case where the AUV needs to turn at the end of each "lane".

## Additional Challenge
If you complete the basic lawnmower pattern, consider implementing a simple terrain-following behavior to maintain a constant altitude above the seafloor. This would involve:

1. Using the SLAM data to estimate the seafloor depth along the AUV's path.
2. Adjusting the AUV's depth (Z coordinate) to maintain a constant altitude above the estimated seafloor.
3. Implementing this behavior without changing the horizontal (X and Y) waypoints of the lawnmower pattern.

This will require you to work with the SLAM data and modify how the AUV controls its depth during the mission.

## Reference Solution
A complete solution to this problem (not including additional challenges) can be found in the `lawn_mower` branch of the repository. You're encouraged to try implementing the solution yourself first, but you can refer to this branch if you get stuck or want to compare your solution.

## Submission
Once you've completed your implementation, please submit your solution by creating a new branch in the `whoigit/orca4` repository named `<team name>/lawn_mower`. This will allow for easy review and comparison of different team solutions.

Good luck with your implementation!