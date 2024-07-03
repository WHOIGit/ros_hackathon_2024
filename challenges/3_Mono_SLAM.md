# Challenge 3: Mono SLAM Implementation

## Objective
Modify the Orca4 project to use a single camera for SLAM (Simultaneous Localization and Mapping) instead of the current stereo camera setup, while maintaining the functionality from the lawnmower pattern (Challenge 1) and QR code detection (Challenge 2).

## Background
SLAM is crucial for autonomous underwater vehicles to navigate and map unknown environments. This challenge simulates a scenario where the AUV needs to operate with a single camera, perhaps due to hardware limitations or to reduce power consumption.

## Requirements
1. Update the ORB-SLAM2 configuration to use mono SLAM mode.
2. Modify the navigation system to work with mono SLAM pose estimates.
3. Ensure the lawnmower pattern search and QR code detection still function correctly.


## Suggested Approach
1. Review the sequence diagram in [orca_base](https://github.com/WHOIGit/orca4/tree/main/orca_base) to better understand SLAM's role.
2. Modify ORB-SLAM2 configuration for mono SLAM mode.
3. Improve pose estimation by integrating depth sensor data, incorporating depth data received in `ardu_pose_cb`.
4. Adjust camera topic subscriptions to use a single camera feed.


## Files to Modify
- ORB-SLAM2 configuration and launch files (`sim_orca_params.yml`, `bringup.py`, `sim_launch.py`).
- Navigation and control system (e.g., `base_controller.cpp`).
- Any files making reference to stereo SLAM topics or nodes.

## Testing
1. Run the simulation with the modified mono SLAM setup.
2. Verify lawnmower pattern navigation and QR code detection.
3. Test pose estimation over longer trajectories.
4. Compare performance with the previous stereo SLAM setup.

## Hints
- Mono SLAM may require more movement for initialization and accurate estimates.

## Additional Challenge
Implement a SLAM system switching mechanism that can alternate between mono and stereo SLAM based on certain conditions (e.g., available computational resources, environment characteristics).

## Submission
Submit your solution by creating a new branch named `<team name>/mono_slam`, including all changes from previous challenges and mono SLAM modifications.

## Reference
A reference implementation is available in the `orb_slam2_mono` branch. Try implementing your solution first before referring to it.

Good luck with your implementation!