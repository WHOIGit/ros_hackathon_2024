# Challenge 2: QR Code Detection

## Objective
Building upon the lawnmower pattern implemented in Challenge 1, create a new node for detecting a QR code on the seafloor, printing its message to the console, and stopping the AUV at the location where the QR code is found.

## Background
In underwater exploration and research, visual markers like QR codes can be used for various purposes, such as identifying specific locations or conveying information. This challenge simulates a scenario where the AUV needs to locate and read a QR code placed on the seafloor.

## Task Description
Your task is to implement a QR code detection system within the Orca4 project. This system should work in conjunction with the lawnmower pattern search implemented in Challenge 1.

## Requirements
1. Create a ROS2 node for QR code detection.
2. The node should subscribe to the camera feed from one of the AUV's downward-facing camera.
3. When a QR code is detected:
   a. Print the decoded message to the console.
   b. Stop the AUV.

## Suggested Approach
1. Start by examining the `orca_extend` package, which provides an example Python ROS2 package as a starting point.

2. Modify the existing node or create a new one within `orca_extend` for QR code detection.

3. Subscribe to the camera topic. You can find the correct topic name by examining the `sim_launch.py` file or by using the `ros2 topic list` command when the simulation is running.

4. Implement the QR code detection logic using OpenCV and the `pyzbar` library, which are already installed and available.

5. When a QR code is detected, publish a message to a new topic indicating that a QR code has been found. Include relevant information such as the decoded message and possibly the AUV's current position.

6. Modify the `mission_runner.py` file to subscribe to the QR code detection topic and handle the signal when a QR code is found (e.g., stopping the AUV and ending the mission).

7. Update the launch files to include your QR code detection node if you created a new one.

## Files to Modify/Create
- Modify or create new files in the `orca_extend` package for QR code detection
- Modify: `orca_bringup/scripts/mission_runner.py` (to handle QR code detection signals)
- Modify: `orca_bringup/launch/sim_launch.py` (to launch the new node if necessary)
- Possibly create a new message definition in `orca_msgs` if needed

## Testing
1. A QR code texture has been placed on the seafloor in the Gazebo simulation.
2. Run the simulation with your lawnmower pattern and QR code detection active.
3. Verify that the AUV successfully detects the QR code, prints the message, and stops at the correct location.

## Hints
- Use the `cv_bridge` package to convert between ROS image messages and OpenCV images.
- Remember that `pyzbar` and `opencv-python` are already installed and available for import.

## Additional Challenges
If you complete the basic QR code detection, consider implementing one or more of the following enhancements:

1. Implement unit tests. See `orca_extend/tests/test_example_node.py` for an example set of unit tests.
   
   a. You can run unit tests with `colcon test --packages-select orca_extend`
   b. View results with `colcon test-result --verbose`.

2. Add the ability to detect multiple QR codes and store their locations.
3. Implement a "return to QR code" function that allows the AUV to navigate back to a detected QR code's location.


## Submission
Once you've completed your implementation, please submit your solution by creating a new branch in the repository named `<team name>/qr_code_detector`. This should include all changes from both Challenge 1 (lawnmower pattern) and Challenge 2 (QR code detection), as well as any unit tests you've created.

## Reference
A reference implementation for this challenge can be found in the `qr_code_detector` branch. You're encouraged to try implementing the solution yourself first, but you can refer to this branch if you get stuck or want to compare your solution.

Good luck with your implementation!