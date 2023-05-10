## Medical Snake Main Control Code

To run the medical snake: `roslaunch medical_snake medsnake.launch`

Start the joystick node by: `rosrun joy joy_node`

To run the camera:

Clone the [image_pipeline](https://github.com/ros-perception/image_pipeline.git) and build the workspace.

Launch the camera file: `roslaunch medical_snake medsnake.launch`

The calibration .yaml file for the camera is needed. Follow the [How to Calibrate a Monocular Camera](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration). Copy the .yaml file into the config folder (e.g. 'medsnake_endoscope.yaml') in the usb_cam package. Remember to change the 'camera_info_url' in the 'usb_cam.yml'.