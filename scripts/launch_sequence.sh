#!/bin/bash

# Launch the initial set of nodes
roslaunch cooking_assistant robotics_project.launch &

# Wait for the object detection to complete
while ! rostopic echo -n1 /detection_complete | grep -q "Object detection complete"; do
  sleep 1
done

# Launch the text to speech node
roslaunch cooking_assistant text_to_speech.launch
