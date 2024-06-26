# Robot Cooking Assistant
## Introduction
This is a project done for WID3010 Autonomous Robotics session 2023/2024 where we aim to help those struggling with thinking of what to cook with the ingredients at hand using the help of robot vision and speech. This project was completed using Jupiter Robot Juno and the ROS framework. What are the core modules of the project?

- Object Detection: To detect the ingredients available
- OpenAI Inferencing: Use OpenAI API to prompt the list of ingredients to obtain 5 dishes that can be cooked
- Text-to-Speech: Produces a speech output of the dishes that can be cooked


## Manual

1. ### Create a Catkin Workspace and Change Your Directory to Go to the Workspace

   To create a Catkin workspace and navigate to it, run the following commands:
   
   ```bash
   mkdir catkin_ws
   cd catkin_ws/
   ```
   Create a src folder under the Catkin workspace and build the workspace:

   ```bash
   mkdir src
   catkin_make
   ```

2. ### Clone This GitHub Repository

   To clone the GitHub repository, run the following command:

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/mzisyraf/robot_cooking_assistant.git
   ```

3. ### Install Dependencies

   To install dependencies, run the `requirements.txt` file, run the following command in the project root directory:

   ```bash
   cd ~/catkin_ws/src/robot_cooking_assistant
   pip install -r requirements.txt
   ```

4. ### Build the Project Using `catkin_make`

   To build the project using `catkin_make`, run the following command in the `catkin_ws` directory:

   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

5. ### Provide Permissions to All Files

   Provide permission to files to run with ROS using the following commands:
   
   ```bash
   cd ~/catkin_ws/src/robot_cooking_assistant
   chmod +x launch/robotics_project.launch
   chmod +x launch/text_to_speech.launch
   chmod +x scripts/launch_sequence.sh
   chmod +x scripts/ingredients_detection_node.py
   chmod +x scripts/object_detection.py
   chmod +x scripts/text_to_speech.py
   chmod +x scripts/text_to_speech_controller.py
   chmod +x scripts/train_yolov8.py
   ```

7. ### Run the Launch Sequence File

   To run the launch sequence file, first launch `roscore` on a terminal:

   ```bash
   roscore
   ```

   On a separate terminal, run the `launch_sequence.sh` file using the following command:

   ```bash
   cd ~/catkin_ws/src/robot_cooking_assistant/scripts/
   ./launch_sequence.sh
   ```

8. ### Point Ingredients to the Camera

   After pointing all ingredients to the camera, wait until the camera shuts down after 10 seconds of inactivity.

9. ### Robot Processing and Output

   The robot will process the ingredients and output the speech of possible dishes to cook.

## Note
- In the file `/scripts/text_to_speech.py`, replace the string in `API_KEY` with a valid OpenAI API key for the OpenAI Inference module to work properly
