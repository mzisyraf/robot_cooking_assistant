#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty

class TextToSpeechController:
    def __init__(self):
        rospy.init_node('text_to_speech_controller')
        self.enable_text_to_speech = rospy.get_param('~enable_text_to_speech', False)
        self.ingredients_received = False  # To ensure we only process ingredients once
        rospy.loginfo(f"Text-to-Speech Controller Node Initialized with TTS enabled: {self.enable_text_to_speech}")

        # Subscribe to detected ingredients topic
        rospy.Subscriber('/detected_ingredients', String, self.detected_ingredients_callback)
        # Subscribe to detection completion topic
        rospy.Subscriber('/detection_complete', String, self.detection_complete_callback)

    def detected_ingredients_callback(self, data):
        if not self.ingredients_received:
            self.ingredients = data.data.split(',')
            self.ingredients_received = True
            rospy.loginfo(f"Ingredients received: {self.ingredients}")

    def detection_complete_callback(self, data):
        if self.enable_text_to_speech and self.ingredients_received:
            rospy.loginfo("Object detection completed, starting Text-to-Speech")
            self.launch_text_to_speech()

    def launch_text_to_speech(self):
        # Start the actual text-to-speech node
        try:
            rospy.wait_for_service('/text_to_speech/start')
            start_text_to_speech = rospy.ServiceProxy('/text_to_speech/start', Empty)
            start_text_to_speech()
            rospy.loginfo("Text-to-Speech node started")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        controller = TextToSpeechController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
