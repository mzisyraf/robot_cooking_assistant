#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import openai
from openai import OpenAI
from std_srvs.srv import Empty, EmptyResponse

API_KEY = "YOUR-API-KEY" # Insert your OpenAI API key here


class TextToSpeechNode:
    def __init__(self):
        rospy.init_node('text_to_speech_node')
        self.sound_client = SoundClient()
        rospy.sleep(1)  # Wait for sound client to initialize
        rospy.loginfo("Text to Speech Node Initialized")

        self.ingredients_received = False  # To ensure we only process ingredients once

        # Subscribe to the detected ingredients topic
        rospy.Subscriber('/detected_ingredients', String,
                         self.ingredients_callback)

        # Service to start text-to-speech functionality
        rospy.Service('/text_to_speech/start', Empty,
                      self.start_text_to_speech)

    def ingredients_callback(self, data):
        if not self.ingredients_received:
            ingredients = data.data.split(',')
            rospy.loginfo(f"Ingredients received: {ingredients}")
            response = self.get_recipes(ingredients)
            full_response = f"Here are some dishes that can be made with the available ingredients. {response}. Have fun cooking!"
            self.speak(full_response)
            self.ingredients_received = True  # Mark as received to prevent further processing

    def get_recipes(self, ingredients):
        client = OpenAI(
            api_key=API_KEY
        )
        ingredients_str = ", ".join(ingredients)
        prompt = f"{ingredients_str}. From these ingredients, list 5 food names only that can be made."
        completion = client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": "You are a chef."},
                {"role": "user", "content": prompt}
            ]
        )
        return completion.choices[0].message.content

    def speak(self, text):
        self.sound_client.say(text)
        rospy.loginfo(f"Speaking: {text}")

    def start_text_to_speech(self, request):
        rospy.loginfo("Text-to-Speech functionality started")
        return EmptyResponse()


if __name__ == '__main__':
    try:
        node = TextToSpeechNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
