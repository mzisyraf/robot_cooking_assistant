#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class IngredientDetectionNode:
    def __init__(self):
        rospy.init_node('ingredient_detection_node')
        self.detected_items = []
        self.subscriber = rospy.Subscriber('/detected_objects', String, self.callback)
        self.publisher = rospy.Publisher('/detected_ingredients', String, queue_size=10)
        rospy.loginfo("Ingredient Detection Node Initialized")

    def callback(self, data):
        detected_objects_str = data.data
        self.detected_items = detected_objects_str.split(',')
        rospy.loginfo(f"Detected objects: {self.detected_items}")

    def publish_ingredients(self):
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            ingredients_str = ','.join(self.detected_items)
            self.publisher.publish(ingredients_str)
            rospy.loginfo(f"Published ingredients: {ingredients_str}")
            rate.sleep()

if __name__ == '__main__':
    try:
        node = IngredientDetectionNode()
        node.publish_ingredients()
    except rospy.ROSInterruptException:
        pass
