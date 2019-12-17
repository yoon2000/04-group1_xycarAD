import rospy
from std_msgs.msg import Int32MultiArray

class ObstacleDetector:

    def __init__(self, topic):
        self.left = []
        self.mid = -1
        self.right = []
        self.back = -1
        rospy.Subscriber(topic, Int32MultiArray, self.read_distance)

    def read_distance(self, data):
        self.left.extend([data.data[0], data.data[7], data.data[3]])
        self.mid = data.data[1]
        self.back = data.data[4]
        self.right.extend([data.data[2], data.data[6], data.data[5]])

    def get_distance(self):
        return self.left, self.mid, self.right, self.back
