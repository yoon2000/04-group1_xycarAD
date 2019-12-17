#!/usr/bin/env python

import rospy, time

from linedetector import LineDetector
from obstacledetector import ObstacleDetector
from motordriver import MotorDriver


class AutoDrive:

    def __init__(self, otherspeed=30):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.obstacle_detector = ObstacleDetector('/ultrasonic')
        self.driver = MotorDriver('/xycar_motor_msg')
        self.otherspeed = otherspeed

    def trace(self):
        obs_l, obs_m, obs_r, obs_back = self.obstacle_detector.get_distance()
        line_l, line_r, l_dot, r_dot = self.line_detector.detect_lines()
        # self.line_detector.show_images(line_l, line_r)
        angle = self.steer(line_l, line_r, l_dot, r_dot, obs_l, obs_r)
        speed = self.accelerate(obs_m, angle, self.otherspeed, obs_back)
        self.driver.drive(angle + 90, speed + 90)

    def steer(self, left, right, l_dot, r_dot, obs_l, obs_r):

        if left == -40 or right == 680:
            mid = (left + right) // 2
            angle = (mid - 320) // 1.8
        else:
            angle = 0

        if min(obs_l) > 40 and l_dot == True:
            angle = -25
        elif min(obs_r) > 40 and r_dot == True:
            angle = 25
        print("angle:", angle)
        return angle

    def accelerate(self, obs_m, otherspeed, obs_back):
        if obs_m < 50:
            speed = 0
        elif obs_back < 60:
            speed = otherspeed
        else:
            speed = 20

        print("speed : ", speed)
        return speed

    def exit(self):
        print('finished')


if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(5)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        car.trace()
        rate.sleep()
    rospy.on_shutdown(car.exit)
