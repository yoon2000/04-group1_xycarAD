import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class LineDetector:

    def __init__(self, topic):
        self.scan_height, self.scan_width = 20, 200
        self.image_width = 640
        self.lmid, self.rmid = self.scan_width, self.image_width - self.scan_width
        self.left, self.right = -1, -1
        self.roi_vertical_pos = 300
        self.area_width, self.area_height = 10, 6
        self.pixel_cnt_threshold = 0.5 * self.area_width * self.area_height
        self.row_begin = (self.scan_height - self.area_height) // 2
        self.row_end = self.row_begin + self.area_height
        self.value_threshold = -1

        # Initialize various class-defined attributes, and then...
        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.mask = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)
        self.edge = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)
        self.bridge = CvBridge()
        rospy.Subscriber(topic, Image, self.conv_image)

    def conv_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        self.gray = cv2.cvtColor(self.cam_img, cv2.COLOR_BGR2GRAY)

        self.Blur = cv2.GaussianBlur(self.gray, (5, 5), 0)
        hsv = cv2.cvtColor(self.cam_img, cv2.COLOR_BGR2GRAY)
        lower_white = np.array([0, 0, 70])
        upper_white = np.array([131, 255, 255])
        self.range = cv2.inRange(hsv, lower_white, upper_white)

        sobelx = cv2.Sobel(self.Blur, cv2.CV_64F, 1, 0, ksize=1)
        sobelx = cv2.convertScaleAbs(sobelx)

        sobely = cv2.Sobel(self.Blur, cv2.CV_64F, 0, 1, ksize=5)
        sobely = cv2.convertScaleAbs(sobely)

        self.img_sobel = cv2.addWeighted(sobelx, 1, sobely, 1, 0)

        height, width = self.cam_img.shape[:2]
        vertices = np.array([[(0, 330), (width / 2 - 160, height / 2), (width / 2 + 160, height / 2), (width, 340)]],
                            dtype=np.int32)

        mask = np.zeros_like(self.img_sobel)

        if len(self.img_sobel.shape) > 2:
            color = (255, 255, 255)
        else:
            color = 255

        cv2.fillPoly(mask, vertices, color)

        self.ROI_image = cv2.bitwise_and(self.img_sobel, mask)

        ret, self.ROI_image = cv2.threshold(self.ROI_image, 220, 255, cv2.THRESH_BINARY)

        lines = cv2.HoughLinesP(self.ROI_image, 1, np.pi / 180, 140, 100, 5)

        self.zero = np.zeros_like(self.cam_img)

        cv2.fillPoly(self.zero, vertices, (0, 0, 0))

        if lines is not None:
            for i in range(len(lines)):
                for x1, y1, x2, y2 in lines[i]:
                    cv2.line(self.zero, (x1, y1), (x2, y2), (0, 0, 255), 3)

        self.bin = cv2.cvtColor(self.zero, cv2.COLOR_BGR2GRAY)

    def detect_lines(self):
        self.left = -40
        self.right = 680
        self.l_dot = False
        self.r_dot = False

        for l in range(80, 290):
            area = self.bin[285: 295, l - 10: l + 10]
            if cv2.countNonZero(area) > self.pixel_cnt_threshold:
                self.left = l
                break

        for r in range(620, 350, -1):
            area = self.bin[285: 295, r - 10: r + 10]
            if cv2.countNonZero(area) > self.pixel_cnt_threshold:
                self.right = r
                break

        r_dotarea = self.range[240: 340, 440: 640]
        l_dotarea = self.range[240: 340, 80: 280]
        if 50 < cv2.countNonZero(r_dotarea) < 300:
            self.l_dot = True
        elif 50 < cv2.countNonZero(l_dotarea) < 300:
            self.r_dot = True

        # Return positions of left and right lines detected.
        return self.left, self.right, self.l_dot, self.r_dot

    def show_images(self, left, right):
        if left != -40:
            lsquare = cv2.rectangle(self.zero,
                                    (left - 10, 285),
                                    (left + 10, 295),
                                    (0, 255, 0), 3)
        else:
            print("Lost left line")

        if right != 640:
            rsquare = cv2.rectangle(self.zero,
                                    (right - 10, 285),
                                    (right + 10, 295),
                                    (0, 255, 0), 3)
        else:
            print("Lost right line")

        # cv2.imshow("view", self.cam_img)
        cv2.imshow("ROI", self.zero)
        print("left : ", left, "right : ", right)
        # cv2.imshow("img_sobel", self.img_sobel)

        cv2.waitKey(5)

        # Display images for debugging purposes;
        # do not forget to call cv2.waitKey().
