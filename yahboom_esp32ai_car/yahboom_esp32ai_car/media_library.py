#!/usr/bin/env python3
# encoding: utf-8
import base64
import math
import rclpy
from rclpy.node import Node
import cv2 as cv
import mediapipe as mp
from time import sleep
from std_msgs.msg import Int32, Bool,UInt16
from yahboomcar_msgs.msg import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


def get_dist(point1, point2):
    '''
    获取两点之间的距离
    (x1-x2)**2+(y1-y2)**2=dist**2
    '''
    x1, y1 = point1
    x2, y2 = point2
    return abs(math.sqrt(math.pow(abs(y1 - y2), 2) + math.pow(abs(x1 - x2), 2)))

def calc_angle(pt1, pt2, pt3):
    '''
    求中间点的夹角
    cos(B)=(a^2+c^2-b^2)/2ac
    '''
    a = get_dist(pt1, pt2)
    b = get_dist(pt2, pt3)
    c = get_dist(pt1, pt3)
    try:
        radian = math.acos((math.pow(a, 2) + math.pow(b, 2) - math.pow(c, 2)) / (2 * a * b))
        angle = radian / math.pi * 180
    except: angle = 0
    return angle

class HandDetector:
    def __init__(self, mode=False, maxHands=1, detectorCon=0.5, trackCon=0.5):
        self.mpHand = mp.solutions.hands
        self.draw = False
        self.mpDraw = mp.solutions.drawing_utils
        self.hands = self.mpHand.Hands(
            static_image_mode=mode,
            max_num_hands=maxHands,
            min_detection_confidence=detectorCon,
            min_tracking_confidence=trackCon
        )

    def findHands(self, frame):
        lmList = []
        self.cxList = []
        self.cyList = []
        bbox = 0, 0, 0, 0
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.hands.process(img_RGB)
        if self.results.multi_hand_landmarks:
            for i in range(len(self.results.multi_hand_landmarks)):
                if not self.draw: self.mpDraw.draw_landmarks(frame, self.results.multi_hand_landmarks[i], self.mpHand.HAND_CONNECTIONS)
                for id, lm in enumerate(self.results.multi_hand_landmarks[i].landmark):
                    h, w, c = frame.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    # print(id, lm.x, lm.y, lm.z)
                    lmList.append([id, cx, cy])
                    self.cxList.append(cx)
                    self.cyList.append(cy)
        if len(self.cxList) != 0 and len(self.cyList) != 0:
            xmin, xmax = min(self.cxList), max(self.cxList)
            ymin, ymax = min(self.cyList), max(self.cyList)
            bbox = xmin, ymin, xmax, ymax
            if self.draw: cv.rectangle(frame, (xmin - 20, ymin - 20), (xmax + 20, ymax + 20), (0, 255, 0), 2)
        return frame, lmList, bbox

    def fingersUp(self,lmList):
        fingers = []
        point1 = lmList[4][1:3]
        point2 = lmList[3][1:3]
        point3 = lmList[2][1:3]
        point4 = lmList[1][1:3]
        # Thumb
        if (abs(calc_angle(point1, point2, point3)) > 150.0) and (
                abs(calc_angle(point2, point3, point4)) > 150.0): fingers.append(1)
        else: fingers.append(0)
        # 4 finger
        tipIds = [4, 8, 12, 16, 20]
        for id in range(1, 5):
            if lmList[tipIds[id]][2] < lmList[tipIds[id] - 2][2]: fingers.append(1)
            else: fingers.append(0)
        return fingers

    def ThumbTOforefinger(self,lmList):
        point1 = lmList[4][1:3]
        point2 = lmList[0][1:3]
        point3 = lmList[8][1:3]
        return abs(calc_angle(point1, point2, point3))

    def get_gesture(self,lmList):
        gesture = ""
        fingers = self.fingersUp(lmList)
        if fingers[2] == fingers[3] == fingers[4] == 1:
            if self.ThumbTOforefinger(lmList) < 10: gesture = "OK"
        if fingers[1] == fingers[2] == 1 and sum(fingers) == 2: gesture = "Yes"
        try:
            if self.cyList[4] == max(self.cyList): gesture = "Thumb_down"
        except Exception as e: print("e: ", e)
        return gesture

class PoseDetector:
    def __init__(self, mode=False, smooth=True, detectionCon=0.5, trackCon=0.5):
        self.mpPose = mp.solutions.pose
        self.mpDraw = mp.solutions.drawing_utils
        self.pose = self.mpPose.Pose(
            static_image_mode=mode,
            smooth_landmarks=smooth,
            min_detection_confidence=detectionCon,
            min_tracking_confidence=trackCon)
        self.lmDrawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=6)
        self.drawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)

    def pubPosePoint(self, frame, draw=True):
        pointArray = []
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.pose.process(img_RGB)
        if self.results.pose_landmarks:
            if draw: self.mpDraw.draw_landmarks(frame, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
            for id, lm in enumerate(self.results.pose_landmarks.landmark):
                h, w, c = frame.shape
                pointArray.append([id, lm.x * w, lm.y * h, lm.z])
        return frame, pointArray

class Holistic:
    def __init__(self, staticMode=False, landmarks=True, detectionCon=0.5, trackingCon=0.5):
        self.mpHolistic = mp.solutions.holistic
        self.mpFaceMesh = mp.solutions.face_mesh
        self.mpHands = mp.solutions.hands
        self.mpPose = mp.solutions.pose
        self.mpDraw = mp.solutions.drawing_utils
        self.mpholistic = self.mpHolistic.Holistic(
            static_image_mode=staticMode,
            smooth_landmarks=landmarks,
            min_detection_confidence=detectionCon,
            min_tracking_confidence=trackingCon)
        self.lmDrawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=3)
        self.drawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)

    def findHolistic(self, frame, draw=True):
        poseptArray = []
        lhandptArray = []
        rhandptArray = []
        h, w, c = frame.shape
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.mpholistic.process(img_RGB)
        if self.results.pose_landmarks:
            if draw: self.mpDraw.draw_landmarks(frame, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
            for id, lm in enumerate(self.results.pose_landmarks.landmark):
                poseptArray.append([id, lm.x * w, lm.y * h, lm.z])
        if self.results.left_hand_landmarks:
            if draw: self.mpDraw.draw_landmarks(frame, self.results.left_hand_landmarks, self.mpHands.HAND_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
            for id, lm in enumerate(self.results.left_hand_landmarks.landmark):
                lhandptArray.append([id, lm.x * w, lm.y * h, lm.z])
        if self.results.right_hand_landmarks:
            if draw: self.mpDraw.draw_landmarks(frame, self.results.right_hand_landmarks, self.mpHands.HAND_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
            for id, lm in enumerate(self.results.right_hand_landmarks.landmark):
                rhandptArray.append([id, lm.x * w, lm.y * h, lm.z])
        return frame, poseptArray, lhandptArray, rhandptArray

class FaceMesh:
    def __init__(self, staticMode=False, maxFaces=1, minDetectionCon=0.5, minTrackingCon=0.5):
        self.mpDraw = mp.solutions.drawing_utils
        self.mpFaceMesh = mp.solutions.face_mesh
        self.faceMesh = self.mpFaceMesh.FaceMesh(
            static_image_mode=staticMode,
            max_num_faces=maxFaces,
            min_detection_confidence=minDetectionCon,
            min_tracking_confidence=minTrackingCon)
        self.lmDrawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=3)
        self.drawSpec = self.mpDraw.DrawingSpec(color=(0, 255, 0), thickness=1, circle_radius=1)

    def pubFaceMeshPoint(self, frame, draw=True):
        pointArray = []
        h, w, c = frame.shape
        imgRGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.faceMesh.process(imgRGB)
        if self.results.multi_face_landmarks:
            for i in range(len(self.results.multi_face_landmarks)):
                if draw: self.mpDraw.draw_landmarks(frame, self.results.multi_face_landmarks[i], self.mpFaceMesh.FACE_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
                for id, lm in enumerate(self.results.multi_face_landmarks[i].landmark):
                    pointArray.append([id, lm.x * w, lm.y * h, lm.z])
        return frame, pointArray

class Media_ROS(Node):
    def __init__(self):
        super().__init__("mediapipe")
        #rclpy.on_shutdowm(self.cancel)
        self.Joy_active = True
        
        self.pub_cmdVel = self.create_publisher(Twist,"/cmd_vel",1)
        self.pub_Buzzer = self.create_publisher(UInt16,"/beep",1)
        self.pub_img = self.create_publisher(Image,'/image',500)


    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = not self.Joy_active
        self.pub_vel(0, 0)

    def pub_vel(self, x, y, z):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = z
        self.pub_cmdVel.publish(twist)

    def pub_buzzer(self, status):
        self.pub_Buzzer.publish(status)

    def RobotBuzzer(self):
        self.pub_buzzer(True)
        sleep(1)
        self.pub_buzzer(False)
        sleep(1)
        self.pub_buzzer(False)
        for i in range(2):
            self.pub_vel(0, 0)
            sleep(0.1)

    # def pub_arm(self, joints, id=6, angle=180, runtime=500):
    #     arm_joint = ArmJoint()
    #     arm_joint.id = id
    #     arm_joint.angle = angle
    #     arm_joint.run_time = runtime
    #     if len(joints) != 0: arm_joint.joints = joints
    #     else: arm_joint.joints = []
    #     self.pubPoint.publish(arm_joint)
    #     # rospy.loginfo(arm_joint)

    def pub_imgMsg(self, frame):
        pic_base64 = base64.b64encode(frame)
        image = Image()
        size = frame.shape
        image.height = size[0]
        image.width = size[1]
        #image.channels = size[2]
        image.data = pic_base64
        self.pub_img.publish(image)

    def cancel(self):
        self.pub_cmdVel.publish(Twist())
        self.pub_cmdVel.unregister()
        self.pub_img.unregister()
        self.pub_Buzzer.unregister()
        #self.pubPoint.unregister()

class SinglePID:
    def __init__(self, P=0.1, I=0.0, D=0.1):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        Image_Msgprint("init_pid: ", P, I, D)
        self.pid_reset()

    def Set_pid(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        print("set_pid: ", P, I, D)
        self.pid_reset()

    def pid_compute(self, target, current):
        self.error = target - current
        self.intergral += self.error
        self.derivative = self.error - self.prevError
        result = self.Kp * self.error + self.Ki * self.intergral + self.Kd * self.derivative
        self.prevError = self.error
        return result

    def pid_reset(self):
        self.error = 0
        self.intergral = 0
        self.derivative = 0
        self.prevError = 0


