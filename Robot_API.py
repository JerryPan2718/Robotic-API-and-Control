import cv2
import cv2.cv as cv
import numpy as np
import vision_definitions
from naoqi import ALProxy
import time

# Self defined functions
import Select_color

class OpenCVModule:
    def __init__(self, camera_id):
        self._videoProxy = None
        self._cameraID = camera_id
        self._resolution = vision_definitions.kQVGA  # 320 * 240
        self._colorSpace = vision_definitions.kBGRColorSpace
        self._fps = 30
        self._imgClient = ""
        self._imgData = None
        self._img = cv.CreateImageHeader((320, 240), cv.IPL_DEPTH_8U, 3)
        cv.NamedWindow("Camera_OpenCV_" + str(camera_id), 0)

    def subscribe_img_client(self, video_proxy):
        self._imgClient = video_proxy.subscribeCamera("OpenCV_Client", self._cameraID, self._resolution,
                                                     self._colorSpace, self._fps)

    def cancel_img_client(self, video_proxy):
        if self._imgClient is not "":
            video_proxy.unsubscribe(self._imgClient)

    def __show_image(self, center_ra, frame):
        print time.time(), center_ra
        if center_ra != []:
            print "???"
            #cv2.circle(frame, (center_ra[0][0], center_ra[0][1]), center_ra[1], (255, 0, 0), 5)
            #cv2.circle(frame, (center_ra[0][0], center_ra[0][1]), 2, (255, 0, 255), 10)
        cv2.imshow("came", frame)

    def detect_ball(self, video_proxy):
        self._imgData = video_proxy.getImageRemote(self._imgClient)
        cv.SetData(self._img, self._imgData[6])
        mat = self._img[:]
        frame = np.array(mat)  # Frame: An image
        frame2 = frame
        center_ra = Select_color.select_color(frame2)
        self.__show_image(center_ra = center_ra, frame = frame2)
        return center_ra

class MotionModule:
    def __init__(self, ip, port):
        self.motion = ALProxy("ALMotion", ip, port)
        self.posture = ALProxy("ALRobotPosture", ip, port)
        self.head_yaw = 0
        self.head_pitch = 0

    def stand_up(self):
        self.posture.goToPosture("StandInit", 0.5)

    def get_head_yaw(self):
        return self.head_yaw

    def get_head_pitch(self):
        return self.head_pitch

    def set_head_yaw(self, angle):
        self.motion.setStiffnesses("HeadYaw", 1)
        self.head_yaw = angle
        self.motion.setAngles(["HeadYaw"], self.head_yaw, 0.1)

    def set_head_pitch(self, angle):
        self.motion.setStiffnesses("HeadPitch", 1)
        self.head_pitch = angle
        self.motion.setAngles(["HeadPitch"], self.head_pitch, 0.1)

    # angle > 0, to the left
    def turn_theta(self, angle):
        self.motion.moveTo(0, 0, angle)

    def head_turn_theta(self, theta):
        new_head_yaw = self.head_yaw + theta
        if not -2.0857 < new_head_yaw < 2.0857:
            return 0
        else:
            self.set_head_yaw(new_head_yaw)
            return 1

class Robot:
    def __init__(self, ip, port, camera_id0, camera_id1):
        self.mM = MotionModule(ip, port)
        self._videoProxy = ALProxy("ALVideoDevice", ip, port)
        self._OCVM0 = ""
        self._OCVM1 = ""
        if camera_id0 >= 0:
            self._OCVM0 = OpenCVModule(camera_id = camera_id0)
            self._OCVM0.subscribe_img_client(self._videoProxy)
        if camera_id1 >= 0 and camera_id1 != camera_id0:
            self._OCVM1 = OpenCVModule(camera_id = camera_id1)
            self._OCVM1.subscribe_img_client(self._videoProxy)

    def adjust_body(self, detect_res):
        if detect_res == []:
            return 0
        ball_center = detect_res[0]
        if ball_center[0] < 150:  # At the left
            self.mM.head_turn_theta(0.02)
            return 1
        elif ball_center[0] > 170:
            self.mM.head_turn_theta(-0.02)
            return 1
        else:
            head_yaw = self.mM.get_head_yaw()
            self.mM.turn_theta(head_yaw)
            self.mM.set_head_yaw(0)
            return 2

    def find_ball(self):
        self.mM.stand_up()
        self.mM.set_head_yaw(0)
        found = 0
        res = 0
        while found is 0:
            detect_res = self._OCVM0.detect_ball(self._videoProxy)
            res = self.adjust_body(detect_res = detect_res)
            if res is 2:
                found = 1

    def __del__(self):
        if self._OCVM0 is not "":
            self._OCVM0.cancel_img_client(self._videoProxy)
        if self._OCVM1 is not "":
            self._OCVM1.cancel_img_client(self._videoProxy)
        cv.DestroyAllWindows()


if __name__ == '__main__':
    IP = "169.254.199.42"
    PORT = 9559
    CameraID = 0
    myRobot = Robot(ip = IP, port = PORT, camera_id0 = CameraID, camera_id1 = 0)
    myRobot.find_ball()
