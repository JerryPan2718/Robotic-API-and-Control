import sys
import cv2.cv as cv
import cv2
import numpy as np
from naoqi import ALProxy
import vision_definitions


class OpenCVModule():
    def __init__(self, IP, PORT, CameraID):
        self._videoProxy = None
        self._cameraID = CameraID
        self._resolution = vision_definitions.kQVGA  # 320 * 240
        self._colorSpace = vision_definitions.kBGRColorSpace
        self._fps = 30
        self._imgClient = ""
        self._imgData = None
        self._img = cv.CreateImageHeader((320, 240), cv.IPL_DEPTH_8U, 3)
        cv.NamedWindow("Camera_OpenCV", 0)

        self._registerImageClient(IP, PORT)

    def _registerImageClient(self, IP, PORT):
        self._videoProxy = ALProxy("ALVideoDevice", IP, PORT)
        self._imgClient = self._videoProxy.subscribeCamera("OpenCV_Client", self._cameraID, self._resolution,
                                                           self._colorSpace, self._fps)

    def _unregisterImageClient(self):
        if self._imgClient != "":
            self._videoProxy.unsubscribe(self._imgClient)

    def showImage(self):
        while True:
            try:
                self._imgData = self._videoProxy.getImageRemote(self._imgClient)
                cv.SetData(self._img, self._imgData[6])
                mat=self._img[:]
                frame=np.array(mat)
                im_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                im_h, im_s, im_v = cv2.split(im_hsv)
                lower = np.array([30, 0, 0])
                upper = np.array([40, 255, 255])
                mask = cv2.inRange(im_hsv, lower, upper)
                res = cv2.bitwise_and(frame, frame, mask=mask)
                im_gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
                im_bw = cv2.adaptiveThreshold(im_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 7, 0)
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
                im_open = cv2.morphologyEx(im_bw, cv2.MORPH_OPEN, kernel)
                circles = cv2.HoughCircles(im_open, cv2.cv.CV_HOUGH_GRADIENT, 1, 100, param1=100, param2=10, minRadius=20,maxRadius=40)
                if circles is None:
                    cv2.imshow("came", frame)
                else:
                    circles = np.uint16(np.around(circles))
                    for i in circles[0, :]:
                        cv2.circle(frame, (i[0], i[1]), i[2], (255, 0, 0), 5)
                        cv2.circle(frame, (i[0], i[1]), 2, (255, 0, 255), 10)
                    cv2.imshow("came", frame)

            except KeyboardInterrupt:
                break
            except:
                pass
            if cv.WaitKey(10) == 27:
                break
        cv.DestroyAllWindows()
        self._unregisterImageClient()


if __name__ == '__main__':
    IP = "169.254.199.42"


PORT = 9559
CameraID = 0
if len(sys.argv) > 1:
    IP = sys.argv[1]
if len(sys.argv) > 2:
    CameraID = int(sys.argv[2])
myWidget = OpenCVModule(IP, PORT, CameraID)
myWidget.showImage()
