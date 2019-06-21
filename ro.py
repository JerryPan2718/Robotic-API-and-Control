import sys
import cv2.cv as cv
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
                cv.ShowImage("Camera_OpenCV", self._img)
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
