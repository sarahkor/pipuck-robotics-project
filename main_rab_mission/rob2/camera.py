import cv2

class Camera:
    def __init__(self, device=0):
        self.device = device 
        self.cap = None

    def open(self):
        self.cap = cv2.VideoCapture(self.device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    def get_picture(self):
        if not self.cap or not self.cap.isOpened():
            self.open()
        
        ok, frame = self.cap.read()
        if ok:
            return frame
        else:
            print(f"Failed to grab a picture from device {self.device}.")
            return None

    def close(self):
        if self.cap:
            self.cap.release()