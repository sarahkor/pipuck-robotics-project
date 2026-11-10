import cv2

class BaseCamera:
    def __init__(self, device=1):
        # 0 is usually the base camera
        self.device = device 
        self.cap = None

    def open(self):
        self.cap = cv2.VideoCapture(self.device)
        # The base camera's typical maximum resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    def get_picture(self):
        if not self.cap or not self.cap.isOpened():
            self.open()
        
        ok, frame = self.cap.read()
        if ok:
            return frame
        else:
            print("Failed to grab a picture.")
            return None

    def close(self):
        if self.cap:
            self.cap.release()

if __name__ == "__main__":
    print("Testing base camera...")
    
    # Create the camera object
    cam = BaseCamera(device=1) 
    
    # Try to take a picture
    img = cam.get_picture()
    
    if img is not None:
        # Save the picture to a file called 'base_camera_test.jpg'
        cv2.imwrite("pi0_base_camera_test1.jpg", img)
        print("Success! Image saved as pi0_base_camera_test1.jpg")
    else:
        print("Failed to get image. The device number might not be 1.")
    
    # Close the camera when done
    cam.close()