import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import numpy as np


# Ensure the correct path to the cascade file
face_cascade = cv2.CascadeClassifier('/home/mike/Documents/Robotics/Ex4/ex4/facetracking_ws/src/webcam_tracking/webcam_tracking/haarcascade_frontalface_default.xml')

frame_rate = 30
detection_interval = 100

def prep(img):
    img = cv2.flip(img, 1)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return gray, img

def get_trackable_points(gray, img):

    faces = face_cascade.detectMultiScale(gray, 1.1, 5)
    p0 = []
    if len(faces) != 0:
        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
            roi_gray = gray[y:y+h, x:x+w]
            p0 = cv2.goodFeaturesToTrack(roi_gray, maxCorners=70, qualityLevel=0.001, minDistance=5)
            if p0 is not None:
                p0 = np.intp(p0).reshape(-1, 2)
                p0[:, 0] += x
                p0[:, 1] += y

    return p0, faces, img

def do_track_face(gray_prev, gray, p0):

    '''Compute the optical flow and track points from gray_prev (previous frame) to gray (current frame). 
        Input
        gray_prev: The previous frame.
        gray: The current frame.
        p0: The points from the previous frame that we want to track in the current frame.

        Output: 
        p1: The new positions of the tracked points in the current frame.       
    '''
    p0 = np.float32(p0)
    p1, isFound, err = cv2.calcOpticalFlowPyrLK(gray_prev, gray, p0, None, 
                                                winSize=(31, 31),
                                                maxLevel=10,
                                                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.03),
                                                flags=cv2.OPTFLOW_LK_GET_MIN_EIGENVALS,
                                                minEigThreshold=0.00025)
    isFound = isFound.reshape(-1)
    p1 = p1[isFound == 1]
    return p1

class WebcamSubscriber(Node):

    def __init__(self):
        super().__init__('webcam_subscriber')

        self.subscription = self.create_subscription(Image, 'webcam_output', self.listener_callback, 10)
        self.cv_bridge = CvBridge()

        # Initialize previous frame and points
        self.prev_time = 0
        self.gray_prev = None
        self.p0 = []
        self.frame_counter = 0
        self.faces = []
        self.frame = None

    def listener_callback(self, msg):

        # Recieving image message

        self.frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info("Image Received")
        time_elapsed = time.time() - self.prev_time

        if time_elapsed > 1./frame_rate:
            self.prev_time = time.time()
            gray, img = prep(self.frame)

            # Run detection or tracking based on the frame counter
            # The system alternates between detecting faces and tracking them. 
            # It does face detection every detection_interval frames or if there are fewer than 10 trackable points (self.p0).
            #  This helps reduce the computational load by not running the face detection algorithm on every frame.
            if self.frame_counter >= detection_interval or len(self.p0) <= 10:
                self.p0, self.faces, img = get_trackable_points(gray, img)
                self.gray_prev = gray.copy()
                self.frame_counter = 0
            else:
                # Perform face tracking
                for (x, y, w, h) in self.faces:
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
                if len(self.p0) > 0:
                    p1 = do_track_face(self.gray_prev, gray, self.p0)
                    for point in p1:
                        cv2.drawMarker(img, (int(point[0]), int(point[1])), [255, 0, 0], 0)
                    self.p0 = p1

            # Show the updated video frame
            cv2.imshow('Video feed', img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.get_logger().info('Closing stream window..')
                cv2.destroyAllWindows()

            # Update the previous frame and frame counter
            self.gray_prev = gray
            self.frame_counter += 1

def main():
    try:
        rclpy.init(args=None)
        webcam_subscriber = WebcamSubscriber()
        rclpy.spin(webcam_subscriber)

    except KeyboardInterrupt:
        webcam_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
