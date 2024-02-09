import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Float32MultiArray
import subprocess
import math

# load in the calibration data
intrinsic_mat = np.array([925.259979,         0.0, 491.398274,
                    0.0        , 927.502076, 371.463298,
                    0.0        , 0.0       , 1.0]).reshape(3,3)

distortion_coef = np.array([-0.018452, 0.108834, 0.003492, 0.001679, 0.0]).reshape(1,5)

MARKER_SIZE = 1.5  # marker size is in meters
MARKER_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
PARAM_MARKERS = cv2.aruco.DetectorParameters()
DETECTOR = cv2.aruco.ArucoDetector(MARKER_DICT, PARAM_MARKERS)

class ImageDisplayNode(Node):
    def __init__(self):
        super().__init__('image_display_node')
        self.get_logger().info("Press 'q' to exit the node!")
        self.subscription = self.create_subscription(
            Image,
            '/drone/front/image_raw',  # Replace with your actual image topic
            self.image_callback,
            10
        )
        # data
        self.markerCorners = np.nan

        # For optical flow estimation
        self.oldCorner = np.array([[]])
        self.pointSelected = False
        self.arucoDetected = False
        self.stopCode      = False

        self.lk_params = dict(winSize=(20,20),
                              maxLevel=4,
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,10,0.01))
        self.old_gray_image = np.nan

        self.publisher = self.create_publisher(Float32MultiArray, '/corner_data', 10)
        self.dataCorner = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ])
        self.cv_bridge = CvBridge()
        self.process = subprocess.Popen("ros2 topic pub /drone/takeoff std_msgs/msg/Empty {} --once", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        # https://stackoverflow.com/questions/76802576/how-to-estimate-pose-of-single-marker-in-opencv-python-4-8-0
    def estimatePoseSingleMarkers(self, corners, marker_size, mtx, distortion):
        '''
        This will estimate the rvec and tvec for each of the marker corners detected by:
        corners, ids, rejectedImgPoints = detector.detectMarkers(image)
        corners - is an array of detected corners for each detected marker in the image
        marker_size - is the size of the detected markers
        mtx - is the camera matrix
        distortion - is the camera distortion matrix
        RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
        '''
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, -marker_size / 2, 0],
                                [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        trash = []
        rvecs = []
        tvecs = []
        for c in corners:
            nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R)
            tvecs.append(t)
            trash.append(nada)
        return rvecs, tvecs, trash

    def flatten_nested_list(self, nested_list):
        return [float(item) for sublist in nested_list for item in sublist]
    
    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format, its already on 1280x720
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #cv_image = cv2.resize(cv_image, (960,720), interpolation=cv2.INTER_AREA) // Uncomment this if ou want to resize them, not recommended though

        # finding the center of the frame
        (h,w) = cv_image.shape[:2]
        # Ploting the desired image location
        n1 = ((w//2) - 150 , (h//2) - 150) # [490, 210]
        n2 = ((w//2) - 150 , (h//2) + 150) # [490, 510]
        n3 = ((w//2) + 150 , (h//2) + 150) # [790, 510]
        n4 = ((w//2) + 150 , (h//2) - 150) # [790, 210]

        # EDITABLE
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

         # Trying to detect the marker with the object detector defined up there
        self.markerCorners, self.markerIds, rejectedCandidates = DETECTOR.detectMarkers(gray_image)
        self.stopCode = False

        # This connect to if there is detection
        if len(self.markerCorners) > 0:
            # Flattens the IDs
            # Getting the Pose of Estimation by getting the Translation and Rotation Vector
            # Based on the markerCorners, so Kalman Filter shoould estimate the points --> points_hat
            # rVec, tVec, trash = self.estimatePoseSingleMarkers(self.markerCorners, MARKER_SIZE, intrinsic_mat, distortion_coef)
            rVec, tVec,_ = cv2.aruco.estimatePoseSingleMarkers(self.markerCorners, MARKER_SIZE, intrinsic_mat, distortion_coef)
            try: # I'm just gonna take the marker ID with value of 4
                mark = 4
                index = [value[0] for value in self.markerIds].index(mark)

                # Clockwise rotation in order start from top_left
                corners = self.markerCorners[index]
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)

                # markerCorners is the list of corners of the detected markers. 
                # For each marker, its four corners are returned in their original order 
                # (which is clockwise starting with top left). So, the first corner is the top left corner, 
                # followed by the top right, bottom right and bottom left.
                top_left     = corners[0].ravel()
                top_right    = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left  = corners[3].ravel()

                # Euclidean Distance from aruco pose estimations (It's still estimation don't forgot!)
                Z = round(math.sqrt(
                        tVec[index][0][0] **2 + tVec[index][0][1] **2 + tVec[index][0][2] **2
                    ),3) # Already in meters

                floatArrayMsgData = Float32MultiArray()

                floatArrayMsgData.data = [  float(top_left[0])      , float(top_left[1]), 
                                            float(bottom_left[0])   , float(bottom_left[1]), 
                                            float(bottom_right[0])  , float(bottom_right[1]), 
                                            float(top_right[0])     , float(top_right[1]),
                                            Z]
                self.publisher.publish(floatArrayMsgData)

                cv2.putText(
                    cv_image,
                    f"Z-Relative: {round(tVec[index][0][2],3)} m",
                    (top_right[0]+20, top_right[1]+80),
                    cv2.FONT_HERSHEY_DUPLEX,
                    0.6,
                    (0,255,0),
                    2,
                    cv2.LINE_AA
                )
         
                # For Optical flow
                self.arucoDetected = True
                self.stopCode = True
                self.oldCorner = np.array([top_left, top_right, bottom_right, bottom_left],dtype=np.float32)

                corners = (np.array([[top_left, top_right, bottom_right, bottom_left]], dtype=np.float32),)
                # Draw the corners with convinient aruco library
                cv2.aruco.drawDetectedMarkers(cv_image, corners)
            except:
                self.get_logger().info(f"No Marker with ID:2 Detected\n")
                # Don't do nothing, just continue to next if statement

        if self.arucoDetected and self.stopCode == False:
            # Estimate marker corner if not available 
            new_corner, _, _ = cv2.calcOpticalFlowPyrLK(self.old_gray_image, gray_image, self.oldCorner, None, **self.lk_params)
            self.oldCorner  = new_corner
            new_corner = new_corner.astype(int)
            corners = (np.array([new_corner], dtype=np.float32),)

            # Estimate pose from marker corner estimation
            #cv2.aruco.drawDetectedMarkers(cv_image, corners, borderColor=(255,0,0))
            rVec, tVec,_ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, intrinsic_mat, distortion_coef)

            # Clockwise rotation in order start from top_left
            corners = corners[0].reshape(4, 2)
            corners = corners.astype(int)

            # markerCorners is the list of corners of the detected markers. 
            # For each marker, its four corners are returned in their original order 
            # (which is clockwise starting with top left). So, the first corner is the top left corner, 
            # followed by the top right, bottom right and bottom left.
            top_left     = corners[0].ravel()
            top_right    = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left  = corners[3].ravel()

            # Euclidean Distance from aruco pose estimations (It's still estimation don't forgot!)
            Z = round(math.sqrt(
                tVec[0][0][0] **2 + tVec[0][0][1] **2 + tVec[0][0][2] **2
            ),3) # Already in meters

            floatArrayMsgData = Float32MultiArray()
            floatArrayMsgData.data = [  float(top_left[0])             , float(top_left[1]), 
                                        float(bottom_left[0])          , float(bottom_left[1]), 
                                        float(bottom_right[0])         , float(bottom_right[1]), 
                                        float(top_right[0])            , float(top_right[1]),
                                        Z ]
            self.publisher.publish(floatArrayMsgData)

            cv2.putText(
                cv_image,
                f"Z Relative: {round(tVec[0][0][2],3)} m",
                (top_right[0]+20, top_right[1]+80),
                cv2.FONT_HERSHEY_DUPLEX,
                0.6,
                (0,255,0),
                2,
                cv2.LINE_AA
            )

        cv2.circle(cv_image, n1, 5, (255,0,0), 2)
        cv2.circle(cv_image, n2, 5, (255,0,0), 2)
        cv2.circle(cv_image, n3, 5, (255,0,0), 2)
        cv2.circle(cv_image, n4, 5, (255,0,0), 2)

        self.old_gray_image = gray_image.copy()
        key = cv2.waitKey(1)  # Refresh window
        # Safety Mechanism
        if key == ord("q"):
            raise SystemExit
        # Saving the display for logging
        if key == ord('s'):
            cv2.imwrite(f'./data/image{self.get_clock().now()}.png', cv_image)
            self.get_logger().info("Successfully saved the image!")

        # Resize the window frame to 60% Downscale for easy monitoring in the node
        cv_image = cv2.resize(cv_image, (576,360), interpolation=cv2.INTER_AREA) 
        # Display the image
        cv2.imshow('Image Display Node', cv_image)
            

def main(args=None):
    rclpy.init(args=args)
    image_display_node = ImageDisplayNode()
    try:
        rclpy.spin(image_display_node)
    except SystemExit:                 # <--- process the exception 
        rclpy.logging.get_logger("Quitting").info('Done')
    image_display_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
