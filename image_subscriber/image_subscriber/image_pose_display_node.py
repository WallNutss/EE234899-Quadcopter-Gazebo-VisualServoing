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
# calib_data_path = "/home/wallnuts/ros2_sjtu_ws/src/image_subscriber/image_subscriber/data_calibration_camera/MultiMatrix.npz"

# calib_data = np.load(calib_data_path)

# cam_mat = calib_data["camMatrix"]
# dist_coef = calib_data["distCoef"]
# r_vectors = calib_data["rVector"]
# t_vectors = calib_data["tVector"]
cam_mat = np.array([925.259979,         0.0, 491.398274,
                    0.0        , 927.502076, 371.463298,
                    0.0        , 0.0       , 1.0]).reshape(3,3)

dist_coef = np.array([-0.018452, 0.108834, 0.003492, 0.001679, 0.0]).reshape(1,5)

MARKER_SIZE = 150  # centimeters (measure your printed marker size)
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
    

    def normalized(self, data):
        return [data[0]/1280, data[1]/720
                ]
    
    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format, its already on 1280x720
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #cv_image = cv2.resize(cv_image, (960,720), interpolation=cv2.INTER_AREA)
        # finding the center of the frame
        (h,w) = cv_image.shape[:2]
        # Ploting the desired image location
        n1 = ((w//2) - 150 , (h//2) - 150) # [490, 210]
        n2 = ((w//2) - 150 , (h//2) + 150) # [490, 510]
        n3 = ((w//2) + 150 , (h//2) + 150) # [790, 510]
        n4 = ((w//2) + 150 , (h//2) - 150) # [790, 210]

        # Try Projection Transformation at Target
        #self.get_logger().info(f'n1 : {n1}\nn2: {n2}\nn3 :{n3}\nn4 : {n4}')
        
        #self.get_logger().info(f'\nwidth : {w}\nheight: {h}')

        # Ploting the desired image location
        # cv2.putText(cv_image, f"Nomor 1", n1, cv2.FONT_HERSHEY_DUPLEX, 0.8, (0,255,0),1, cv2.LINE_AA)
        # cv2.putText(cv_image, f"Nomor 2", n2, cv2.FONT_HERSHEY_DUPLEX, 0.8, (0,255,0),1, cv2.LINE_AA)
        # cv2.putText(cv_image, f"Nomor 3", n3, cv2.FONT_HERSHEY_DUPLEX, 0.8, (0,255,0),1, cv2.LINE_AA)
        # cv2.putText(cv_image, f"Nomor 4", n4, cv2.FONT_HERSHEY_DUPLEX, 0.8, (0,255,0),1, cv2.LINE_AA)

        # EDITABLE
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
         # Trying to detect the marker with the object detector defined up there
        markerCorners, markerIds, rejectedCandidates = DETECTOR.detectMarkers(gray_image)

        if len(markerCorners) > 0:
            # Flattens the IDs
            # Getting the Pose of Estimation by getting the Translation and Rotation Vector
            rVec, tVec, trash = self.estimatePoseSingleMarkers(markerCorners, MARKER_SIZE, cam_mat, dist_coef)
            # Loop over the IDs
            total_markers = range(0, markerIds.size)
            for ids, corners, i in zip(markerIds, markerCorners, total_markers):
                cv2.aruco.drawDetectedMarkers(cv_image, markerCorners)

                # Clockwise rotation in order start from top_left
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
 
                top_left    = corners[0].ravel()
                top_right    = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left  = corners[3].ravel()

                # Finding the center
                centerX = round(((top_left[0]- top_right[0])/2) + top_right[0])
                centerY = round(((bottom_right[1]- top_right[1])/2) + top_right[1])
                cv2.circle(cv_image, (centerX, centerY), 5, (0,0,255), 2)

                Z = float(tVec[0][2][0])/100 # cm --> m (untuk sekarang depth masih dalam m untuk dipasskan kedalam Jacobian)

                #self.get_logger().info(f"{str(data)}")

                floatArrayMsgData = Float32MultiArray()
                #data = self.flatten_nested_list([top_right, bottom_left, bottom_right, top_left])
                #data.append(float(tVec[0][2][0]*10))  # appending depth point estimation, times 10 to get mm unit measurements
                
                floatArrayMsgData.data = [float(top_left[0]), float(top_left[1]), 
                                          float(bottom_left[0]), float(bottom_left[1]), 
                                          float(bottom_right[0]), float(bottom_right[1]), 
                                          float(top_right[0]), float(top_right[1]), Z]
                #floatArrayMsgData.data = data
                self.publisher.publish(floatArrayMsgData)
                #self.dataCorner = np.vstack((self.dataCorner, self.flatten_nested_list([top_left, bottom_left, bottom_right, top_right])))

                # cv2.circle(cv_image, (top_left), 5, (61,7,219), 2)
                # cv2.circle(cv_image, (top_right), 5, (7,165,219), 2)
                # cv2.circle(cv_image, (bottom_right), 5, (210,199,142), 2)
                # cv2.circle(cv_image, (bottom_left), 5, (13,105,134), 2)

                # Calculate distance trial
                # distance = np.sqrt(
                #     tVec[i][0] **2 + tVec[i][1] **2 + tVec[i][2] **2
                # )
                # self.get_logger().info(str(distance))
                #point = cv2.drawFrameAxes(cv_image, cam_mat, dist_coef, rVec[i], tVec[i], 75, 2)


                #Draw number for debug
                #self.get_logger().info(f'top_left : {top_left}\ntop_right: {top_right}\nbottom_right :{bottom_right}\nbottom_left : {bottom_left}')
                # cv2.putText(cv_image, f"top_left", top_left, cv2.FON2T_HERSHEY_DUPLEX, 0.6, (61,7,219),1, cv2.LINE_AA)
                # cv2.putText(cv_image, f"bottom_left", bottom_left, cv2.FONT_HERSHEY_DUPLEX, 0.6, (13,105,134),1, cv2.LINE_AA)
                # cv2.putText(cv_image, f"bottom_right", bottom_right, cv2.FONT_HERSHEY_DUPLEX, 0.6, (210,199,142),1, cv2.LINE_AA)
                # cv2.putText(cv_image, f"top_right", top_right, cv2.FONT_HERSHEY_DUPLEX, 0.6, (7,165,219),1, cv2.LINE_AA)

                # # Draw the information
                cv2.putText(
                    cv_image,
                    f"Id: {ids[0]} Distance: {round(math.sqrt(tVec[i][0][0]**2 + tVec[i][1][0]**2 + tVec[i][2][0]**2)/100,3)} m",
                    top_right+20,
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
        
        key = cv2.waitKey(1)  # Refresh window
        if key == ord("q"):
            #np.savetxt('./data/result-skenario2-try004.txt', self.dataCorner, fmt='%s')
            raise SystemExit
        # Saving the display for logging
        if key == ord('s'):
            cv2.imwrite(f'./data/{self.get_clock().now()}_image.png', cv_image)
            self.get_logger().info("Successfully saved the image!")
        # Resize the window frame to 60% Downscale for easy monitoring in the node
        cv_image = cv2.resize(cv_image, (640,360), interpolation=cv2.INTER_AREA) 
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
