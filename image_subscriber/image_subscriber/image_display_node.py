import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from cv2 import aruco

MARKER_DICT = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
PARAM_MARKERS = aruco.DetectorParameters()
DETECTOR = cv2.aruco.ArucoDetector(MARKER_DICT, PARAM_MARKERS)

class ImageDisplayNode(Node):
    def __init__(self):
        super().__init__('image_display_node')
        self.subscription = self.create_subscription(
            Image,
            '/drone/front/image_raw',  # Replace with your actual image topic
            self.image_callback,
            10
        )
        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # finding the center of the frame
        (h,w) = cv_image.shape[:2]

        n1 = ((w//2) - 150 , (h//2) - 150)
        n2 = ((w//2) - 150 , (h//2) + 150)
        n3 = ((w//2) + 150 , (h//2) + 150)
        n4 = ((w//2) + 150 , (h//2) - 150)

        # Ploting the desired image location
        cv2.circle(cv_image, n1, 5, (255,0,0), 2)
        cv2.putText(cv_image, f"Nomor 1", n1, cv2.FONT_HERSHEY_DUPLEX, 0.2, (0,255,0),1, cv2.LINE_AA)

        cv2.circle(cv_image, n2, 5, (255,0,0), 2)
        cv2.putText(cv_image, f"Nomor 2", n2, cv2.FONT_HERSHEY_DUPLEX, 0.2, (0,255,0),1, cv2.LINE_AA)

        cv2.circle(cv_image, n3, 5, (255,0,0), 2)
        cv2.putText(cv_image, f"Nomor 3", n3, cv2.FONT_HERSHEY_DUPLEX, 0.2, (0,255,0),1, cv2.LINE_AA)

        cv2.circle(cv_image, n4, 5, (255,0,0), 2)
        cv2.putText(cv_image, f"Nomor 4", n4, cv2.FONT_HERSHEY_DUPLEX, 0.2, (0,255,0),1, cv2.LINE_AA)
        self.get_logger().info(f'n1 : {n1}\nn2: {n2}\nn3 :{n3}\nn4 : {n4}')
        

        # EDITABLE
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
         # Trying to detect the marker with the object detector defined up there
        markerCorners, markerIds, rejectedCandidates = DETECTOR.detectMarkers(gray_image)

        if len(markerCorners) > 0:
            # Flattens the IDs
            markerIds = markerIds.flatten()
            # Loop over the IDs
            for (markerCorner, markerId) in zip(markerCorners, markerIds):
                # extract the marker corners (which are always returned
                # in top-left, top-right, bottom-right, and bottom-left
                # order)
                corners = markerCorner.reshape((4,2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                # Draw the bounding box of the aruco detection
                aruco.drawDetectedMarkers(cv_image, markerCorners)

                # Compute and draw the center of each marker Ids
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(cv_image, (cX,cY), 3, (0,0,255), -1)

                # Draw the marker ID on the frame
                cv2.putText(cv_image, str(markerId),
                    (topLeft[0], topLeft[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
                
                # Draw number for debug
                cv2.putText(cv_image, f"Nomor 1", topLeft, cv2.FONT_HERSHEY_DUPLEX, 0.2, (255,0,0),1, cv2.LINE_AA)
                cv2.putText(cv_image, f"Nomor 2", bottomLeft, cv2.FONT_HERSHEY_DUPLEX, 0.2, (255,0,0),1, cv2.LINE_AA)
                cv2.putText(cv_image, f"Nomor 3", bottomRight, cv2.FONT_HERSHEY_DUPLEX, 0.2, (255,0,0),1, cv2.LINE_AA)
                cv2.putText(cv_image, f"Nomor 4", topRight, cv2.FONT_HERSHEY_DUPLEX, 0.2, (255,0,0),1, cv2.LINE_AA)


        # Display the image
        cv2.imshow('Image Display Node', cv_image)
        cv2.waitKey(1)  # Refresh window

def main(args=None):
    rclpy.init(args=args)
    image_display_node = ImageDisplayNode()
    rclpy.spin(image_display_node)
    #image_display_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
