import numpy as np
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError

import tf
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped
from intera_interface import Cameras


class ArUcoMarker():

    _MARKER_LENGTH = 0.05  # Marker length in meters
    _OFFSET = 0.15  # Offset from marker to center of boundary

    def __init__(self) -> None:

        rospy.init_node('camera_display', anonymous=True)
        self.bridge = CvBridge()


        self.camera = Cameras()
        self.camera.start_streaming("head_camera")

        # Subscribers
        self.rgb_subscriber = rospy.Subscriber(
            "/io/internal_camera/head_camera/image_raw", 
            Image, 
            self.img_callback)
        
        self.camera_info = rospy.wait_for_message(
            "/io/internal_camera/head_camera/camera_info", 
            CameraInfo)
        
        self._dist_coeffs = np.array(self.camera_info.D)
        self._camera_matrix = np.array(self.camera_info.K).reshape(3, 3)


        # Setup tf2 broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()


        # Setup aruco detector
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters()


        # Setup buffer point for pose
        self.rvec = np.zeros((3, 1))
        self.tvec = np.zeros((3, 1))
        self.pose_buffer = np.array([[-self._MARKER_LENGTH/2, self._MARKER_LENGTH/2, 0],    # Bottom-left corner
                                     [self._MARKER_LENGTH/2, self._MARKER_LENGTH/2, 0],      # Bottom-right corner
                                     [self._MARKER_LENGTH/2, -self._MARKER_LENGTH/2, 0],    # Top-right corner
                                     [-self._MARKER_LENGTH/2, -self._MARKER_LENGTH/2, 0]],       # Top-left corner
                                    dtype=np.float64)
        

    def clean_shutdown():
        print("Shutting down camera_display node.")
        cv2.destroyAllWindows()


    def img_callback(self, data):
        """
        Callback function for the pose of the area marker.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            corners, ids, _ = cv2.aruco.detectMarkers(
                cv_image, self.aruco_dict, parameters=self.parameters)
            

            # Check if four markers are detected
            if ids is not None and len(ids) == 4:

                # Draw markers
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)


                # Calculate center of boundary formed by markers
                center = np.mean([np.mean(corner, axis=0)
                                 for corner in corners], axis=0)


                # Calculate pose 
                _, self.rvec, self.tvec = cv2.solvePnP(self.pose_buffer, center, self._camera_matrix, self._dist_coeffs, self.rvec, self.tvec)
                cv2.drawFrameAxes(cv_image, self._camera_matrix, self._dist_coeffs, self.rvec, self.tvec, 0.1)


                # Set orientation (if you have orientation data)
                rotation_matrix, _ = cv2.Rodrigues(self.rvec)
                rot_mat_4x4 = tf.transformations.identity_matrix()
                rot_mat_4x4[:3, :3] = rotation_matrix
                ori_in_quat = tf.transformations.quaternion_from_matrix(rot_mat_4x4)


                # Broadcast transform
                self.tf_broadcaster.sendTransform((self.tvec[0], self.tvec[1], self.tvec[2]-self._OFFSET),
                                                  ori_in_quat,
                                                  rospy.Time.now(),
                                                  "aruco_marker",
                                                  "head_camera")


        except CvBridgeError as err:
            rospy.logerr(err)
            return

        # Imshow for debugging purposes
        cv2.imshow('Test_saywer_head_Cam', cv_image)
        cv2.waitKey(3)


def main():

    ArUcoMarker()
    rospy.on_shutdown(ArUcoMarker.clean_shutdown)
    rospy.spin()


if __name__ == '__main__':
    main()
