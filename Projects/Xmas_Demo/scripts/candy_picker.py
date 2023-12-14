import numpy as np

import rospy
import tf
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped

import intera_interface
from intera_interface import CHECK_VERSION
from intera_motion_msgs.msg import TrajectoryOptions
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)

class CandyPicker():
    """
    Class for generating the candy picker node.
    This class will handle the controller for the robot arm using 
    the pose of the area marker.

    """

    _PICK_OFFSET = 0.22

    def __init__(self, limb) -> None:

        

        # Initialise interfaces
        rs = intera_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state()
        self.r = rospy.Rate(5)

        rp = intera_interface.RobotParams()
        self.valid_limbs = rp.get_limb_names()
        self._limb = limb

        # Initialize Gripper
        self.gripper = self._gripper_ctrl_init()


        # Open tf listenner
        self._head = intera_interface.Head()
        self._tf_listener = tf.TransformListener()


        # Setup intera motion traj for motion generator
        self._tip_name = 'right_hand'
        self._cartersian_pose_init()

        # Setup Sawyer body navigator
        self.nav = intera_interface.Navigator()
        self.start_looking = None
        self.register = self.nav.register_callback(self.rethink_pressed, 'head_button_show')
        

    def rethink_pressed(self, data):
        
        self.start_looking  = data
        rospy.loginfo ("Button 'Rethink': {0}".format(self.nav.button_string_lookup(data)))


    def go_to_joint_angles(self, desired_js):

        if len(desired_js) != len(self.joint_angles):
            rospy.logerr('The number of joint_angles must be %d',
                         len(self.joint_angles))
            return None

        self.waypoint.set_joint_angles(joint_angles=desired_js)
        self.traj.append_waypoint(self.waypoint.to_msg())

        result = self.traj.send_trajectory()
        if result is None:
            rospy.logerr('Trajectory FAILED to send')
            return

        if result.result:
            rospy.loginfo(
                'Motion controller successfully finished the trajectory!')
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)

        self.traj.clear_waypoints()


    def go_to_Cartesian_pose(self, desired_pose):

        endpoint_state = self._limb.tip_state(self._tip_name)
        if endpoint_state is None:
            rospy.logerr('Endpoint state not found with tip name %s', self._tip_name)
            return None
        pose = endpoint_state.pose
        print(pose)

        pose.position.x = desired_pose[0][0] 
        pose.position.y = desired_pose[0][1] 
        pose.position.z = desired_pose[0][2] +  self._PICK_OFFSET

        pose.orientation.x = 0
        pose.orientation.y = 1
        pose.orientation.z = 0
        pose.orientation.w = 0

        poseStamped = PoseStamped()
        poseStamped.pose = pose

        joint_angles = self._limb.joint_ordered_angles()
        self.waypoint.set_cartesian_pose(poseStamped, self._tip_name, joint_angles)

        rospy.loginfo('Sending waypoint: \n%s', self.waypoint.to_string())
        self.traj.append_waypoint(self.waypoint.to_msg())

        result = self.traj.send_trajectory()
        if result is None:
            rospy.logerr('Trajectory FAILED to send')
            return

        if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                        result.errorId)
                
        self.traj.clear_waypoints()
        return result.result
    

    def look_at_candy_box(self):
        """
        """
        control_rate = rospy.Rate(100)
        angle = 3.14
        while (not rospy.is_shutdown() and
                not (abs(self._head.pan() - angle) <=
                    intera_interface.HEAD_PAN_ANGLE_TOLERANCE)):
            self._head.set_pan(angle, speed=0.3, timeout=0, active_cancellation=True)
            control_rate.sleep()
    

    def pick_candy(self):
        """
        Function for picking the candy.
        """

        # Wait for the detected pose of the median marker to be ready
        self._tf_listener.waitForTransform('aruco_marker', 'head_camera', rospy.Time.now(), timeout=rospy.Duration(4.0))
        candy_pose = self._tf_listener.lookupTransform('base', 'aruco_marker', rospy.Time(0))
        print(candy_pose)
        rospy.sleep(1)


        # Send tf obtained as the pose for intera motion interface
        self.go_to_Cartesian_pose(candy_pose)
        rospy.sleep(1)


        # Close Gripper for whatever it picked
        self.gripper.close()
        rospy.sleep(1)



    def hand_candy(self):
        """
        Function to hand candy
        """
        

        pass


    def run(self):

        while not rospy.is_shutdown():

            if self.start_looking:

                print('haha')
                
                self.gripper.calibrate()
                rospy.sleep(3)
                # self.look_at_candy_box()
                self.pick_candy()   
                self.hand_candy()
                break

            else: rospy.loginfo('Awaiting indicator to start!')

            self.r.sleep()

    def clean_shutdown():
        rospy.loginfo('Shutting down candy picker node!')


    def _gripper_ctrl_init(self):
        """
        Initialise the gripper controller.
        """

        gripper = None
        original_deadzone = None

        # Cleaning function (executed on shutdown)
        def clean_shutdown():
            if gripper and original_deadzone:
                gripper.set_dead_zone(original_deadzone)

        try:
            # Instantiate the gripper object
            gripper = intera_interface.Gripper(
                self.valid_limbs[0] + "_gripper")
        except (ValueError, OSError) as e:
            rospy.logerr("Could not detect an electric gripper attached to the robot.")
            clean_shutdown()
            return

        # Possible deadzone values: 0.001 - 0.002
        original_deadzone = gripper.get_dead_zone()
        gripper.set_dead_zone(0.001)

        return gripper


    def _cartersian_pose_init(self):

        try:
            self.traj_options = TrajectoryOptions()
            self.traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
            self.traj = MotionTrajectory(trajectory_options = self.traj_options, limb = self._limb)

            wpt_opts = MotionWaypointOptions(max_linear_speed = 0.3,
                                            max_linear_accel = 0.2,
                                            max_rotational_speed = 1.57,
                                            max_rotational_accel = 1.57,
                                            max_joint_speed_ratio = 1.0)
            self.waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = self._limb)

        except rospy.ROSInterruptException:
            rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.') 


    def _joint_angles_control_init(self):
            """
            """
            try:
                self.traj = MotionTrajectory(limb=self._limb)

                wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=0.2,
                                                max_joint_accel=0.04)
                self.waypoint = MotionWaypoint(
                    options=wpt_opts.to_msg(), limb=self._limb)

                self.joint_angles = self._limb.joint_ordered_angles()

                self.waypoint.set_joint_angles(joint_angles=self.joint_angles)
                self.traj.append_waypoint(self.waypoint.to_msg())

            except rospy.ROSInterruptException:
                rospy.logerr(
                    'Keyboard interrupt detected from the user. Exiting before trajectory completion.')



def main():
    
    rospy.init_node('candy_picker', anonymous=True)
    limb = intera_interface.Limb()
    merryXmas = CandyPicker(limb)
    rospy.on_shutdown(CandyPicker.clean_shutdown)
    merryXmas.run()


if __name__ == '__main__':
    main()