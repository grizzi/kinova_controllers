#! /usr/bin/env python

import numpy as np
import rospy
import tf2_ros
import pinocchio as pin
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from mpl_toolkits.mplot3d import Axes3D


class ValveTrajectoryGenerator(object):
    """
    Generates a trajectory for a manipulator to open a valve.
    The convention is that the grasp pose is with the z axis pointing the same as the
    the axis of rotation of the valve, and the x axis pointing inward, towards the valve
    center
    """
    def __init__(self,
                 valve_radius=0.1,
                 end_effector_frame="ee",
                 reference_frame="arm_base_link"):

        self.reference_frame = reference_frame
        self.end_effector_frame = end_effector_frame

        self.valve_radius = valve_radius
        self.valve_origin = None
        self.valve_axis = None

        self.theta = 0.0
        self.theta_dot = 0.1  # rad/s
        self.theta_dot_max = 1.0
        self.theta_desired = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_ref_valve = None
        self.tf_ref_valve_bc = tf2_ros.StaticTransformBroadcaster()

        self.prev_ee_pose = self.get_end_effector_pose()

        self.target_pose_pub = rospy.Publisher("/target_pose", PoseStamped, queue_size=1)

    @staticmethod
    def project_to_plane(plane_origin, plane_normal, p, in_plane=False):
        """
        Takes a non normalized plane normal, its origin, a point in 3d space and returns the closest point
        to p in the plane
        :param plane_origin:
        :param plane_normal:
        :param p:
        :param in_plane: set to True to only return the point vector in the plane frame
        :return:
        """
        p = np.asarray(p)
        plane_origin = np.asarray(plane_origin)
        plane_normal = np.asarray(plane_normal)
        plane_normal = plane_normal / np.linalg.norm(plane_normal)
        tangent_vector = (p - plane_origin) - np.dot(p - plane_origin, plane_normal) * plane_normal
        if in_plane:
            return tangent_vector
        else:
            return plane_origin + tangent_vector

    @staticmethod
    def se3_to_pose_ros(se3pose):
        pose_ros = Pose()
        pose_ros.position.x = se3pose.translation[0]
        pose_ros.position.y = se3pose.translation[1]
        pose_ros.position.z = se3pose.translation[2]
        q = R.from_dcm(se3pose.rotation).as_quat()
        pose_ros.orientation.x = q[0]
        pose_ros.orientation.y = q[1]
        pose_ros.orientation.z = q[2]
        pose_ros.orientation.w = q[3]
        return pose_ros

    @staticmethod
    def tf_to_se3(transform):
        q = pin.Quaternion(transform.transform.rotation.w,
                           transform.transform.rotation.x,
                           transform.transform.rotation.y,
                           transform.transform.rotation.z)
        t = np.array([transform.transform.translation.x,
                      transform.transform.translation.y,
                      transform.transform.translation.z])
        return pin.SE3(q, t)

    @staticmethod
    def pose_to_se3(pose):
        q = pin.Quaternion(pose.orientation.w,
                           pose.orientation.x,
                           pose.orientation.y,
                           pose.orientation.z)
        t = np.array([pose.position.x,
                      pose.position.y,
                      pose.position.z])
        return pin.SE3(q, t)

    def reset_grasp_pose(self):
        """
        The grasp pose is located on the perimeter. This is set to the current end effector
        pose. The orientation is the same as the valve orientation
        """
        tf_ref_ee = self.get_end_effector_pose()
        tf_valve_ee = pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([-self.valve_radius, 0.0, 0.0]))
        self.tf_ref_valve = tf_ref_ee.act(tf_valve_ee.inverse())

        # reset tracking variables
        self.valve_origin = self.tf_ref_valve.translation
        self.valve_axis = self.tf_ref_valve.rotation[:, 2]
        self.theta = 0.0
        self.theta_desired = 0.0

        tf_ref_valve_ros = TransformStamped()

        tf_ref_valve_ros.header.stamp = rospy.Time.now()
        tf_ref_valve_ros.header.frame_id = self.reference_frame
        tf_ref_valve_ros.child_frame_id = "valve_est_from_grasp"

        tf_ref_valve_ros.transform.translation.x = self.tf_ref_valve.translation[0]
        tf_ref_valve_ros.transform.translation.y = self.tf_ref_valve.translation[1]
        tf_ref_valve_ros.transform.translation.z = self.tf_ref_valve.translation[2]
        q = R.from_dcm(self.tf_ref_valve.rotation).as_quat()
        tf_ref_valve_ros.transform.rotation.x = q[0]
        tf_ref_valve_ros.transform.rotation.y = q[1]
        tf_ref_valve_ros.transform.rotation.z = q[2]
        tf_ref_valve_ros.transform.rotation.w = q[3]

        self.tf_ref_valve_bc.sendTransform(tf_ref_valve_ros)

    def estimate_rotation_angle(self):
        """
        Estimate what is the current angle given the pose change in the end effector
        and assuming a stable grasp. The two consecutive positions of the end
        effector are projected in the valve plane. Their cartesian norm
        tells what is the angle between them which is also how much the valve moved
        (assumption of stable grasp)
        """

        current_pose = self.get_end_effector_pose()
        curr_p_in_plane = self.project_to_plane(plane_origin=self.valve_origin,
                                                plane_normal=self.valve_axis,
                                                p=current_pose.translation,
                                                in_plane=True)
        prev_p_in_plane = self.project_to_plane(plane_origin=self.valve_origin,
                                                plane_normal=self.valve_axis,
                                                p=self.prev_ee_pose.translation,
                                                in_plane=True)

        curr_p_in_plane_normalized = curr_p_in_plane / np.linalg.norm(curr_p_in_plane)
        prev_p_in_plane_normalized = prev_p_in_plane / np.linalg.norm(prev_p_in_plane)
        theta_increment = np.arccos(np.dot(curr_p_in_plane_normalized, prev_p_in_plane_normalized))
        self.theta += theta_increment
        self.prev_ee_pose = current_pose

    def advance(self, dt):
        """
        Increment theta according to current velocity and step size and computes the new
        target pose for the end effector
        :param dt:
        :return:
        """
        self.theta_desired += self.theta_dot * dt
        rospy.loginfo_throttle(1.0, "New theta desired is {}".format(self.theta_desired))

        q = R.from_euler('xyz', [0.0, 0.0, - self.theta_desired], degrees=False).as_quat()
        t = np.array([-self.valve_radius*np.cos(self.theta_desired),
                      self.valve_radius*np.sin(self.theta_desired),
                      0.0])
        tf_valve_eedes = pin.SE3(pin.Quaternion(q[3], q[0], q[1], q[2]), t)
        tf_ref_eedes = self.tf_ref_valve.act(tf_valve_eedes)

        target_pose = PoseStamped()
        target_pose.pose = self.se3_to_pose_ros(tf_ref_eedes)
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.get_rostime()
        return target_pose

    def adapt_velocity(self):
        pass
        # error = self.theta_desired - self.theta
        # self.theta_dot = np.min([self.theta_dot_max, 1.0/error])

    def get_end_effector_pose(self):
        """ Retrieve the end effector pose. Let it fail if unable to get the transform """
        transform = self.tf_buffer.lookup_transform(self.reference_frame,            # target frame
                                                    self.end_effector_frame,         # source frame
                                                    rospy.Time(0),                   # tf at first available time
                                                    rospy.Duration(3))
        return self.tf_to_se3(transform)

    def run(self, dt, theta_max):
        rate = rospy.Rate(1/dt)
        self.reset_grasp_pose()
        while self.theta_desired < theta_max and not rospy.is_shutdown():

            # adapt velocity based on the current tracking error
            self.adapt_velocity()

            # extract new target pose in valve frame
            target_pose = self.advance(dt)

            # publish to controller
            self.target_pose_pub.publish(target_pose)
            rate.sleep()
        return True


if __name__ == "__main__":
    rospy.init_node("trajectory_generation_test")

    rospy.loginfo("Sleeping 2.0 sec before starting")
    rospy.sleep(2.0)
    valve_radius = rospy.get_param("~valve_radius", 0.1)
    end_effector_frame = rospy.get_param("~end_effector_frame", "end_effector")
    reference_frame = rospy.get_param("~reference_frame", "arm_base_link")
    dt = rospy.get_param("~dt", 0.1)
    target_angle_rad = rospy.get_param("~max_velocity_rad_s", 1.0)

    trajectory_generator = ValveTrajectoryGenerator(valve_radius,
                                                    end_effector_frame,
                                                    reference_frame)
    trajectory_generator.run(dt=dt, theta_max=target_angle_rad)

    rospy.spin()
