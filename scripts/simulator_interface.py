#!/usr/bin/env python

import rospy

from math import atan, atan2, cos, sin, sqrt, pi
import numpy as np
import time

from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Vector3, PointStamped, PoseStamped
from std_msgs.msg import String
from simple_robot_driving.msg import SimpleRobotDriveMsg, ThreeAngles
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from nav_msgs.srv import GetPlan, GetPlanResponse
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion


# message = "0"
scan_angle = 270
scan_step = 3
scan_angle_rad = np.deg2rad(scan_angle)
scan_step_rad = np.deg2rad(scan_step)
scan_average = 1
laser_frequency = 1
scan_range_min = 0.001
scan_range_max = 1.5


def get_dist_ang_from_point(orig_x: float, orig_y: float,
                            orig_th: float, dest_x: float, dest_y: float) -> SimpleRobotDriveMsg:
    """ Get angle(rad) and distance(m) needed to get from source to destination. """
    delta_x = dest_x - orig_x
    delta_y = dest_y - orig_y

    if delta_x > 0.0:
        goal_th = atan(delta_y / delta_x)
    elif delta_x < 0.0:
        goal_th = atan(delta_y / delta_x) + pi
    elif delta_y >= 0.0:                        # delta_x is 0, delta_y is positive
        goal_th = pi * 0.5
    else:                                       # delta_x is 0, delta_y is negative
        goal_th = -pi * 0.5

    print("\tdelta_x:", delta_x, "\tdelta_y:", delta_y, "\tgoal_th:", goal_th)

    # if delta_x == 0.0:
    #     goal_th = -pi / 2                       # TODO maybe without minus
    # else:
    #     goal_th = -atan(delta_y / delta_x)      # TODO maybe without minus
    
    new_data = SimpleRobotDriveMsg()
    new_data.distance = sqrt(delta_x**2 + delta_y**2)
    new_data.rotation = goal_th - orig_th       # TODO
    while new_data.rotation > pi:
        new_data.rotation -= 2.0 * pi
    while new_data.rotation < -pi:
        new_data.rotation += 2.0 * pi

    print("\tProposed goal:", new_data.distance, new_data.rotation)

    return new_data


def str2float(s: str) -> float:
    return float(s)


def filter_scan_data(data, delta):
    """ Filter the scan data """
    pass


def get_difference_between_angles(a1: float = 0.0, a2: float = 0.0) -> float:
    """ Calculate the difference between two angles """
    a1x = cos(a1)
    a1y = sin(a1)
    a2x = cos(a2)
    a2y = sin(a2)

    dx = a1x - a2x
    dy = a1y - a2y

    da = atan2(dx, dy)

    # print("Difference between angles", a1, "and", a2, "is", da, ".")

    return da


def get_time_from_dist(dist: float) -> float:
    """ Given the driving distance, calculate the necessary robot driving time. """
    dist1 = abs(dist)
    if dist1 == 0.0:
        return 0.0
    elif dist1 < 0.0576:
        time = -48.241 * pow(dist1, 2) + 10.035 * dist1 + 0.0727
    else:
        time = 2.4104 * dist1 + 0.3518
    
    if dist < 0:
        time *= -1
    
    time *= 1.00    # Artificially increase the value, due to the battery depletion

    return time


def rotate_point(point, angle):
    """ Rotate the point by the given angle (in degrees) and return the new point coordinates. """
    angle_rad = np.deg2rad(angle)
    new_point = (point[0] * cos(angle_rad) - point[1] * sin(angle_rad), point[0] * sin(angle_rad) + point[1] * cos(angle_rad))
    return new_point


def get_distance(len1: float, len2: float, angle=3) -> float:
    """ Return the eucledean distance between two measured points. 
        Here, point is a measured distance and not actually a point with 2D coordinates.
    """
    point1 = (len1, 0)
    point2 = rotate_point((len2, 0), angle)

    return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


def get_max_neighbour_distance(scan_step, max_alpha):
    """ Calculate the maximum value fraction between the neighbouring points, based on the scan_step and max_alpha values. """
    return np.sin(np.abs(scan_step)) / np.sin(pi - np.abs(scan_step) - max_alpha)


# Function that removes all points that are potentionally the result of ghosting from the sensor.
def remove_ghosting_from_points(distances, scan_step: float):
    max_alpha = np.deg2rad(30)
    max_distance_nearest_neighbour = get_max_neighbour_distance(scan_step, max_alpha)
    # print("max_distance_nearest_neighbour", max_distance_nearest_neighbour)
    
    # Every other point
    for i in range(1, len(distances) - 1):
        nearest_neighbour = min(get_distance(distances[i], distances[i-1]), get_distance(distances[i], distances[i+1]))
        if nearest_neighbour > distances[i] * max_distance_nearest_neighbour:
            distances[i] = 0.0
    # First point
    if get_distance(distances[0], distances[1]) > distances[0] * max_distance_nearest_neighbour:
        distances[0] = 0.0
    # Last point
    if get_distance(distances[-1], distances[-2]) > distances[-1] * max_distance_nearest_neighbour:
        distances[-1] = 0.0

    return distances


class driver:
    def __init__(self):
        rospy.init_node('sim_interface', anonymous=True)

        rospy.Subscriber('/dist_ang', SimpleRobotDriveMsg, self.get_dist_ang)
        rospy.Subscriber('/clicked_point', PointStamped, self.get_clicked_point)
        # rospy.Subscriber('/sim_odom', Odometry, self.get_sim_odom)
        rospy.Subscriber('/sim_scan', LaserScan, self.get_sim_scan)
        rospy.Subscriber('/sim_angles', ThreeAngles, self.get_sim_angles)
        self.scan_pub = rospy.Publisher('scan', LaserScan, queue_size=50)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
        self.drive_pub = rospy.Publisher('make_drive', SimpleRobotDriveMsg, queue_size=50)
        self.make_scan_pub = rospy.Publisher('make_scan', String, queue_size=10)
        # self.path_pub = rospy.Publisher('path_plan', GetPlanResponse, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        self.x = rospy.get_param('initial_pos_x')
        self.y = rospy.get_param('initial_pos_y')
        self.th = rospy.get_param('initial_pos_a')
        self.current_distance = 0

        self.can_get_goal = False
        # self.move_speed = 0.274

        time.sleep(0.5)

        self.make_scan_pub.publish("PUB")

        rospy.spin()
        #self.start_publishing_odom()
        pass


    def get_sim_scan(self, data) -> None:
        global scan_step_rad

        timestamp = rospy.Time.now()

        distances = np.array(data.ranges)

        distances = remove_ghosting_from_points(distances, scan_step_rad)    # TODO??? Maybe comment out?

        self.publish_odometry(timestamp)
        self.publish_laser_scan(timestamp, distances)

        self.can_get_goal = True
        pass


    def get_sim_angles(self, data) -> None:
        starting_angle = data.first
        middle_angle = data.second
        ending_angle = data.third

        # print("\tStarting angle:", starting_angle, "----", "Middle angle:", middle_angle, "----", "Ending angle:", ending_angle)

        avg_move_th = (middle_angle + ending_angle) / 2.0       # Normalize the angles (carteasians to normalize the coordinates, atan2(x, y))

        delta_x = self.current_distance * cos(avg_move_th)
        delta_y = self.current_distance * sin(avg_move_th)

        self.x += delta_x
        self.y += delta_y
        self.th += (ending_angle - starting_angle)     # Normalize the angles

        # Send a request to perform a scan
        self.make_scan_pub.publish("PUB")
        pass
    

    def get_dist_ang(self, data) -> None:
        self.can_get_goal = False
        self.current_distance = data.distance

        self.publish_drive(data)
        pass
        

    def publish_drive(self, data) -> None:
        dist_ang = SimpleRobotDriveMsg()
        dist_ang.distance = data.distance
        dist_ang.rotation = data.rotation

        self.drive_pub.publish(dist_ang)
        pass


    def get_path(self, data) -> Path:
        rospy.wait_for_service('/move_base/make_plan')
        try:
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            print("\tTranslation:", trans, "\tRotation:", rot)

            # Robot pose:
            robot_pose = PoseStamped()
            robot_pose.header.frame_id = 'map'
            robot_pose.pose.position.x = trans[0]
            robot_pose.pose.position.y = trans[1]
            robot_pose.pose.orientation.z = rot[2]
            robot_pose.pose.orientation.w = rot[3]
            print("robot_pose:", robot_pose.pose.position)

            # Goal pose:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = data.point.x
            goal_pose.pose.position.y = data.point.y
            goal_pose.pose.orientation.z = rot[2]
            goal_pose.pose.orientation.w = rot[3]
            # print("goal_pose:", goal_pose.pose.position)

            # Tolerance:
            tolerance = 1.0

            get_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
            path = get_plan(robot_pose, goal_pose, tolerance)
            #print("path:", path)

            return path

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return Path()


    def get_temp_goal_from_path(self, path) -> Point:
        max_travel = 0.3
        max_rotate = 1.0

        temp_goal = Point()
        (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
        temp_goal.x = trans[0]
        temp_goal.y = trans[1]

        robot_th = euler_from_quaternion(rot)[2]
        print("Robot's orientation:",robot_th)

        for pose in path.plan.poses:
            delta_th = get_difference_between_angles(robot_th, pose.pose.orientation.z)
            while delta_th > 2 * pi:
                delta_th -= 2 * pi
            delta_dist = sqrt((trans[0] - pose.pose.position.x)**2 + (trans[1] - pose.pose.position.y)**2)

            if delta_dist > max_travel:# or delta_th > max_rotate:
                return temp_goal
            
            temp_goal.x = pose.pose.position.x
            temp_goal.y = pose.pose.position.y

        return temp_goal


    def get_clicked_point(self, data) -> None:
        new_data = get_dist_ang_from_point(self.x, self.y, self.th, data.point.x, data.point.y)
        self.get_dist_ang(new_data)


    def start_publishing_odom(self) -> None:
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()


    def publish_laser_scan(self, timestamp, distances) -> None:
        # global scan_angle_rad
        # global scan_step_rad
        # global laser_frequency
        # global scan_range_min
        # global scan_range_max

        distances = remove_ghosting_from_points(distances, scan_step_rad)

        num_readings = len(distances)

        scan = LaserScan()
        scan.header.stamp = timestamp
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = -scan_angle_rad / 2.0
        scan.angle_max = scan_angle_rad / 2.0
        scan.angle_increment = scan_step_rad
        scan.time_increment = 0.00001
        scan.range_min = scan_range_min
        scan.range_max = scan_range_max

        scan.ranges = []
        scan.intensities = []
        for d in distances:
            scan.ranges.append(d)

        self.scan_pub.publish(scan)


    def publish_odometry(self, timestamp) -> None:
        odom_quat = quaternion_from_euler(0, 0, self.th)
        
        # Publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            timestamp,
            "base_link",
            "odom"
        )

        self.odom_broadcaster.sendTransform(
            (0, 0, 0),
            quaternion_from_euler(0, 0, 0),
            timestamp,
            "laser_frame",
            "base_link"
        )

        # Publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = "odom"

        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

        self.odom_pub.publish(odom)


def main():
    message = "0"

    try:
        driver()
    except rospy.ROSInterruptException:
        print("\n")
        pass


if __name__ == '__main__':
    main()
