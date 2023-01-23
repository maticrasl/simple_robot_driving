#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Vector3, PointStamped
from simple_robot_driving.msg import SimpleRobotDriveMsg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import sin, cos, atan, sqrt, pi
import tf
import socket
import numpy as np


# message = "0"
scan_angle = 270
scan_step = 3
scan_angle_rad = np.deg2rad(scan_angle)
scan_step_rad = np.deg2rad(scan_step)
scan_average = 1
laser_frequency = 1
scan_range_min = 0.001
scan_range_max = 1.5


def get_dist_ang_from_point(orig_x, orig_y, orig_th, dest_x, dest_y):       # TODO check coordinates here!
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


def str2float(str):
    return float(str)


# Function for filtering the scan data
def filter_scan_data(data, delta):
    return


# Calculate the robot's driving time from a given distance.
def get_time_from_dist(dist):
    dist1 = abs(dist)
    if dist1 == 0.0:
        return 0.0
    elif dist1 < 0.0576:
        time = -48.241 * pow(dist1, 2) + 10.035 * dist1 + 0.0727
    else:
        time = 2.4104 * dist1 + 0.3518
    
    if dist < 0:
        time *= -1
    
    time *= 1.00                    # Artificially increase the value, due to the battery depletion

    return time


# Rotates the point by the given angle (in degrees) and returns the new point coordinates.
def rotate_point(point, angle):
    angle_rad = np.deg2rad(angle)
    new_point = (point[0] * cos(angle_rad) - point[1] * sin(angle_rad), point[0] * sin(angle_rad) + point[1] * cos(angle_rad))
    return new_point


# Returns the eucledean distance between two points.
def get_distance(len1, len2, angle=3):
    point1 = (len1, 0)
    point2 = rotate_point((len2, 0), angle)

    return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


# Function to calculate the maximum value fraction between the neighbouring points, based on the scan_step and max_alpha values.
def get_max_neighbour_distance(scan_step, max_alpha):
    return np.sin(np.abs(scan_step)) / np.sin(pi - np.abs(scan_step) - max_alpha)


# Function that removes all points that are potentionally the result of ghosting from the sensor.
def remove_ghosting_from_points(distances, scan_step):
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
        rospy.init_node('robot_interface', anonymous=True)
        rospy.Subscriber('/cmd_vel', Twist, self.get_cmd_vel)
        rospy.Subscriber('/dist_ang', SimpleRobotDriveMsg, self.get_dist_ang)
        rospy.Subscriber('/clicked_point', PointStamped, self.get_clicked_point)
        self.scan_pub = rospy.Publisher('scan', LaserScan, queue_size=50)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()
        # self.tf_listener = tf.TransformListener()

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.start_publishing_odom()


    def get_cmd_vel(self, data):
        linear = data.linear.x
        rotation = data.angular.z
        self.send_cmd_vel_to_arduino(linear, rotation)


    def send_cmd_vel_to_arduino(self, linear, rotation):
        global message
        message = "m{},{},*".format(linear, rotation)
        print("\tLinear", linear, "----", "Angular", rotation)
        # client.sendto(message.encode(), (TCP_IP, TCP_PORT))


    def get_dist_ang(self, data):                           # TODO check for coordinates here!
        rotation = data.rotation
        distance = data.distance
        time = get_time_from_dist(distance)

        data_odom = self.send_time_rot_to_arduino(time, rotation)
            
        # Wait for recieved data, publish it to /odom and send the scan command to the robot.
        #while True:
        # data, address = sock.recvfrom(1024)
        recv_message = data_odom.strip("* \t\n\0").split(",")
        starting_angle = float(recv_message[0])
        middle_angle = float(recv_message[1])
        ending_angle = float(recv_message[2])
        print("\tStarting angle:", starting_angle, "----", "Middle angle:", middle_angle, "----", "Ending angle:", ending_angle)

        data_scan = self.send_scan_to_arduino()

        # Upon recieving the scan data, publish it to the /scan
        recv_message = data_scan.strip("* \t\n\0")

        timestamp = rospy.Time.now()

        self.publish_odometry(distance, starting_angle, middle_angle, ending_angle, timestamp)
        self.publish_laser_scan(recv_message, timestamp)


    def send_time_rot_to_arduino(self, time, rotation):
        global message
        global TCP_IP
        global TCP_PORT

        message = "d{},{},*".format(time, rotation)
        print("\tTime", time, "----", "Rotation", rotation)
        # client.connect((TCP_IP, TCP_PORT))
        # client.send(message.encode())
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.connect((TCP_IP, TCP_PORT))
        client.send(message.encode())
        # print("Sending command to arduino ...")
        data = client.recv(4096).decode('utf-8')
        # print("Received data ...")
        client.close()
        # print("Closed client ...")
        # print("\t", data)
        return data


    def send_scan_to_arduino(self):
        global message
        global scan_angle
        global scan_step
        global scan_average
        global TCP_IP
        global TCP_PORT
        
        message = "s{},{},{},*".format(scan_angle, scan_step, scan_average)
        # client.connect((TCP_IP, TCP_PORT))
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.connect((TCP_IP, TCP_PORT))
        client.send(message.encode())
        data = client.recv(4096).decode('utf-8')
        client.close()
        # print(data)
        return data


    def get_clicked_point(self, data):
        new_data = get_dist_ang_from_point(self.x, self.y, self.th, data.point.x, data.point.y)
        self.get_dist_ang(new_data)


    def start_publishing_odom(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            timestamp = rospy.Time.now()
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
            
            # Publish the transform over tf
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0.),
                odom_quat,
                timestamp,
                "base_link",
                "odom"
            )

            # Publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = timestamp
            odom.header.frame_id = "odom"

            odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

            self.odom_pub.publish(odom)

            r.sleep()


    def publish_laser_scan(self, data, timestamp):
        global scan_angle_rad
        global scan_step_rad
        global laser_frequency
        global scan_range_min
        global scan_range_max

        distances = np.array(list(map(str2float, data.split("\t")))) / 1000.0

        distances = remove_ghosting_from_points(distances, scan_step_rad)

        num_readings = distances.size

        # print("\tnum_readings:", num_readings)

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
        return


    def publish_odometry(self, dist, starting_angle, middle_angle, ending_angle, timestamp):        # TODO check coordinates here!
        # delta_th = ending_angle - self.th
        avg_move_th = (middle_angle + ending_angle) / 2.0

        delta_x = dist * cos(avg_move_th)
        delta_y = dist * sin(avg_move_th)

        self.x += delta_x
        self.y += delta_y
        self.th = ending_angle

        print("\tNew x:", self.x, "----", "New y:", self.y, "----", "New th:", self.th)

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        
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
            tf.transformations.quaternion_from_euler(0, 0, 0),
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

        return


if __name__ == '__main__':
    TCP_IP = "192.168.208.39"
    #TCP_IP = "192.168.0.143"
    #UDP_IP = "192.168.5.13"
    TCP_PORT = 8888
    BUFFER_SIZE = 512
    # client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    message = "0"

    try:
        driver()
    except rospy.ROSInterruptException:
        print("\n")
        pass
