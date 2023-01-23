from multiprocessing import dummy
import rospy
import rosbag
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import sin, cos, pi
import tf
import numpy as np
import time


message = "0"
scan_angle = 270
scan_step = 3
scan_average = 1
laser_frequency = 1
scan_range_min = 0.001
scan_range_max = 1.5


odom_broadcaster = tf.TransformBroadcaster()
x = 0.0
y = 0.0
th = 0.0


class frame:
    timestamp = time.time()
    x = 0.0
    y = 0.0
    th = 0.0
    distances = []


def str2float(str):
    return float(str)


def get_max_neighbour_distance(scan_step, max_alpha):
    return np.sin(np.abs(scan_step)) / np.sin(pi - np.abs(scan_step) - max_alpha)


def rotate_point(point, angle):
    angle_rad = np.deg2rad(angle)
    new_point = (point[0] * cos(angle_rad) - point[1] * sin(angle_rad), point[0] * sin(angle_rad) + point[1] * cos(angle_rad))
    return new_point


def get_distance(len1, len2, angle=3):
    point1 = (len1, 0)
    point2 = rotate_point((len2, 0), angle)

    return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


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


def publish_from_file(filename):
    global scan_pub, odom_pub, odom_broadcaster

    global scan_step
    data = frame()
    last_timestamp = rospy.Time.now().to_sec()
    with open(filename) as f:
        line = f.readline()
        scan_step = np.deg2rad(int(line))
        try:
            while line:
                # Timestamp
                line = f.readline()
                if not line:
                    break
                
                seconds = int(line)

                #data.timestamp = rospy.Time(secs=seconds)
                data.timestamp = rospy.Time.now()

                # Position
                line = f.readline()
                vals = line.strip().split('\t')
                data.x = float(vals[0])
                data.y = -float(vals[1])
                data.th = -float(vals[2]) + pi/2

                # Laser scan
                line = f.readline()
                data.distances = remove_ghosting_from_points(np.array(list(map(str2float, line.strip().split("\t")))), np.abs(scan_step))

                for i in range(len(data.distances)):
                    data.distances[i] = data.distances[i] - 0.03

                # Publish frame
                publish_frame(data)
                print("Published to topics at time", data.timestamp)

                time_difference = data.timestamp.to_sec() - last_timestamp
                last_timestamp = data.timestamp.to_sec()
                # time.sleep(time_difference)
                time.sleep(1.0)
                # dummy_input = input()
        except rospy.ROSInterruptException:
            pass
    return


def publish_frame(data):
    scan_pub = rospy.Publisher('scan', LaserScan, queue_size=50)
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    global scan_angle, scan_step, scan_average, laser_frequency, scan_range_min, scan_range_max
    global beginning_scan_angle, ending_scan_angle
    global x, y, th
    global bag
    x = data.x
    y = data.y
    th = data.th
    print("\tNew x:", x, "----", "New y:", y, "----", "New th:", th)
    timestamp = rospy.Time.now()
    # timestamp = data.timestamp

    # PUBLISH ODOMETRY
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # Publish the transform over tf
    
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        timestamp,
        "base_link",
        "odom"
    )

    # Publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = timestamp
    odom.header.frame_id = "odom"

    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

    odom_pub.publish(odom)

    # PUBLISH LASER SCAN
    distances = data.distances
    num_readings = distances.size

    scan = LaserScan()
    scan.header.stamp = timestamp
    scan.header.frame_id = 'laser_frame'
    scan.angle_min = np.deg2rad(135)
    scan.angle_max = np.deg2rad(-132)
    scan.angle_increment = -scan_step
    scan.time_increment = 0.000001
    scan.range_min = scan_range_min
    scan.range_max = scan_range_max

    scan.ranges = []
    scan.intensities = []
    for d in distances:
        scan.ranges.append(d)

    scan_pub.publish(scan)

    return


if __name__ == '__main__':
    # bag = rosbag.Bag('test.bag', 'w')

    # dummy_input = input()

    rospy.init_node('publish_captured_data', anonymous=True)
    filename = "/home/matic/Documents/Magistrska/scan_files/BigField03_Step3Average1_CORRECTED.txt"
    try:
        #while True:
        publish_from_file(filename)
    except rospy.ROSInterruptException:
        pass