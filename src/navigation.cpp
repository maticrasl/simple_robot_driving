#include "ros/ros.h"
#include "std_msgs/String.h"
//#include <tf/transform_listener.h>
//#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Polygon.h>
#include <vector>
#include <cmath>

//#include <sstream>

// ~~~~~~~~~~ CLASSES DEFINITIONS: ~~~~~~~~~~~
#pragma region CLASSES_DEFINITIONS

class Point {
    private:
    double x, y;

    public:
    Point(const double X, const double Y) : x(X), y(Y) {}
    Point(const Point p) : x(p.x), y(p.y) {}

    void set_x(double X) {x = X;}
    double get_x() {return x;}
    void set_y(double Y) {y = Y;}
    double get_y() {return y;}
    void set(double X, double Y) {
        x = X;
        y = Y;
    }
    double get_distance(Point p) {
        return std::sqrt(std::pow(x - p.x, 2) + std::pow(y - p.y, 2));
    }
    void operator += (Point p) {
        x += p.x;
        y += p.y;
    }
    void operator -= (Point p) {
        x -= p.x;
        y -= p.y;
    }
};

#pragma endregion

// ~~~~~~~~~~ FUNCTION DEFINITIONS: ~~~~~~~~~~
#pragma region FUNCTION_DEFINITIONS

bool IsPathClear(Point a, Point b) {
    Point d = b - a;                    // Vector from point a to point b
    int sx = (d.x > 0.0 ? 1 : -1);        // Set the directions
    int sy = (d.y > 0.0 ? 1 : -1);        // Set the directions
    if(d.x == 0 && d.y == 0) {          // If the goal is the same point as the source
        if()                            
    }
}


#pragma endregion

// ~~~~~~~~~~ GLOBAL VARIABLES: ~~~~~~~~~~~~~~
#pragma region GLOBAL_VARIABLES



#pragma endregion

// ~~~~~~~~~~ FUNCITONS: ~~~~~~~~~~~~~~~~~~~~~
#pragma region FUNCTIONS



#pragma endregion

int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation");
    ros::NodeHandle node;

    //tf::TransformListener tf_listener(ros::Duration(10));
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    std::string costmap_name = "my_costmap";
    costmap_2d::Costmap2DROS costmap("my_costmap", tfBuffer);
    costmap.loadOldParameters(node);
    costmap.start();

    geometry_msgs::Polygon polygon = new geometry_msgs::Polygon();

    ros::Rate rate(10.0);
    while(node.ok()) {
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer.lookupTransform("base_link", "ground_truth", ros::Time(0));
            std::cout << transformStamped.transform.translation.x << "\t" << transformStamped.transform.translation.y << "\n";
        }
        catch (tf2::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        rate.sleep();
    }
    costmap.stop();

    //ros::spin();

    return 0;
}

/*
To implement the A* algorithm and finding of the next scan position, take a closer look
into the following functions in the C# program:
 - public Point GetNextScanLocation( ... )  (1005 - 1048)
 - public Point getNextMovePoint( ... )     (868 - 970)
 - public bool isClearPath( ... )           (798 - 865)
*/