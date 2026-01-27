#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class oopsquare : public rclcpp::Node
{
public:
    oopsquare() : rclcpp::Node("oopsquare")
    {
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 50, std::bind(&oopsquare::odomCb, this, std::placeholders::_1));

        timer_ = create_wall_timer(20ms, std::bind(&oopsquare::square, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    static double wrapToPi(double a)
    {
        while (a > M_PI)
            a -= 2.0 * M_PI;
        while (a < -M_PI)
            a += 2.0 * M_PI;
        return a;
    }

    static double yawFromQuat(const geometry_msgs::msg::Quaternion &qmsg)
    {
        tf2::Quaternion q;
        tf2::fromMsg(qmsg, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;
        theta = yawFromQuat(msg->pose.pose.orientation);

        if (!init)
        {

            yaw0 = theta;
            start_x = x;
            start_y = y;
            init = true;
            //   RCLCPP_INFO(get_logger(), "Initialized: yaw0=%.3f", yaw0_);
        }
    }

    void square()
    {
        if (!init)
            return;

        static const double DIRS[4] = {0.0, M_PI / 2.0, -M_PI, -M_PI / 2.0};

        geometry_msgs::msg::Twist cmd;

        if (rotate)
        {
            const double desired = yaw0 + DIRS[direction];
            double error = wrapToPi(desired - theta);

            if (std::fabs(error) > yaw_tol)
            {
                cmd.linear.x = 0.0;
                cmd.angular.z = (error > 0.0) ? angular_speed : -angular_speed;
            }
            else
            {
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                rotate = false;
                start_x = x;
                start_y = y;
            }
        }
        else
        {
            double distance = std::hypot(x - start_x, y - start_y);
            if (distance < (side_length - dist_tol))
            {
                cmd.linear.x = linear_speed;
                cmd.angular.z = 0;
            }
            else
            {
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                rotate = true;
                direction = (direction + 1) % 4;
                edges_done++;

                if (edges_done >= 4)
                {
                    done = true;
                    RCLCPP_INFO(get_logger(), "Square complete");
                }
                else
                {
                    RCLCPP_INFO(get_logger(), "Edge complete; preparing to rotate to dir %d.", direction);
                }
            }
        }
        if (done)
        {
            cmd.linear.x = 0;
            cmd.angular.z = 0;
        }

        cmd_pub_->publish(cmd);
    }

    double side_length{1.0};
    double linear_speed{1};
    double angular_speed{1};
    double dist_tol{0.01};
    double yaw_tol{0.02};

    bool init{false};
    bool rotate{true}; // align to dis[0]
    bool done{false};
    int direction{0};
    int edges_done{0};

    double x{0.0}, y{0.0}, theta{0.0};
    double start_x{0.0}, start_y{0.0};
    double yaw0{0.0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<oopsquare>());
    rclcpp::shutdown();
    return 0;
}

// ```

// ---

// ### Step 2: Build File Updates

// You just need to edit your `CMakeLists.txt` to add this new node. **You do not need to add any new dependencies** to `package.xml` because this node only uses `rclcpp` and `nav_msgs`, which your `moverobo` node already uses.

// **In `CMakeLists.txt`:**

// Add this block, right next to the block for your `moverobo` executable:

// ```cmake
// #-- - Add this for the new human_detector_node -- -
// add_executable(human_detector_node src/human_detector_node.cpp)

// ament_target_dependencies(human_detector_node
//   rclcpp
//   nav_msgs
//   geometry_msgs
// )

// install(TARGETS
//   human_detector_node
//   DESTINATION lib/${PROJECT_NAME}
// )
// ```
// *(Note: I added `geometry_msgs` as a dependency, which is good practice. Make sure `find_package(geometry_msgs REQUIRED)` is at the top of your `CMakeLists.txt`, which it should be from `moverobo`.)*

// ---

// ### Step 3: How to Run Everything

// This is very important. This new node **only watches**. It cannot find the moved human unless the robot *moves* and *scans* the relevant areas.

// You need to run **three** things at once:

// 1.  **Terminal 1 (Simulation):**
//     ```bash
//     ros2 launch gazeboenvs tb4_warehouse.launch.py use_rviz:=true
