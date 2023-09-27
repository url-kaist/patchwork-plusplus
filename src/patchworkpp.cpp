#include <iostream>
// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "patchworkpp/patchworkpp.hpp"
#include <signal.h>
#include <pcl_conversions/pcl_conversions.h>


using PointType = pcl::PointXYZI;
using namespace std;

namespace patchworkpp {
    using PatchworkppPointXYZI = PatchWorkpp<pcl::PointXYZI>;
}

RCLCPP_COMPONENTS_REGISTER_NODE(patchworkpp::PatchworkppPointXYZI)


