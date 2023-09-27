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

std::shared_ptr<PatchWorkpp<PointType>> PatchworkppGroundSeg;

int main(int argc, char**argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    cout << "Operating patchwork++..." << endl;
    PatchworkppGroundSeg.reset(new PatchWorkpp<PointType>(options));
    rclcpp::spin(PatchworkppGroundSeg);
    rclcpp::shutdown();

    return 0;
}
