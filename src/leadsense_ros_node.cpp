#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "leadsense_ros_node");

    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;

    nodelet.load(ros::this_node::getName(),
            "leadsense_ros/leadsense_ros_nodelet",
            remap, nargv);

    ros::spin();

    return 0;
}
