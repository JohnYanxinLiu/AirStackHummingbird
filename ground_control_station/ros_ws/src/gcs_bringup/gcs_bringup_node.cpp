#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <gcs_bringup/gcs_bringup.h>
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GcsBringup>());
    return 0;
}

