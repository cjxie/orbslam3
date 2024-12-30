#include "stereo_inertial_node.hpp"

int main(int argc, char** argv)
{
    bool bEqual = false;
    if(argc < 4)
    {
        cout<< "Usage: ros2 run ORB_SLAM3 Stereo_Inertial path_to_vocabulary path_to_settings do_rectify [do_equalize]" << endl;
        rclcpp::shutdown();
        return 1;
    }
    
    std::string sbRect(argv[3]);
    bool bRect = sbRect == "true" ? true : false;
    cout<< "Rectify images: " << bRect << endl;
    if(argc>=5)
    {
    std::string sbEqual(argv[4]);
    if(sbEqual == "true")
        bEqual = true;
    } 

    rclcpp::init(argc, argv);

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO, true);
    auto node = std::make_shared<StereoInertialNode>(&SLAM, bRect, bEqual);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}