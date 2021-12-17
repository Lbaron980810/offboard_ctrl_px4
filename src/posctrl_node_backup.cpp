#include <time.h>
#include <ros/ros.h>
#include <mavros_msgs/PosRosctrlSetpoint.h>

using namespace std;
mavros_msgs::PosRosctrlSetpoint data;
int main(int argc, char **argv){
    ros::init(argc, argv, "ss");
    ros::NodeHandle nh;

    data.Posx = 1.1;
    data.Posy = 1.1;
    data.Posz = 1.1;

    ros::Publisher ctrldatapub;
    ctrldatapub = nh.advertise<mavros_msgs::PosRosctrlSetpoint>("/mavros/pos_rosctrl_setpoint/pos_rosctrl_setpoint",10);

    ros::Rate rate(20.0);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        ctrldatapub.publish(data);
    }
    return 0;
}


