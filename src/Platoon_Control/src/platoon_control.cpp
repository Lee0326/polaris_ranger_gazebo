#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include "nav_msgs/Odometry.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;

class PlatoonWrapper
{
public:
    PlatoonWrapper(ros::NodeHandle &nh)
    {
        driveCmdPublish1 = nh.advertise<std_msgs::Float64>("/platoon/1/drive_cmd",1);
        driveCmdPublish2 = nh.advertise<std_msgs::Float64>("/platoon/2/drive_cmd",1);
        driveCmdPublish3 = nh.advertise<std_msgs::Float64>("/platoon/3/drive_cmd",1);
    }
    void callback(const nav_msgs::Odometry::ConstPtr& msg1, const nav_msgs::Odometry::ConstPtr& msg2, const nav_msgs::Odometry::ConstPtr& msg3);

private:
    ros::Publisher driveCmdPublish1;
    ros::Publisher driveCmdPublish2;
    ros::Publisher driveCmdPublish3;
    std_msgs::Float64 driveCmd1;
    std_msgs::Float64 driveCmd2;
    std_msgs::Float64 driveCmd3;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "platoon_controller");
    ros::NodeHandle nh;
    PlatoonWrapper pltController(nh);
    string v1_odom_topic = "vehicle1/state";
    string v2_odom_topic = "vehicle2/state";
    string v3_odom_topic = "vehicle3/state";
    message_filters::Subscriber<nav_msgs::Odometry> vehicle1_sub(nh, v1_odom_topic, 10);
    message_filters::Subscriber<nav_msgs::Odometry> vehicle2_sub(nh, v2_odom_topic, 10);
    message_filters::Subscriber<nav_msgs::Odometry> vehicle3_sub(nh, v3_odom_topic, 10);
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry, nav_msgs::Odometry> sync_policy;
    message_filters::Synchronizer<sync_policy> sync(sync_policy(10),vehicle1_sub,vehicle2_sub,vehicle3_sub);
    sync.registerCallback(boost::bind(&PlatoonWrapper::callback,pltController, _1, _2,_3));

    ros::spin();

    return 0;
}

void PlatoonWrapper::callback(const nav_msgs::Odometry::ConstPtr& msg1, const nav_msgs::Odometry::ConstPtr& msg2, const nav_msgs::Odometry::ConstPtr& msg3)
{
    //TODO use the states to determine control signals
/**




**/
    cout<<"publishing cmd"<<endl;
    driveCmd1.data = 10.0;
    driveCmd2.data = 10.0;
    driveCmd3.data = 10.0;
    driveCmdPublish1.publish(driveCmd1);
    driveCmdPublish2.publish(driveCmd2);
    driveCmdPublish3.publish(driveCmd3);
}