#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "string"
#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"
#include "math.h"
#include "vins_fusion_extra/EulerAngle.h"

ros::Subscriber sub;
ros::Publisher pub;
ros::Publisher RPY_pub;
ros::Timer pub_timer;
ros::Timer RPY_timer;

bool data_ready = false;
bool pub_RPY = true;
geometry_msgs::PoseStamped cur_pose;

std::string sub_topic;
std::string pub_topic;
int rate;

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg_p){
    data_ready = true;
    cur_pose.pose = msg_p->pose.pose;
}

void pub_timer_cb(const ros::TimerEvent &event){
    if (data_ready == false) return;

    geometry_msgs::PoseStamped pose_tf = cur_pose;
    pose_tf.header.frame_id = "ground";
    pose_tf.header.stamp = ros::Time::now();

    // 修改位置
    double x = pose_tf.pose.position.y;
    double y = -pose_tf.pose.position.x;
    pose_tf.pose.position.x = x;
    pose_tf.pose.position.y = y;

    // 修改四元数
    double yaw, pitch, roll;
    tf2::getEulerYPR(pose_tf.pose.orientation, yaw, pitch, roll);

    tf2::Quaternion qtn;
    qtn.setRPY(pitch, -(roll+1.5707963267949), yaw);
    pose_tf.pose.orientation.x = qtn.getX();
    pose_tf.pose.orientation.y = qtn.getY();
    pose_tf.pose.orientation.z = qtn.getZ();
    pose_tf.pose.orientation.w = qtn.getW();

    tf2::getEulerYPR(pose_tf.pose.orientation, yaw, pitch, roll);

    // 发送转换后的pose
    pub.publish(pose_tf);

    // 发送RPY
    vins_fusion_extra::EulerAngle ea;
    ea.yaw = yaw;
    ea.pitch = pitch;
    ea.roll = roll;

    RPY_pub.publish(ea);
}

void RPY_timer_cb(const ros::TimerEvent &event){

}

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"listener");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // param
    private_nh.param("sub_odom_topic", sub_topic, std::string("/vins_estimator/camera_pose")); 
    private_nh.param("pub_pose_topic", pub_topic, std::string("mavros/vision_pose/pose")); 
    private_nh.param("pub_rate", rate, 20); 
    private_nh.param("pub_RPY", pub_RPY, true); 

    sub = nh.subscribe<nav_msgs::Odometry>(sub_topic.c_str(), 10, &odom_cb);
    pub = nh.advertise<geometry_msgs::PoseStamped>(pub_topic.c_str(), 10);
    RPY_pub = nh.advertise<vins_fusion_extra::EulerAngle>("RPY", 10);

    pub_timer = nh.createTimer(ros::Duration(1.0/rate), &pub_timer_cb);

    ros::spin();

    return 0;
}