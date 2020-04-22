#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

geometry_msgs::TransformStamped current_pose;
geometry_msgs::TransformStamped Left_Controller;

double x_position;
double y_position;
double z_position;
double roll;
double pitch;
double yaw;
double w;
    
    
void recvPose(const geometry_msgs::TransformStampedConstPtr& msg){
    geometry_msgs::TransformStamped current_pose(*msg);


    /*current_pose.transform.translation.x = x_position;                               // Left_Controller.transform.translation.x;
    current_pose.transform.translation.y = y_position;
    current_pose.transform.translation.z = z_position;*/
    
    x_position = current_pose.transform.translation.x;                             
    y_position = current_pose.transform.translation.y;
    z_position = current_pose.transform.translation.z;
    
    
    
    roll = current_pose.transform.rotation.x;                                         // Left_Controller.transform.rotation.x;
    pitch = current_pose.transform.rotation.y;
    yaw = current_pose.transform.rotation.z;
    w = current_pose.transform.rotation.w;

}

void timerCallback(const ros::TimerEvent& event){
    
    static tf::TransformBroadcaster broadcaster;
      
    tf::StampedTransform transform;
    transform.frame_id_ = "map";
    transform.child_frame_id_ = "waist";
    transform.stamp_ = event.current_real;
    transform.setOrigin(tf::Vector3(x_position,y_position,z_position));
    transform.setRotation(tf::createQuaternionFromRPY(roll,pitch,yaw));
    
    broadcaster.sendTransform(transform);
    
}

int main(int argc, char** argv){
    
    ros::init(argc,argv,"human_sim2");
    ros::NodeHandle nh;
    
    ros::Subscriber pose_sub = nh.subscribe("/Waist",1,recvPose);
    
    ros::Timer timer = nh.createTimer(ros::Duration(0.0001), timerCallback);
   
    ros::spin();
}



