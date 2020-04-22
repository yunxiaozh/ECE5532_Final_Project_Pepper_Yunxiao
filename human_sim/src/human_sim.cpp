#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

geometry_msgs::TransformStamped current_pose;
geometry_msgs::TransformStamped Left_Controller;
//ros::Publisher pub_markers;
   
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
    transform.child_frame_id_ = "Left_Controller";
    transform.stamp_ = event.current_real;
    transform.setOrigin(tf::Vector3(x_position,y_position,z_position));
    transform.setRotation(tf::createQuaternionFromRPY(roll,pitch,yaw));
    
    broadcaster.sendTransform(transform);
    
    
   
    
    /*current_pose.transform.translation.x = 1;
    current_pose.transform.translation.y = 1;
    current_pose.transform.translation.z = 1;
    
     geometry_msgs::TransformStamped current_pose;
     geometry_msgs::TransformStamped Left_Controller;
    
    current_pose.transform.rotation.x = 1;
    current_pose.transform.rotation.y = 1;
    current_pose.transform.rotation.z = 1;
    current_pose.transform.rotation.w = 1;*/
    
    

    
    /*visualization_msgs::MarkerArray marker_array;
    
    marker_array.markers.resize(2);
    
    visualization_msgs::Marker MarkerArray;
    MarkerArray.header.frame_id = "map";
  
    MarkerArray.type = visualization_msgs::Marker::SPHERE;
    MarkerArray.header.stamp = event.current_real;
     
    MarkerArray.color.r = 0.0;
    MarkerArray.color.g = 1.0;
    MarkerArray.color.b = 0.0;
    MarkerArray.color.a = 1.0;
    
    MarkerArray.scale.x = 0.2;
    MarkerArray.scale.y = 0.2;
    MarkerArray.scale.z = 0.2;
   
    MarkerArray.pose.position.x = 0;                                                               //current_pose.transform.translation.x;
    MarkerArray.pose.position.y = 0;
    MarkerArray.pose.position.z = 1.65;
    
     
    MarkerArray.pose.orientation.x = 0;                                                            //current_pose.transform.rotation.x;
    MarkerArray.pose.orientation.y = 0;
    MarkerArray.pose.orientation.z = 0;
    MarkerArray.pose.orientation.w = 0;
    MarkerArray.id = 0;
    
    marker_array.markers[0] = MarkerArray;
    
    MarkerArray.type = visualization_msgs::Marker::CYLINDER;
    MarkerArray.header.stamp = event.current_real;
     
    MarkerArray.color.r = 0.0;
    MarkerArray.color.g = 1.0;
    MarkerArray.color.b = 0.0;
    MarkerArray.color.a = 1.0;
    
    MarkerArray.scale.x = 0.1;
    MarkerArray.scale.y = 0.1;
    MarkerArray.scale.z = 0.05;
   
    MarkerArray.pose.position.x = 0;
    MarkerArray.pose.position.y = 0;
    MarkerArray.pose.position.z = 1.55;
    MarkerArray.id = 1;
    
    marker_array.markers[1] = MarkerArray;
    
    
    MarkerArray.type = visualization_msgs::Marker::CUBE;
    MarkerArray.header.stamp = event.current_real;
     
    MarkerArray.color.r = 0.0;
    MarkerArray.color.g = 1.0;
    MarkerArray.color.b = 0.0;
    MarkerArray.color.a = 1.0;
    
    MarkerArray.scale.x = 0.2;
    MarkerArray.scale.y = 0.2;
    MarkerArray.scale.z = 0.2;
   
    MarkerArray.pose.position.x = 0;                                                               //current_pose.transform.translation.x;
    MarkerArray.pose.position.y = 0.25;
    MarkerArray.pose.position.z = 1.5;
    
    MarkerArray.id = 2;
    
    marker_array.markers[2] = MarkerArray;
    
    pub_markers.publish(marker_array);*/
    
}


int main(int argc, char** argv){
    
    ros::init(argc,argv,"human_sim");
    ros::NodeHandle nh;
    
    ros::Subscriber pose_sub = nh.subscribe("/Left_Controller",1,recvPose);
    
    ros::Timer timer = nh.createTimer(ros::Duration(0.0001), timerCallback);
    
    //pub_markers = nh.advertise<visualization_msgs::MarkerArray>("markers",1);
   
    ros::spin();
}



