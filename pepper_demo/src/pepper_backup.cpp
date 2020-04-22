#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <std_msgs/Float64MultiArray.h>

geometry_msgs::TransformStamped current_pose;
//geometry_msgs::TransformStamped Left_Controller;
ros::Publisher pub_markers;

double z_position_shoulder = 1.5;
double y_position_shoulder = -0.1;
double x_position_shoulder = -0.2;
double x_position_Right_Elbow;
double y_position_Right_Elbow;
double z_position_Right_Elbow;
double x_position_Right_Controller;
double y_position_Right_Controller;
double z_position_Right_Controller;
double roll_Right_Elbow;
double pitch_Right_Elbow;
double yaw_Right_Elbow;
double roll_Right_Controller;
double pitch_Right_Controller;
double yaw_Right_Controller;
//double w_Right_Elbow;
//double w_Right_Controller;
double angle_pitch_shoulder;
double angle_roll_shoulder;
double angle_pitch_elbow;
double angle_roll_elbow;

    
void recvPose_Right_Elbow(const geometry_msgs::TransformStampedConstPtr& msg){
    geometry_msgs::TransformStamped current_pose(*msg);


    x_position_Right_Elbow = current_pose.transform.translation.x;                             
    y_position_Right_Elbow = current_pose.transform.translation.y;
    z_position_Right_Elbow = current_pose.transform.translation.z;
    
    //roll_Right_Elbow = current_pose.transform.rotation.x;                                     
    //pitch_Right_Elbow = current_pose.transform.rotation.y;
    //yaw_Right_Elbow = current_pose.transform.rotation.z;
    //w_Right_Elbow = current_pose.transform.rotation.w;

}
void recvPose_Right_Controller(const geometry_msgs::TransformStampedConstPtr& msg){
    geometry_msgs::TransformStamped current_pose(*msg);


    x_position_Right_Controller = current_pose.transform.translation.x;                             
    y_position_Right_Controller = current_pose.transform.translation.y;
    z_position_Right_Controller = current_pose.transform.translation.z;
    
    //roll_Right_Controller = current_pose.transform.rotation.x;                                     
    //pitch_Right_Controller = current_pose.transform.rotation.y;
    //yaw_Right_Controller = current_pose.transform.rotation.z;
    //w_Right_Controller = current_pose.transform.rotation.w;
    
}
void timerCallback(const ros::TimerEvent& event){
    
    /*if (x_position_Right_Controller > x_position_shoulder && x_position_Right_Elbow > x_position_shoulder){
    angle_pitch = atan((z_position_Right_Elbow - z_position_shoulder)/(x_position_Right_Elbow - x_position_shoulder));
    angle_roll = atan((y_position_shoulder - y_position_Right_Elbow)/(x_position_Right_Elbow - x_position_shoulder));
    }
    else{
    angle_pitch = 0;
    angle_roll = 0;
    }
     if (x_position_Right_Elbow > x_position_shoulder){
    angle_roll = atan((y_position_shoulder - y_position_Right_Elbow)/(x_position_Right_Elbow - x_position_shoulder));
    }
    else{
    angle_roll = 0;}*/
    
    angle_pitch_shoulder = atan((z_position_Right_Elbow - z_position_shoulder)/(x_position_Right_Elbow - x_position_shoulder));
    angle_roll_shoulder = atan((y_position_shoulder - y_position_Right_Elbow)/(x_position_Right_Elbow - x_position_shoulder));
    
    std_msgs::Float64MultiArray tf_elbow_position1;
    tf_elbow_position1.data.resize(3);
    tf_elbow_position1.data[0]= cos(angle_pitch_shoulder)*x_position_Right_Elbow + 0*y_position_Right_Elbow + sin(angle_pitch_shoulder)*z_position_Right_Elbow;
    tf_elbow_position1.data[1]= 0*x_position_Right_Elbow + 1*x_position_Right_Elbow + 0*x_position_Right_Elbow;
    tf_elbow_position1.data[2]= -sin(angle_pitch_shoulder)*x_position_Right_Elbow + 0*x_position_Right_Elbow + cos(angle_pitch_shoulder)*x_position_Right_Elbow;
   
    std_msgs::Float64MultiArray tf_elbow_position2;
    tf_elbow_position2.data.resize(3);
    tf_elbow_position2.data[0]= 1*tf_elbow_position1.data[0] + 0*tf_elbow_position1.data[1] + 0*tf_elbow_position1.data[2];
    tf_elbow_position2.data[1]= 0*tf_elbow_position1.data[0] + cos(angle_roll_shoulder)*tf_elbow_position1.data[1] - sin(angle_roll_shoulder)*tf_elbow_position1.data[2];
    tf_elbow_position2.data[2]= 0*tf_elbow_position1.data[0] - sin(angle_roll_shoulder)*tf_elbow_position1.data[1] + cos(angle_roll_shoulder)*tf_elbow_position1.data[2];
    
    angle_pitch_elbow = atan((z_position_Right_Controller - tf_elbow_position2.data[2])/(x_position_Right_Controller - tf_elbow_position2.data[0]));
    angle_roll_elbow = atan((tf_elbow_position2.data[1] - y_position_Right_Controller)/(x_position_Right_Controller - tf_elbow_position2.data[0]));
    
    //ROS_INFO("angle: (%lf, %lf)", angle_pitch, angle_roll);
    
    
    

    static tf::TransformBroadcaster broadcaster;
      
    tf::StampedTransform transform;
    transform.frame_id_ = "map";
    transform.child_frame_id_ = "shoulder_position";
    transform.stamp_ = event.current_real;
    transform.setOrigin(tf::Vector3(-0.2,-0.1,1.5));
    transform.setRotation(tf::createQuaternionFromRPY(angle_roll_shoulder,angle_pitch_shoulder,0));
    
    tf::StampedTransform transform_left_shoulder;
    transform_left_shoulder.frame_id_ = "shoulder_position";
    transform_left_shoulder.child_frame_id_ = "elbow_position";
    transform_left_shoulder.stamp_ = event.current_real;
    transform_left_shoulder.setOrigin(tf::Vector3(0,0,0.6));
    transform_left_shoulder.setRotation(tf::createQuaternionFromRPY(angle_roll_elbow,angle_pitch_elbow,0));
    
    broadcaster.sendTransform(transform);
    broadcaster.sendTransform(transform_left_shoulder);
    
    
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(2);
    
    visualization_msgs::Marker MarkerArray;
    MarkerArray.header.frame_id = "shoulder_position";
  
    MarkerArray.type = visualization_msgs::Marker::CYLINDER;
    MarkerArray.header.stamp = event.current_real;
     
    MarkerArray.color.r = 0.0;
    MarkerArray.color.g = 1.0;
    MarkerArray.color.b = 0.0;
    MarkerArray.color.a = 1.0;
    
    MarkerArray.scale.x = 0.1;
    MarkerArray.scale.y = 0.1;
    MarkerArray.scale.z = 0.6;
   
    MarkerArray.pose.position.x = 0;                                                               //current_pose.transform.translation.x;
    MarkerArray.pose.position.y = 0;
    MarkerArray.pose.position.z = 0.3;
    
     
    /*MarkerArray.pose.orientation.x = 0;                                                            //current_pose.transform.rotation.x;
    MarkerArray.pose.orientation.y = 0;
    MarkerArray.pose.orientation.z = 0;
    MarkerArray.pose.orientation.w = 1;*/
    MarkerArray.id = 0;
    
    marker_array.markers[0] = MarkerArray;
    
    visualization_msgs::Marker MarkerArray1;
    MarkerArray1.header.frame_id = "elbow_position";
    MarkerArray1.type = visualization_msgs::Marker::CYLINDER;
    MarkerArray1.header.stamp = event.current_real;
     
    MarkerArray1.color.r = 0.0;
    MarkerArray1.color.g = 1.0;
    MarkerArray1.color.b = 0.0;
    MarkerArray1.color.a = 1.0;
    
    MarkerArray1.scale.x = 0.1;
    MarkerArray1.scale.y = 0.1;
    MarkerArray1.scale.z = 0.6;
   
    MarkerArray1.pose.position.x = 0;
    MarkerArray1.pose.position.y = 0;
    MarkerArray1.pose.position.z = 0.3;
    
    /*MarkerArray1.pose.orientation.x = 0;                                                            //current_pose.transform.rotation.x;
    MarkerArray1.pose.orientation.y = 0;
    MarkerArray1.pose.orientation.z = 0;
    MarkerArray1.pose.orientation.w = 1;*/
    MarkerArray1.id = 1;
    
    marker_array.markers[1] = MarkerArray1;
    
    
    pub_markers.publish(marker_array);
}

int main(int argc, char** argv){
    
    ros::init(argc,argv,"pepper_demo");
    ros::NodeHandle nh;
    
    
    ros::Subscriber pose_sub_Right_Elbow = nh.subscribe("/Right_Elbow",1,recvPose_Right_Elbow);
    ros::Subscriber pose_sub_Right_Controller = nh.subscribe("/Right_Controller",1,recvPose_Right_Controller);
    
    
    ros::Timer timer = nh.createTimer(ros::Duration(0.01), timerCallback);
    pub_markers = nh.advertise<visualization_msgs::MarkerArray>("markers",1);
   
    ros::spin();
}





