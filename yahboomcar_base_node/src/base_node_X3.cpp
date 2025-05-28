#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class OdomPublisher:public rclcpp ::Node
{
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
   //rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
   std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
   double linear_scale_x_ = 0.0 ;
   double linear_scale_y_ = 0.0;
   double vel_dt_ = 0.0;
   double x_pos_ = 0.0;
   double y_pos_ = 0.0;
   double heading_ = 0.0;
   double linear_velocity_x_ = 0.0;
   double linear_velocity_y_ = 0.0;
   double angular_velocity_z_ = 0.0;
   double wheelbase_ = 0.25;
   bool pub_odom_tf_ = false;
   rclcpp::Time last_vel_time_  ;
   std::string odom_frame = "odom";
   std::string base_footprint_frame = "base_footprint";
	public:
	  OdomPublisher()
	  : Node("base_node")
	  {            
            this->declare_parameter<double>("wheelbase",0.25);
            this->declare_parameter<std::string>("odom_frame","odom");
            this->declare_parameter<std::string>("base_footprint_frame","base_footprint"); 
            this->declare_parameter<double>("linear_scale_x",1.0);
            this->declare_parameter<double>("linear_scale_y",1.0);
            this->declare_parameter<bool>("pub_odom_tf",false);

            this->get_parameter<double>("linear_scale_x",linear_scale_x_);
            this->get_parameter<double>("linear_scale_y",linear_scale_y_);
            this->get_parameter<double>("wheelbase",wheelbase_);
            this->get_parameter<bool>("pub_odom_tf",pub_odom_tf_);
            this->get_parameter<std::string>("odom_frame",odom_frame);
            this->get_parameter<std::string>("base_footprint_frame",base_footprint_frame);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

	  	subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("odom_raw",50,std::bind(&OdomPublisher::handle_vel,this,_1));
	  	
	  	}
	  	private:
	  	  void handle_vel(const std::shared_ptr<nav_msgs::msg::Odometry > msg)
	  	  {
                       
	  	  	rclcpp::Time curren_time = rclcpp::Clock().now();

			vel_dt_ = (curren_time - last_vel_time_).seconds();

    		last_vel_time_ = curren_time;


            
            
                geometry_msgs::msg::TransformStamped t;
                rclcpp::Time now = this->get_clock()->now();
                t.header.stamp = now;
                t.header.frame_id = odom_frame;
                t.child_frame_id = base_footprint_frame;
                t.transform.translation.x = msg->pose.pose.position.x;
                t.transform.translation.y = msg->pose.pose.position.y;
                t.transform.translation.z = msg->pose.pose.position.z;
                
                t.transform.rotation.x = msg->pose.pose.orientation.x;
                t.transform.rotation.y = msg->pose.pose.orientation.y;
                t.transform.rotation.z = msg->pose.pose.orientation.z;
                t.transform.rotation.w = msg->pose.pose.orientation.w;
                
                tf_broadcaster_->sendTransform(t);
                //std::cout<<"pos.x: "<<msg->pose.pose.position.x<<std::endl;
                  
            
		  	  }

};


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OdomPublisher>());
	rclcpp::shutdown();
    return 0;
}

