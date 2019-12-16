#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "tf/tf.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MyNode
{
public:

  MoveBaseClient* ac_;
  double last_x_set = 0; double last_y_set = 0; double last_th_set = 0;
  void doStuff(const double& x_set, const double& y_set, const double& th_set)
  {

      //MoveBaseClient ac("move_base", true);
      ac_ = new MoveBaseClient("move_base",true);
       //wait for the action server to come up
      while(!ac_->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
      }

      move_base_msgs::MoveBaseGoal goal;
      if(x_set != last_x_set || y_set != last_y_set || th_set != last_th_set)
      {
          //we'll send a goal to the robot to move 1 meter forward
          goal.target_pose.header.frame_id = "base_link";
          goal.target_pose.header.stamp = ros::Time::now();

          goal.target_pose.pose.position.x = x_set;
          goal.target_pose.pose.position.y = y_set;
          goal.target_pose.pose.position.z = 0;

          tf::Quaternion q;
          q.setRotation(tf::Vector3(0, 0, 1), th_set);
          goal.target_pose.pose.orientation.x = q.x();
          goal.target_pose.pose.orientation.y = q.y();
          goal.target_pose.pose.orientation.z = q.z();
          goal.target_pose.pose.orientation.w = q.w();

          ROS_INFO("Sending goal");
         // ac.sendGoal(goal);
          ac_->sendGoal(goal,
                          boost::bind(&MyNode::doneCb, this, _1, _2),
                          MoveBaseClient::SimpleActiveCallback(),
                          MoveBaseClient::SimpleFeedbackCallback());
      }
      last_x_set = x_set; last_y_set = y_set; last_th_set = th_set;
  }

  void doneCb(const actionlib::SimpleClientGoalState& state,
                             const move_base_msgs::MoveBaseResultConstPtr& result)
  {
      ROS_INFO("RESULT %s",state.toString().c_str());
     // boost::unique_lock<boost::mutex> lock(wp_mutex_);
      switch (state.state_) {
      case actionlib::SimpleClientGoalState::ABORTED:
      {
        ROS_INFO("NavigationManager::moveBaseResultCallback: ABORTED");
      }
        break;
      case actionlib::SimpleClientGoalState::SUCCEEDED:
      {
        ROS_INFO("NavigationManager::moveBaseResultCallback: SUCCEEDED");
      }
        break;
      default:
        break;
      }
   }
   void pose_get(void)
   {
       if(listener_.waitForTransform("/map","/base_link",ros::Time(0),ros::Duration(0.2)))
       {
         listener_.lookupTransform("/map","/base_link",ros::Time(0),world_pose);
         geometry_msgs::Pose pose;
         pose.position.x = world_pose.getOrigin().getX();
         pose.position.y = world_pose.getOrigin().getY();
         pose.position.z = world_pose.getOrigin().getZ();
         pose.orientation.x = world_pose.getRotation().getX();
         pose.orientation.y = world_pose.getRotation().getY();
         pose.orientation.z = world_pose.getRotation().getZ();
         pose.orientation.w = world_pose.getRotation().getW();
         ROS_INFO("position.x : %f",pose.position.x);
         ROS_INFO("position.y : %f",pose.position.y);
         ROS_INFO("position.z : %f",pose.position.z);
         ROS_INFO("orientation.x : %f",pose.orientation.x);
         ROS_INFO("orientation.y : %f",pose.orientation.y);
         ROS_INFO("orientation.z : %f",pose.orientation.z);
         ROS_INFO("orientation.w : %f",pose.orientation.w);
       }
   }

private:
  tf::StampedTransform world_pose;
  tf::TransformListener listener_;
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci_class_client");
  MyNode my_node;
  //ros::Rate loop_rate(10);
  while(ros::ok())
  {
      my_node.doStuff(1,0,1);
      my_node.pose_get();
     // ros::spin();
      ros::spinOnce();
     // loop_rate.sleep();
  }
  return 0;
}
