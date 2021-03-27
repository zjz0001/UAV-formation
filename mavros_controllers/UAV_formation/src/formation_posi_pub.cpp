#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

float d=1;
geometry_msgs::PoseStamped uav1_goal;
//geometry_msgs::PoseStamped uav2_goal;


geometry_msgs::PoseStamped uav1_relapose(geometry_msgs::PoseStamped uav0_now)
{
  
    uav1_goal.pose.position.x = uav0_now.pose.position.x + d;
    uav1_goal.pose.position.y = uav0_now.pose.position.y;
    uav1_goal.pose.position.z = uav0_now.pose.position.z;
  
    //used for debugging
    //ROS_INFO("uav1_pose: x:%0.6f, y:%0.6f", uav1_goal.pose.position.x, uav1_goal.pose.position.y);
  
    return uav1_goal;
}

//geometry_msgs::PoseStamped uav2_relapose(geometry_msgs::PoseStamped uav0_now)
//{
  
  //  uav2_goal.pose.position.x = uav0_now.pose.position.x - d;
   // uav2_goal.pose.position.y = uav0_now.pose.position.y;
    //uav2_goal.pose.position.z = uav0_now.pose.position.z;
  
    //used for debugging
    //ROS_INFO("uav1_pose: x:%0.6f, y:%0.6f", uav1_goal.x, uav1_goal.y);
  
    //return uav2_goal;
//}

void leaderPosiCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  
  uav1_goal = uav1_relapose(*msg);
  //uav2_goal = uav2_relapose(*msg);
  
  //used for debugging
  //ROS_INFO("uav0_pose: x:%0.6f, y:%0.6f", (*msg).pose.position.x, (*msg).pose.position.y);


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation_posi_pub");
    
    //UAV1
    ros::NodeHandle nh1;
  
    ros::Subscriber state_sub1 = nh1.subscribe<mavros_msgs::State>
            ("uav1/mavros/state", 10, state_cb);
    ros::Publisher posepub1= nh1.advertise<geometry_msgs::PoseStamped>
            ("/uav1/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client1 = nh1.serviceClient<mavros_msgs::CommandBool>
            ("/uav1/mavros/cmd/arming");
    ros::ServiceClient set_mode_client1 = nh1.serviceClient<mavros_msgs::SetMode>
            ("/uav1/mavros/set_mode");

    //uav2
   // ros::NodeHandle nh2;
    
    //ros::Subscriber state_sub2 = nh2.subscribe<mavros_msgs::State>
            //("node2/mavros/state", 10, state_cb);
    //ros::Publisher posepub2 = nh2.advertise<geometry_msgs::PoseStamped>
            //("/node2/mavros/setpoint_position/local", 10);
    //ros::ServiceClient arming_client2 = nh2.serviceClient<mavros_msgs::CommandBool>
            //("/node2/mavros/cmd/arming");
    //ros::ServiceClient set_mode_client2 = nh2.serviceClient<mavros_msgs::SetMode>
            //("/node2/mavros/set_mode");

    //subscribe UAV0
    ros::Subscriber leader_sub = nh1.subscribe("/uav0/mavros/local_position/pose", 10, leaderPosiCallback);



    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);



    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

   while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client1.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client1.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

       //publish the pose_goal of uav1 (how to control the rate of publishing)
       posepub1.publish(uav1_goal);
       //posepub2.publish(uav2_goal);
        ROS_INFO("uav1");
       ROS_INFO("uav1_pose: x:%0.6f, y:%0.6f", uav1_goal.pose.position.x, uav1_goal.pose.position.y);
        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}
