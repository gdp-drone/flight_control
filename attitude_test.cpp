#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>

// Store controller board state
mavros_msgs::State current_state;

// Store altitude
mavros_msgs::Altitude infrared_altitude;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// Void function to store altitude
void altitude_cb(const mavros_msgs::Altitude::ConstPtr& msg){
    infrared_altitude = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // Subscribe to altitude node - For lidar
    ros::Subscriber altitude_sub = nh.subscribe<mavros_msgs::Altitude>
            ("/mavros/altitude", 50, altitude_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose1;
    pose1.pose.position.x = 0;
    pose1.pose.position.y = 0;
    pose1.pose.position.z = 1;

    geometry_msgs::PoseStamped pose2;
    pose2.pose.position.x = 3;
    pose2.pose.position.y = 0;
    pose2.pose.position.z = 1;

    geometry_msgs::PoseStamped pose3;
    pose3.pose.position.x = -3;
    pose3.pose.position.y = 0;
    pose3.pose.position.z = 1;

    //Send a few setpoints before starting Before entering Offboard mode, you must have already 
    //started streaming setpoints. Otherwise the mode switch will be rejected. 
    //Here, 100 was chosen as an arbitrary amount.

    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose1);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    float diff = 1;

    while(ros::ok() && diff>0.30){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose1);
        diff = pose1.pose.position.z - infrared_altitude.bottom_clearance;

        ros::spinOnce();
        rate.sleep();
    }

    // Ocillate between poses
    int counter = 0;
    while(ros::ok()){
        while(ros::ok() && counter < 100){
            local_pos_pub.publish(pose2);
            ros::spinOnce();
            rate.sleep();
            counter++;
        }

        counter = counter * -1;

        while(ros::ok() && counter < 0){
            local_pos_pub.publish(pose3);
            ros::spinOnce();
            rate.sleep();
            counter++;
        }
    }

    return 0;
}
