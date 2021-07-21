#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <time.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <motion/take_off.h>
#include <motion/global_current_pos.h>
#include <tuple>

using namespace std;

struct coordinates
{
   float x;
   float y;
};

// We create a simple callback which will save the current state of the autopilot 
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_position; // for comparing current position(local) to wanted goal location(local)
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_position = *msg;
}

geometry_msgs::PoseStamped goal_pose; // for setting wanted goal location (local coordinates) 
void go_to(){
    
    goal_pose.pose.position.x = 0;
    goal_pose.pose.position.y = 0;
    goal_pose.pose.position.z = 2;
}

coordinates local_to_global_coords(string ID, float in_x, float in_y, int direction){
    // direction controlls whether the coordinates are converted from local to global or from global to local, should either be 1 or -1.
    
    float out_x, out_y;

    if (ID == "0"){
        out_x = in_x + 15*direction;
        out_y = in_y;
    }
    else if (ID == "1"){
        out_x = in_x - 15*direction;
        out_y = in_y;
    }
    else if (ID == "2"){
        out_x = in_x;
        out_y = in_y + 15*direction;
    }
    else if (ID == "3"){
        out_x = in_x;
        out_y = in_y - 15*direction;
    }

    coordinates out_coords = {out_x, out_y};
    return out_coords;
}

int main(int argc, char **argv)
{
    srand (time(NULL));
    string ID = argv[1];
    ros::init(argc, argv, "uav" + ID);
    ros::NodeHandle nh;
    string uav = "uav" + ID;
    


    /* We instantiate a publisher to publish the commanded local position and the appropriate clients to request 
    arming and mode change. Note that for your own system, the "mavros" prefix might be different as it will 
    depend on the name given to the node in it's launch file. */
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(uav + "/mavros/state", 10, state_cb);
    // ros::Subscriber local_pos = nh.subscribe<geometry_msgs::PoseStamped>(uav + "/mavros/local_position/pose", 10, boost::bind(position_cb,_1, ID)); // this allowes the callback to take one more argument
    ros::Subscriber local_pos = nh.subscribe<geometry_msgs::PoseStamped>(uav + "/mavros/local_position/pose", 10, position_cb);
    ros::Publisher local_goal_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(uav + "/mavros/setpoint_position/local", 10);
    ros::Publisher global_goal_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(uav + "/mavros/setpoint_position/global", 10);
    ros::Publisher took_off_pub = nh.advertise<motion::take_off>(uav + "/motion/take_off_topic", 10);
    ros::Publisher global_current_pos_pub = nh.advertise<motion::global_current_pos>(uav + "/motion/global_current_pos_topic", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(uav + "/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(uav + "/mavros/set_mode");

    /* PX4 has a timeout of 500ms between two Offboard commands. If this timeout is exceeded, the commander will fall 
    back to the last mode the vehicle was in before entering Offboard mode. This is why the publishing rate must 
    be faster than 2 Hz to also account for possible latencies. This is also the same reason why it is recommended 
    to enter Offboard mode from Position mode, this way if the vehicle drops out of Offboard mode it will stop in 
    its tracks and hover. */
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    /* Before publishing anything, we wait for the connection to be established between MAVROS and the autopilot. This 
    loop should exit as soon as a heartbeat message is received. */
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    /* Even though the PX4 Pro Flight Stack operates in the aerospace NED coordinate frame, MAVROS translates these 
    coordinates to the standard ENU frame and vice-versa. This is why we set z to positive 2. */
    go_to();

    /* Before entering Offboard mode, you must have already started streaming setpoints. Otherwise the mode switch will 
    be rejected. Here, 100 was chosen as an arbitrary amount. */
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_goal_pos_pub.publish(goal_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    //We set the custom mode to OFFBOARD
    offb_set_mode.request.custom_mode = "OFFBOARD";

    /* The rest of the code is pretty self explanatory. We attempt to switch to Offboard mode, after which we arm the quad 
    to allow it to fly. We space out the service calls by 5 seconds so to not flood the autopilot with the requests. In 
    the same loop, we continue sending the requested pose at the appropriate rate. */
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){ //check if mode is not offboard
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){ // call set_mode
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){ // check if drone is not armed
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){ // call  cmd/arming topic
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_goal_pos_pub.publish(goal_pose);

        geometry_msgs::PoseStamped global_goal_pose; // for ploting goal positions at plot.py
        coordinates global_goal = local_to_global_coords(ID, goal_pose.pose.position.x, goal_pose.pose.position.y, 1);
        global_goal_pose.pose.position.x = global_goal.x;
        global_goal_pose.pose.position.y = global_goal.y;
        global_goal_pos_pub.publish(global_goal_pose);
        // ROS_INFO("uav%s %f %f", ID, global_goal_pose.pose.position.x, global_goal_pose.pose.position.y);
        // ROS_INFO("%f\n%f\n%f\n", global_goal_pose.pose.position.x, global_goal_pose.pose.position.y, goal_pose.pose.position.z);

        motion::take_off take_off_flag;
        motion::global_current_pos global_current_pose;
        // take_off_flag.took_off = false;
        if(abs(goal_pose.pose.position.x - current_position.pose.position.x)<0.1){
            if(abs(goal_pose.pose.position.y - current_position.pose.position.y)<0.1){
                if(abs(goal_pose.pose.position.z - current_position.pose.position.z)<0.1){ //if drone reaches goal take_off position
                    //go_to(ID);
                    take_off_flag.took_off = true;
                    took_off_pub.publish(take_off_flag);
                    
                    coordinates global_current = local_to_global_coords(ID, current_position.pose.position.x, current_position.pose.position.y, 1);
                    global_current_pose.position.x = global_current.x;
                    global_current_pose.position.y = global_current.y;
                    global_current_pos_pub.publish(global_current_pose);
                    // goal_pose.pose.position.x = current_position.pose.position.x;
                    // goal_pose.pose.position.y = current_position.pose.position.y;
                    // goal_pose.pose.position.z = current_position.pose.position.z;
                } 
            } 
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}