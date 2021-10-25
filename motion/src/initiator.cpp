#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <time.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <motion/new_point.h>
#include <motion/on_target.h>

using namespace std;

double safeDistance;
double flightHeight;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped position_local;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::CommandBool arm_cmd;
motion::new_point target_global_msg;
geometry_msgs::PoseStamped take_off_point_local;
geometry_msgs::PoseStamped take_off_point_global;
geometry_msgs::PoseStamped waypoint_local;
geometry_msgs::PoseStamped target_global;
geometry_msgs::PoseStamped target_local;
motion::new_point waypoint_global_msg;
motion::on_target on_target_msg;

bool reached_waypoint = false;
bool recieved_new_target = false;

void state_cb(const mavros_msgs::State::ConstPtr& msg){ // current state of the autopilot 
    current_state = *msg;
}


void position_local_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){ // current local position
    position_local = *msg;
}

/* ******************* */
/* FUCTIONS */
/* ******************* */


geometry_msgs::PoseStamped local_to_global_coords(string ID, geometry_msgs::PoseStamped pos, int direction){
    // direction controlls whether the coordinates are converted from local to global or from global to local, should either be 1 or -1.
    
    if (ID == "0"){
        pos.pose.position.x += 15*direction;
    }
    else if (ID == "1"){
        pos.pose.position.x -= 15*direction;
    }
    else if (ID == "2"){
        pos.pose.position.y += 15*direction;
    }
    else if (ID == "3"){
        pos.pose.position.y -= 15*direction;
    }

    return pos;
} 

bool has_reached_waypoint(geometry_msgs::Point waypoint_local_, geometry_msgs::Point position_local_){
    if(abs(waypoint_local_.x - position_local_.x)<0.1){
        if(abs(waypoint_local_.y - position_local_.y)<0.1){
            if(abs(waypoint_local_.z - position_local_.z)<0.1){ //if drone reaches goal take_off position
                return true;
            }
        }
    }
    return false;
}

bool too_close_to_target(string ID, geometry_msgs::Point waypoint_local_, geometry_msgs::Point target_local_){
    
    if(abs(waypoint_local_.x - target_local_.x)<safeDistance){
        if(abs(waypoint_local_.y - target_local_.y)<safeDistance){
            if(abs(waypoint_local_.z - target_local_.z)<safeDistance){ //if drone is close enough to the target
                return true;
            }
        }
    }
    return false;
}

/* ******************* */
/* NODE */
/* ******************* */

int main(int argc, char **argv)
{
    srand (time(NULL));
    string ID = argv[1]; // to discriminate the drones and init a diffrent node for each one
    string uav = "uav" + ID;
    ros::init(argc, argv, uav);
    ros::NodeHandle nh;

    if (ros::param::get("/flightHeight", flightHeight)){} // param /flightHeight declared in simulation.launch file of motion package
    
    if (ros::param::get("/safeDistance", safeDistance)){} // param /safeDistance declared in simulation.launch file of motion package
    
    bool reached_waypoint = false; 

    /* Subscribers */

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    (uav + "/mavros/state", 10, state_cb); //instantiate a publisher to publish the commanded local position

    ros::Subscriber position_local_sub = nh.subscribe<geometry_msgs::PoseStamped> 
    (uav + "/mavros/local_position/pose", 10, position_local_cb);

    /* Publishers */

    ros::Publisher waypoint_local_pub = nh.advertise<geometry_msgs::PoseStamped>
    (uav + "/mavros/setpoint_position/local", 10);

    ros::Publisher take_off_global_pub = nh.advertise<geometry_msgs::PoseStamped>
    (uav + "/motion/position/global", 1);

    /* Service Clients */

    ros::ServiceClient target_point_client = nh.serviceClient<motion::new_point>
    (uav + "/motion/position/global/target");

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    (uav + "/mavros/cmd/arming"); //client to request arming

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    (uav + "/mavros/set_mode"); //client to request mode change

    ros::ServiceClient waypoint_client = nh.serviceClient<motion::new_point>
    (uav + "/motion/position/global/waypoint");

    ros::ServiceClient on_target_client = nh.serviceClient<motion::on_target>
    (uav + "/motion/on_target");

    /* PX4 has a timeout of 500ms between two Offboard commands. If this timeout is exceeded, the commander will fall 
    back to the last mode the vehicle was in before entering Offboard mode. This is why the publishing rate must 
    be faster than 2 Hz to also account for possible latencies. This is also the same reason why it is recommended 
    to enter Offboard mode from Position mode, this way if the vehicle drops out of Offboard mode it will stop in 
    its tracks and hover. */
    ros::Rate rate(20.0); //the setpoint publishing rate MUST be faster than 2Hz

    /* Before publishing anything, we wait for the connection to be established between MAVROS and the autopilot. This 
    loop should exit as soon as a heartbeat message is received. */
        while(ros::ok() && !current_state.connected){ // wait for FCU connection
        ros::spinOnce();
        rate.sleep();
    }

    take_off_point_local = position_local; //initial position, when drones have not taken off yet
    take_off_point_local.pose.position.z = flightHeight; // set take off desired poisition 
    take_off_point_global = local_to_global_coords(ID, take_off_point_local, 1);
    /* Before entering Offboard mode, you must have already started streaming setpoints. Otherwise the mode switch will 
    be rejected. Here, 100 was chosen as an arbitrary amount. */
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        waypoint_local_pub.publish(take_off_point_local); // for mavros
        take_off_global_pub.publish(take_off_point_global); // for Dstar
        ros::spinOnce();
        rate.sleep();
    }

    target_global_msg.request.ready = true;
    target_global_msg.response.ready = false;
    while(ros::ok() && !target_global_msg.response.ready){
        if (target_point_client.call(target_global_msg)){
            target_global = target_global_msg.response.new_point;
            ROS_INFO_STREAM("first target " << target_global.pose.position);
        }
    }

    target_local = local_to_global_coords(ID, target_global, -1);
    waypoint_local = take_off_point_local;

    offb_set_mode.request.custom_mode = "OFFBOARD"; //We set the custom mode to OFFBOARD

    /* The rest of the code is pretty self explanatory. We attempt to switch to Offboard mode, after which we arm the quad 
    to allow it to fly. We space out the service calls by 5 seconds so to not flood the autopilot with the requests. In 
    the same loop, we continue sending the requested pose at the appropriate rate. */
    
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){ //if mode is not offboard
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){ // check uav 's /mavros/set_mode service
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){ // if mode is offboard & drone is not armed
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){ // check uav 's /mavros/cmd/arming service
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        
        if (!reached_waypoint){
            reached_waypoint = has_reached_waypoint(waypoint_local.pose.position, position_local.pose.position);
        }
        else{
            waypoint_global_msg.request.ready = true;
            waypoint_global_msg.response.ready = false;
            ROS_INFO_STREAM("reached waypoint! Wainting for new waypoint...");
            while(!waypoint_global_msg.response.ready){ // brakes when response is ready
                if(waypoint_client.call(waypoint_global_msg)){
                }
                waypoint_local_pub.publish(waypoint_local); // keep streaming previous setpoint until you get the new one
            }
            waypoint_local = local_to_global_coords(ID, waypoint_global_msg.response.new_point, -1);
            reached_waypoint = false;
            ROS_INFO_STREAM("trying to reach waypoint...\n");
        }

        if(too_close_to_target(ID, waypoint_local.pose.position, target_local.pose.position)){ // keep publishing target position until an new one is set
            
            on_target_msg.request.arrival = true;
            on_target_msg.response.arrival_granted = false;
            while(!on_target_msg.response.arrival_granted){
                if(on_target_client.call(on_target_msg)){
                    ROS_INFO_STREAM("UAV" << ID << " reached target");
                }
            }

            while(!recieved_new_target){
                waypoint_local_pub.publish(waypoint_local); // keep streaming setpoint to stay safely close to the target, ecxept if UAV gets a new target
            }
        }else{
            waypoint_local_pub.publish(waypoint_local); // keep streaming setpoint until UAV reaches it
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}