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

int swarmPopulation;

#define PI 3.14159265
int deg;
double rad;
bool entered_detection_circle = false;
double detection_circle_x;
double detection_circle_y;

double safeDistance;
double patrolHeight;
double overheatHeight;

double dx;
double dy;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped position_local;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::CommandBool arm_cmd;
geometry_msgs::PoseStamped take_off_point_local;
geometry_msgs::PoseStamped take_off_point_global;
geometry_msgs::PoseStamped waypoint_local;
geometry_msgs::PoseStamped target_global;
geometry_msgs::PoseStamped patrol_target_global;
geometry_msgs::PoseStamped patrol_target_local;
geometry_msgs::PoseStamped overheat_target_local;
geometry_msgs::PoseStamped overheat_target_global;
motion::new_point waypoint_global_msg;
motion::on_target on_target_msg;

string mode = "patrol";

bool reached_waypoint = false;
bool got_overheat_target = false;
bool recieved_new_target = false;

void state_cb(const mavros_msgs::State::ConstPtr& msg){ // current state of the autopilot 
    current_state = *msg;
}


void position_local_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){ // current local position
    position_local = *msg;
}

void target_point_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){ // current global target(either for patrol or for detected overheat)
    target_global = *msg;
}

/* ******************* */
/* FUCTIONS */
/* ******************* */


geometry_msgs::PoseStamped local_to_global_coords(string ID, geometry_msgs::PoseStamped pos, int direction){
    // direction controlls whether the coordinates are converted from local to global or from global to local, should either be 1 or -1.
    
    if (ID == "0"){
        pos.pose.position.x -= 17*direction;
        pos.pose.position.y += 14*direction;
    }
    else if (ID == "1"){
       pos.pose.position.x += 17*direction;
       pos.pose.position.y += 14*direction;
    }
    else if (ID == "2"){
        pos.pose.position.x -= 17*direction;
        pos.pose.position.y -= 14*direction;
    }
    else if (ID == "3"){
        pos.pose.position.x += 17*direction;
        pos.pose.position.y -= 14*direction;
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

bool on_patrol_target(string ID, geometry_msgs::Point position_local_, geometry_msgs::Point patrol_target_local_){
    
    if(abs(position_local_.x - patrol_target_local_.x) < 0.1){
        if(abs(position_local_.y - patrol_target_local_.y) < 0.1){
            if(abs(position_local_.z - patrol_target_local_.z) < 0.1){ //if drone has reached patrol target
                return true;
            }
        }
    }
    return false;
}

bool detected_target(string ID, geometry_msgs::Point position_local_, geometry_msgs::Point overheat_target_local_){
    
    if(abs(position_local_.x - overheat_target_local_.x)<safeDistance){
        if(abs(position_local_.y - overheat_target_local_.y)<safeDistance){
            if(abs(position_local_.z - overheat_target_local_.z)<safeDistance){ //if drone is close enough to the target
                return true;
            }
        }
    }
    return false;
}

// Function to calculate distance
float distance(int x1, int y1, int x2, int y2)
{
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
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

    if (ros::param::get("/swarmPopulation", swarmPopulation)){} // param /swarmPopulation declared in simulation.launch file of motion package

    if (ros::param::get("/patrolHeight", patrolHeight)){} // param /patrolHeight declared in simulation.launch file of motion package
    
    if (ros::param::get("/overheatHeight", overheatHeight)){} // param /patrolHeight declared in simulation.launch file of motion package
    
    if (ros::param::get("/safeDistance", safeDistance)){} // param /safeDistance declared in simulation.launch file of motion package
    
    bool reached_waypoint = false; 

    /* Subscribers */

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    (uav + "/mavros/state", 10, state_cb); //instantiate a publisher to publish the commanded local position

    ros::Subscriber position_local_sub = nh.subscribe<geometry_msgs::PoseStamped> 
    (uav + "/mavros/local_position/pose", 10, position_local_cb);

    ros::Subscriber target_point_sub = nh.subscribe<geometry_msgs::PoseStamped> 
    (uav + "/motion/position/global/target", 10, target_point_cb);

    /* Publishers */

    ros::Publisher waypoint_local_pub = nh.advertise<geometry_msgs::PoseStamped>
    (uav + "/mavros/setpoint_position/local", 10);

    ros::Publisher global_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    (uav + "/motion/position/global", 1);

    /* Service Clients */

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
    ros::Rate circle_rate(3.0); // rate to publish setpoints for circling overheated point

    /* Before publishing anything, we wait for the connection to be established between MAVROS and the autopilot. This 
    loop should exit as soon as a heartbeat message is received. */
        while(ros::ok() && !current_state.connected){ // wait for FCU connection
        ros::spinOnce();
        rate.sleep();
    }

    take_off_point_local = position_local; //initial position, when drones have not taken off yet
    take_off_point_local.pose.position.z = patrolHeight; // set take off desired poisition 
    take_off_point_local.pose.orientation.w = 0;
    take_off_point_local.pose.orientation.x = 1;
    take_off_point_global = local_to_global_coords(ID, take_off_point_local, 1);
    /* Before entering Offboard mode, you must have already started streaming setpoints. Otherwise the mode switch will 
    be rejected. Here, 100 was chosen as an arbitrary amount. */
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        waypoint_local_pub.publish(take_off_point_local); // for mavros
        global_pos_pub.publish(take_off_point_global); // for Dstar
        ros::spinOnce();
        rate.sleep();
    }
    waypoint_local = take_off_point_local; // consider take_off_point to be the first waypoint
    
    target_global = take_off_point_global;
    patrol_target_global = take_off_point_global;
    patrol_target_local = local_to_global_coords(ID, patrol_target_global, -1);
    
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

        if (ros::param::get("/mode" + ID, mode)){} // param /mode + ID declared in simulation.launch file of motion package        

        if(mode == "patrol"){
            if (!reached_waypoint){
                reached_waypoint = has_reached_waypoint(waypoint_local.pose.position, position_local.pose.position);
            }
            else{
                waypoint_global_msg.request.ready = true;
                waypoint_global_msg.response.ready = false;
                ROS_INFO_STREAM("reached waypoint!" << waypoint_global_msg.response.new_point.pose.position << "Wainting for new waypoint...");
                while(!waypoint_global_msg.response.ready){ // brakes when response is ready
                    if(waypoint_client.call(waypoint_global_msg)){
                    }
                    waypoint_local_pub.publish(waypoint_local); // keep streaming previous waypoint setpoint until you get the new one
                    global_pos_pub.publish(local_to_global_coords(ID, position_local, 1));
                }
                waypoint_local = local_to_global_coords(ID, waypoint_global_msg.response.new_point, -1);
                reached_waypoint = false;
                ROS_INFO_STREAM("UAV" << ID << " trying to reach patrol waypoint...\n");
            }

            if(on_patrol_target(ID, position_local.pose.position, patrol_target_local.pose.position)){ 
                
                on_target_msg.request.arrival = true;
                on_target_msg.response.arrival_granted = false;
                while(!on_target_msg.response.arrival_granted){
                    if(on_target_client.call(on_target_msg)){ // notify main.py that this UAV reached patrol subtarget
                        ROS_INFO_STREAM("UAV" << ID << " reached patrol target");
                    }
                }
                
                while (target_global.pose.position == patrol_target_global.pose.position){
                    waypoint_local_pub.publish(waypoint_local); // keep streaming patrol subtarget setpoint until UAV gets a new patrol subtarget
                    global_pos_pub.publish(local_to_global_coords(ID, position_local, 1));
                    ros::spinOnce();
                    rate.sleep();
                } 

                patrol_target_global =  target_global; // new subtarget is set
                patrol_target_local = local_to_global_coords(ID, patrol_target_global, -1);              
            }else{
                waypoint_local_pub.publish(waypoint_local); // keep streaming waypoint setpoint until UAV reaches it
                global_pos_pub.publish(local_to_global_coords(ID, position_local, 1));
            }
        }

        if(mode == "detect"){
            // ROS_INFO_STREAM("UAV" << ID << "on detect mode");
            while (!got_overheat_target){
                if (overheatHeight != target_global.pose.position.z){
                    ros::spinOnce();
                    rate.sleep();
                }
                else{
                    overheat_target_global =  target_global; // new subtarget is set
                    overheat_target_local = local_to_global_coords(ID, overheat_target_global, -1);
                    ROS_INFO_STREAM("overheat target is:\n " << overheat_target_global.pose.position);
                    got_overheat_target = true;
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
                    waypoint_local_pub.publish(waypoint_local); // keep streaming previous waypoint setpoint until you get the new one
                    global_pos_pub.publish(local_to_global_coords(ID, position_local, 1));
                }
                waypoint_local = local_to_global_coords(ID, waypoint_global_msg.response.new_point, -1);
                reached_waypoint = false;
                ROS_INFO_STREAM("trying to reach waypoint...\n");
            }

            if(detected_target(ID, position_local.pose.position, overheat_target_local.pose.position)){ // keep publishing target position until an new one is set
                
                on_target_msg.request.arrival = true;
                on_target_msg.response.arrival_granted = false;
                while(!on_target_msg.response.arrival_granted){
                    if(on_target_client.call(on_target_msg)){ // notify main.py that this UAV reached overheated point
                        ROS_INFO_STREAM("UAV" << ID << " reached overheated area");
                    }
                }

                while(!recieved_new_target){
                    for(deg = 0; deg <360; deg = deg + 10){

                        rad = deg * PI / 180.0;

                        detection_circle_x = overheat_target_local.pose.position.x + cos(rad)*safeDistance;
                        detection_circle_y = overheat_target_local.pose.position.y + sin(rad)*safeDistance;
                        
                        if (!entered_detection_circle){
                            waypoint_local_pub.publish(waypoint_local); //keep streaming current waypoint
                            rate.sleep();
                            if (distance(waypoint_local.pose.position.x, waypoint_local.pose.position.y, detection_circle_x, detection_circle_y) < 2){
                                
                                // for 
                                
                                entered_detection_circle = true;
                            }
                        }else{
                            waypoint_local.pose.position.x = detection_circle_x;
                            waypoint_local.pose.position.y = detection_circle_y;

                            /* dx = waypoint_local.pose.position.x - overheat_target_local.pose.position.x;
                            dy = waypoint_local.pose.position.y - overheat_target_local.pose.position.y;
                            waypoint_local.pose.orientation.z = atan2(dx, dy); */

                            waypoint_local_pub.publish(waypoint_local);
                            circle_rate.sleep();
                        }
                    }
                }
            }else{
                waypoint_local_pub.publish(waypoint_local); // keep streaming setpoint until UAV reaches it
                global_pos_pub.publish(local_to_global_coords(ID, position_local, 1));
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}