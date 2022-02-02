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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

#define PI 3.14159265
int deg;
double rad;
bool circling_target = false;
double detection_circle_x;
double detection_circle_y;

float detectionDistance;
double patrolHeight;
double overheatHeight;
string mode = "patrol";
string overheat_sensed_at;


double dx;
double dy;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped position_local;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::CommandBool arm_cmd;
geometry_msgs::PoseStamped take_off_point_local;
geometry_msgs::PoseStamped take_off_point_global;
geometry_msgs::PoseStamped waypoint_local;
geometry_msgs::PoseStamped last_waypoint;
geometry_msgs::PoseStamped target_global;
geometry_msgs::PoseStamped patrol_target_global;
geometry_msgs::PoseStamped patrol_target_local;
geometry_msgs::PoseStamped overheat_target_local;
geometry_msgs::PoseStamped overheat_target_global;
motion::new_point waypoint_global_srv;
motion::on_target on_target_srv;

bool reached_waypoint = false;
bool got_overheat_target = false;
bool received_new_target = false;
bool waiting_to_be_on_patrol = false;

tf2::Quaternion myQuaternion;

/* ******************* */
/* CALLBSCKS */
/* ******************* */

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
    
    /* if (ID == "0"){
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
    } */

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
    
    if(abs(position_local_.x - overheat_target_local_.x)<=detectionDistance + 0.5){
        if(abs(position_local_.y - overheat_target_local_.y)<=detectionDistance + 0.5){
            if(abs(position_local_.z - overheat_target_local_.z)<=detectionDistance + 0.5){ //if drone is close enough to the target
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

void check_for_conflict(int id, geometry_msgs::Point my_pos, float safety_dist_from_other_UAVs, geometry_msgs::PoseStamped waypoint_local, ros::Rate rate, ros::NodeHandle nh){
    
    ros::Publisher waypoint_local_pub = nh.advertise<geometry_msgs::PoseStamped>
    ("uav" + to_string(id) + "/mavros/setpoint_position/local", 10);

    ros::Publisher global_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    ("uav" + to_string(id) + "/motion/position/global", 1);

    int swarmPopulation;
    string mode;
    geometry_msgs::PoseStamped other_UAV_pos;
    if (ros::param::get("/swarmPopulation", swarmPopulation)){}
    float dist = 0;

    for (int other_UAV_id = 0; other_UAV_id < swarmPopulation; other_UAV_id++){
        if (other_UAV_id == id) continue;
        if (ros::param::get("mode" + to_string(other_UAV_id), mode)){}

        if (mode == "detect"){
            ROS_INFO_STREAM("in function");
            while (dist < safety_dist_from_other_UAVs && ros::ok()){
                const geometry_msgs::PoseStamped::ConstPtr& msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>
                ("uav" + to_string(other_UAV_id) + "/motion/position/global");
                other_UAV_pos = *msg;
                ROS_INFO_STREAM("mine " << my_pos << " and " << other_UAV_pos.pose.position << "with ID: " << other_UAV_id);
                dist = distance(my_pos.x, my_pos.y, other_UAV_pos.pose.position.x, other_UAV_pos.pose.position.y);
                ROS_INFO_STREAM("in check for conflict " << dist );
                waypoint_local_pub.publish(waypoint_local); //keep streaming current waypoint
                global_pos_pub.publish(local_to_global_coords(to_string(id), position_local, 1));
                rate.sleep();
            }
        }
    }
}


/* ******************* */
/* NODE */
/* ******************* */

int main(int argc, char **argv)
{
    srand (time(NULL));
    string ID = argv[1]; // to discriminate the drones and init a diffrent node for each one
    int id = stoi(ID);
    // string uav = "uav" + ID;
    ros::init(argc, argv, "iris");
    ros::NodeHandle nh;

    if (ros::param::get("/patrolHeight", patrolHeight)){} // param /patrolHeight declared in simulation.launch file of motion package
    
    if (ros::param::get("/overheatHeight", overheatHeight)){} // param /patrolHeight declared in simulation.launch file of motion package
    
    if (ros::param::get("/detectionDistance", detectionDistance)){} // param /detectionDistance declared in simulation.launch file of motion package
    
    bool reached_waypoint = false; 

    /* Subscribers */

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    ("/mavros/state", 10, state_cb); //instantiate a publisher to publish the commanded local position

    ros::Subscriber position_local_sub = nh.subscribe<geometry_msgs::PoseStamped> 
    ("/mavros/local_position/pose", 10, position_local_cb);

    ros::Subscriber target_point_sub = nh.subscribe<geometry_msgs::PoseStamped> 
    ("/motion/position/global/target", 10, target_point_cb);

    /* Publishers */

    ros::Publisher waypoint_local_pub = nh.advertise<geometry_msgs::PoseStamped>
    ("/mavros/setpoint_position/local", 10);

    ros::Publisher global_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    ("/motion/position/global", 1);

    /* Service Clients */

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    ("/mavros/cmd/arming"); //client to request arming

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    ("/mavros/set_mode"); //client to request mode change

    ros::ServiceClient waypoint_client = nh.serviceClient<motion::new_point>
    ("/motion/position/global/waypoint");

    ros::ServiceClient on_target_client = nh.serviceClient<motion::on_target>
    ("/motion/on_target");


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
                waypoint_global_srv.request.ready = true;
                waypoint_global_srv.response.ready = false;
                ROS_INFO_STREAM("reached waypoint!" << waypoint_global_srv.response.new_point.pose.position << "Wainting for new waypoint...");
                while(!waypoint_global_srv.response.ready && ros::ok()){ // brakes when response is ready
                    if(waypoint_client.call(waypoint_global_srv)){
                    }
                    waypoint_local_pub.publish(waypoint_local); // keep streaming previous waypoint setpoint until you get the new one
                    global_pos_pub.publish(local_to_global_coords(ID, position_local, 1));
                }
                waypoint_local = local_to_global_coords(ID, waypoint_global_srv.response.new_point, -1);
                global_pos_pub.publish(local_to_global_coords(ID, position_local, 1));
                reached_waypoint = false;
                ROS_INFO_STREAM("UAV" << ID << " trying to reach patrol waypoint...\n");
            }

            if(on_patrol_target(ID, position_local.pose.position, patrol_target_local.pose.position)){ 
                
                on_target_srv.request.arrival = true;
                on_target_srv.response.arrival_granted = false;
                while(!on_target_srv.response.arrival_granted && ros::ok()){
                    if(on_target_client.call(on_target_srv)){ // notify main.py that this UAV reached patrol subtarget
                        ROS_INFO_STREAM("UAV" << ID << " reached patrol target");
                    }
                }
                
                while (target_global.pose.position == patrol_target_global.pose.position && ros::ok()){
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
            while (!got_overheat_target && ros::ok()){
                if (overheatHeight != target_global.pose.position.z){
                    ros::spinOnce();
                    rate.sleep();
                }
                else{
                    overheat_target_global =  target_global; // overheated point target is set
                    overheat_target_local = local_to_global_coords(ID, overheat_target_global, -1);
                    ROS_INFO_STREAM("overheat target is:\n " << overheat_target_global.pose.position);
                    got_overheat_target = true;
                }
            }
            

            if (!reached_waypoint){
                reached_waypoint = has_reached_waypoint(waypoint_local.pose.position, position_local.pose.position);
            }
            else{
                waypoint_global_srv.request.ready = true;
                waypoint_global_srv.response.ready = false;
                ROS_INFO_STREAM("reached waypoint! Wainting for new waypoint...");
                while(!waypoint_global_srv.response.ready && ros::ok()){ // brakes when response is ready
                    if(waypoint_client.call(waypoint_global_srv)){
                    }
                    waypoint_local_pub.publish(waypoint_local); // keep streaming previous waypoint setpoint until you get the new one
                    global_pos_pub.publish(local_to_global_coords(ID, position_local, 1));
                }
                waypoint_local = local_to_global_coords(ID, waypoint_global_srv.response.new_point, -1);
                global_pos_pub.publish(local_to_global_coords(ID, position_local, 1));
                reached_waypoint = false;
                ROS_INFO_STREAM("trying to reach waypoint...\n");
            }

            if(detected_target(ID, position_local.pose.position, overheat_target_local.pose.position)){ // keep publishing target position until an new one is set
                
                on_target_srv.request.arrival = true;
                on_target_srv.response.arrival_granted = false;
                while(!on_target_srv.response.arrival_granted && ros::ok()){
                    if(on_target_client.call(on_target_srv)){ // notify main.py that this UAV reached overheated point
                        ROS_INFO_STREAM("UAV" << ID << " reached overheated area");
                    }
                }

                float circleRadius = 2;
                float allow_entrance_dist = detectionDistance - circleRadius + 0.5;
                float safety_dist_from_other_UAVs = detectionDistance + circleRadius/2;
                float allow_exit_dist = allow_entrance_dist;  
                last_waypoint = waypoint_local;
                while(!received_new_target && ros::ok()){
                    for(deg = 0; deg <360; deg = deg + 10){

                        rad = deg * PI / 180.0;

                        detection_circle_x = overheat_target_local.pose.position.x + sin(rad)*circleRadius;
                        detection_circle_y = overheat_target_local.pose.position.y + cos(rad)*circleRadius;
                        
                        if (!circling_target){
                            waypoint_local_pub.publish(waypoint_local); //keep streaming current waypoint
                            global_pos_pub.publish(local_to_global_coords(ID, position_local, 1));
                            // choose entrance point, in order to not let UAV travel through the circle to enter it
                            if (distance(waypoint_local.pose.position.x, waypoint_local.pose.position.y, detection_circle_x, detection_circle_y) < allow_entrance_dist){
                                check_for_conflict(id, local_to_global_coords(ID, position_local, 1).pose.position, safety_dist_from_other_UAVs, waypoint_local , rate, nh);
                                circling_target = true;
                            }
                        }else{
                            if (ros::param::get("/overheat_sensed_at", overheat_sensed_at)){} // param /overheat_sensed_at declared in simulation.launch file of motion package

                            
                            waypoint_local.pose.position.x = detection_circle_x;
                            waypoint_local.pose.position.y = detection_circle_y; 
                            myQuaternion.setRPY( 0, 0, -atan2(sin(rad),cos(rad)) - PI/2 );
                            waypoint_local.pose.orientation = tf2::toMsg(myQuaternion);
                            
                            waypoint_local_pub.publish(waypoint_local);
                            global_pos_pub.publish(local_to_global_coords(ID, position_local, 1));
                            ros::spinOnce();
                            circle_rate.sleep();
                            if (overheat_sensed_at == "" && distance(waypoint_local.pose.position.x, waypoint_local.pose.position.y, last_waypoint.pose.position.x, last_waypoint.pose.position.y) < allow_exit_dist){
                                waiting_to_be_on_patrol = true;
                                break;
                            }
                        }
                        ros::spinOnce();
                        rate.sleep();
                    }
                    while (waiting_to_be_on_patrol && ros::ok()){
                        waypoint_local_pub.publish(last_waypoint);
                        // ROS_INFO_STREAM("after "" : "<<last_waypoint.pose.position << overheat_target_local.pose.position);
                        global_pos_pub.publish(local_to_global_coords(ID, position_local, 1));
                        ROS_INFO_STREAM("waiting " << local_to_global_coords(ID, position_local, 1).pose.position );
                        
                        if (target_global != overheat_target_global){
                            received_new_target = true;
                            patrol_target_global =  target_global; // new subtarget is set
                            patrol_target_local = local_to_global_coords(ID, patrol_target_global, -1);
                            ROS_INFO_STREAM("my " << ID << "new patrol target is /n" << patrol_target_global.pose.position );
                            waiting_to_be_on_patrol = false;
                        }
                        ros::spinOnce();
                        rate.sleep();
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