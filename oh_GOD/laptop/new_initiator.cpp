#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <motion/new_point.h>
#include <motion/flag.h>

using namespace std;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped position_local;
geometry_msgs::PoseStamped take_off_point_local;
geometry_msgs::PoseStamped take_off_point_global;
geometry_msgs::PoseStamped setpoint_local;
geometry_msgs::PoseStamped target_point_global;
motion::new_point new_setpoint_global_msg;
geometry_msgs::PoseStamped setpoint_global;

bool request_new_setpoint = false;

/* ******************* */
/* SERVICE SERVER HANLDLERS */
/* ******************* */

bool reached_setpoint_handler(motion::flag::Request &req, motion::flag::Response &res){
    res.value = request_new_setpoint;
    return true;
}

/* bool reached_new_point_handler(motion::flag::Request &req, motion::flag::Response &res){
    res.value = false;
    has_reached_setpoint = false;
    return true;
} */


/* ******************* */
/* CALLBACKS */
/* ******************* */

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
        pos.pose.position.y =- 15*direction;
    }

    return pos;
}

bool has_UAV_reached_setpoint(geometry_msgs::Point P_setpoint_local, geometry_msgs::Point P_position_local){
    if(abs(P_setpoint_local.x - P_position_local.x)<0.1){
        if(abs(P_setpoint_local.y - P_position_local.y)<0.1){
            if(abs(P_setpoint_local.z - P_position_local.z)<0.1){ //if drone reaches goal take_off position
                return true;
            }
        }
    }
    ROS_INFO_STREAM(P_setpoint_local <<"\nset and pos\n" << P_position_local);
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

    /* Subscribers */

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    (uav + "/mavros/state", 10, state_cb); //instantiate a publisher to publish the commanded local position

    ros::Subscriber position_local_sub = nh.subscribe<geometry_msgs::PoseStamped> 
    (uav + "/mavros/local_position/pose", 10, position_local_cb);

    /* Publishers */

    ros::Publisher new_setpoint_local_pub = nh.advertise<geometry_msgs::PoseStamped>
    (uav + "/mavros/setpoint_position/local", 10);

    ros::Publisher take_off_point_global_pub = nh.advertise<geometry_msgs::PoseStamped>
    (uav + "/motion/position/global", 1);

    /* Service Clients */

    ros::ServiceClient target_point_client = nh.serviceClient<motion::new_point>
    ("motion/position/global/target");

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    (uav + "/mavros/cmd/arming"); //client to request arming

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    (uav + "/mavros/set_mode"); //client to request mode change

    ros::ServiceClient setpoint_client = nh.serviceClient<motion::new_point>
    (uav + "/motion/position/global/setpoint");

    /* Service Server */

    ros::ServiceServer reached_setpoint_server = nh.advertiseService
    (uav + "/motion/reached_setpoint", &reached_setpoint_handler); 

    /* ros::ServiceServer reached_new_point_server = nh.advertiseService
    (uav + "/motion/reached_new_point", &reached_new_point_handler); */

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
    take_off_point_local.pose.position.z = 2; // set take off desired poisition 
    take_off_point_global = local_to_global_coords(ID, take_off_point_local, 1);
    /* Before entering Offboard mode, you must have already started streaming setpoints. Otherwise the mode switch will 
    be rejected. Here, 100 was chosen as an arbitrary amount. */
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        new_setpoint_local_pub.publish(take_off_point_local); // for mavros
        take_off_point_global_pub.publish(take_off_point_global); // for Dstar
        ros::spinOnce();
        rate.sleep();
    }

    motion::new_point target_point_global_msg;
    target_point_global_msg.request.value = true;
    target_point_global_msg.response.ready = false;
    while(ros::ok() && !target_point_global_msg.response.ready){
        if (target_point_client.call(target_point_global_msg)){
            target_point_global = target_point_global_msg.response.new_point;
            ROS_INFO_STREAM("first target " << target_point_global.pose.position);
        }
    }

    setpoint_local = take_off_point_local;
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD"; //We set the custom mode to OFFBOARD

    /* The rest of the code is pretty self explanatory. We attempt to switch to Offboard mode, after which we arm the quad 
    to allow it to fly. We space out the service calls by 5 seconds so to not flood the autopilot with the requests. In 
    the same loop, we continue sending the requested pose at the appropriate rate. */
    mavros_msgs::CommandBool arm_cmd;
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

        
        request_new_setpoint = has_UAV_reached_setpoint(setpoint_local.pose.position, position_local.pose.position);
        ROS_INFO_STREAM("Is it ever getting "<<request_new_setpoint);
        // When UAV reaches setpoint it publishes it to get a new one
        /* if(abs(setpoint_local.pose.position.x - position_local.pose.position.x)<0.1){
                if(abs(setpoint_local.pose.position.y - position_local.pose.position.y)<0.1){
                    if(abs(setpoint_local.pose.position.z - position_local.pose.position.z)<0.1){ //if drone reaches goal take_off position
                        request_new_setpoint = true; */
                     
        
        while(request_new_setpoint){
            new_setpoint_global_msg.request.value = request_new_setpoint;
            if (setpoint_client.call(new_setpoint_global_msg)){
                setpoint_global = new_setpoint_global_msg.response.new_point;
                ROS_INFO_STREAM("UAV" << ID << "reached setpoint\n" << setpoint_global.pose.position);
                setpoint_local = local_to_global_coords(ID, setpoint_global, -1);
                request_new_setpoint = false;
            }
        }
        
        
        
            /*         } 
                } 
            } */
       
        //  όταν φτάνω στο setpoint θα κάνω true ένα variable που θα καλεί η main.py
        //  θα παίρνω με service call το new_setpoint (εξω απο το if ? οχι)

        new_setpoint_local_pub.publish(setpoint_local); // keep streaming setpoints

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}