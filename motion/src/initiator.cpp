#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <time.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
// #include <motion/take_off.h>

using namespace std;


mavros_msgs::State current_state;
geometry_msgs::PoseStamped position_local;
geometry_msgs::PoseStamped target_point_global;

geometry_msgs::PoseStamped next_step_local; 

bool has_reached_take_off_point = false;
bool has_reached_step_point = false;


/* ******************* */
/* CALLBACKS */
/* ******************* */


void state_cb(const mavros_msgs::State::ConstPtr& msg){ // current state of the autopilot 
    current_state = *msg;
}


void position_local_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){ // current local position
    position_local = *msg;
}


void target_point_global_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){ // current global target point
    target_point_global = *msg;
}

/* ******************* */
/* FUCTION DECLARATIONS & DEFINITIONS */
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

geometry_msgs::PoseStamped avoid_on_goal_collision(string ID, geometry_msgs::PoseStamped pos){

    /* if (ID == "0"){
        pos.pose.position.x += 0.5;
    }
    else if (ID == "1"){
        pos.pose.position.x -= 0.5;
    }
    else if (ID == "2"){
        pos.pose.position.y += 0.5;
    }
    else if (ID == "3"){
        pos.pose.position.y -= 0.5;
    } */

    if (ID == "0"){
         14;
    }
    else if (ID == "1"){
        pos.pose.position.x -= 14;
    }
    else if (ID == "2"){
        pos.pose.position.y += 14;
    }
    else if (ID == "3"){
        pos.pose.position.y -= 14;
    }

    return pos;
}



/* Even though the PX4 Pro Flight Stack operates in the aerospace NED coordinate frame, MAVROS translates these 
coordinates to the standard ENU frame and vice-versa. */

void request_next_step(string ID){

    has_reached_step_point = false;   

    // next_step_local =local_to_global_coords(ID, target_point_global, -1); //pass local goal position to geometry_msgs::PoseStamped variable
    // //ROS_INFO_STREAM("uav" << ID << " 's local target is/n x: " << next_step_local.pose.position.x << "\ty: " << next_step_local.pose.position.y << "\tz: " <<  next_step_local.pose.position.z);
    
    // next_step_local = avoid_on_goal_collision(ID, next_step_local);
    // //ROS_INFO_STREAM("uav" << ID << " 's local target is/n x: " << next_step_local.pose.position.x << "\ty: " << next_step_local.pose.position.y << "\tz: " <<  next_step_local.pose.position.z);
    
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

    /* ros::Subscriber position_local_sub = nh.subscribe<geometry_msgs::PoseStamped>
    ("mavros/local_position/pose/" + uav, 10, boost::bind(position_local_cb,_1, ID)); */ // this allows the callback to take one more argument

    ros::Subscriber position_local_sub = nh.subscribe<geometry_msgs::PoseStamped> 
    (uav + "/mavros/local_position/pose", 10, position_local_cb);

    ros::Subscriber target_point_global_sub = nh.subscribe<geometry_msgs::PoseStamped>
    ("motion/position/global/target", 10, target_point_global_cb);
    

    /* Publishers */
    ros::Publisher next_step_local_pub = nh.advertise<geometry_msgs::PoseStamped>
    (uav + "/mavros/setpoint_position/local", 10);

    ros::Publisher target_point_global_pub = nh.advertise<geometry_msgs::PoseStamped>
    (uav + "/mavros/setpoint_position/global", 10);

    /* ros::Publisher took_off_pub = nh.advertise<motion::take_off>
    (uav + "/motion/take_off_topic", 10); */

    ros::Publisher position_global_pub = nh.advertise<geometry_msgs::PoseStamped>
    (uav + "/motion/position/global", 10);
    

    /* Service Clients */
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    (uav + "/mavros/cmd/arming"); //client to request arming

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    (uav + "/mavros/set_mode"); //client to request mode change



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
    
    geometry_msgs::PoseStamped take_off_point_local = position_local; //initial position, when drones have not taken off yet
    take_off_point_local.pose.position.z = 2; // set take off desired poisition 

    /* Before entering Offboard mode, you must have already started streaming setpoints. Otherwise the mode switch will 
    be rejected. Here, 100 was chosen as an arbitrary amount. */
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        next_step_local_pub.publish(take_off_point_local);
        ros::spinOnce();
        rate.sleep();
    }
    // ROS_INFO("fed local_goal_pos 100 times for uav");

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
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){ //if mode is not offboard
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){ // check uav 's /mavros/set_mode topic
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){ // if mode is offboard & drone is not armed
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){ // check uav 's /mavros/cmd/arming topic
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if(!has_reached_take_off_point){ 
            next_step_local = take_off_point_local; // until it reaches take off setpoint
        } else {
            if (has_reached_step_point)
            {
                request_next_step(ID); // when it reaches every point request the next step(it is stored in next_step_local)
            }
        }
        
        
        next_step_local_pub.publish(next_step_local); // keep streaming setpoints
        ROS_INFO_STREAM("published next_step_local " << next_step_local.pose.position << " for " << uav);

        target_point_global_pub.publish(target_point_global); // for ploting goal positions at plot.py
        //ROS_INFO_STREAM(uav << " 's global target point is/n x: " << target_point_global.pose.position.x << "/ty: " << target_point_global.pose.position.y << "/tz: " << target_point_global.pose.position.z);

        geometry_msgs::PoseStamped position_global = local_to_global_coords(ID, position_local, 1);
        position_global_pub.publish(position_global);

        // motion::take_off take_off_flag;
        // take_off_flag.took_off = false;
        if(abs(next_step_local.pose.position.x - position_local.pose.position.x)<0.1){
            if(abs(next_step_local.pose.position.y - position_local.pose.position.y)<0.1){
                if(abs(next_step_local.pose.position.z - position_local.pose.position.z)<0.1){ //if drone reaches goal take_off position
                    has_reached_take_off_point = true;
                    has_reached_step_point = true;
                    //request_next_step(ID);
                    /* take_off_flag.took_off = true;
                    took_off_pub.publish(take_off_flag); */
            
                    // next_step_local.pose.position.x = position_local.pose.position.x;
                    // next_step_local.pose.position.y = position_local.pose.position.y;
                    // next_step_local.pose.position.z = position_local.pose.position.z;
                } 
            } 
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}