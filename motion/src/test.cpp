#include <ros/ros.h>
#include <iostream>
#include <utility>
#include <cstdlib>
#include <time.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

// length for  global_position_of_UAVs array, that stores the last <num_of_positions_kept> positions of UAVs, 
// to allow comparison among them, so we can procced to the collision checking
const int num_of_positions_kept = 20; 

// local position for UAVs aquared from their callbacks -> (position_local_cb)
geometry_msgs::PoseStamped position_local;

// iterator for the global_position_of_UAVs array
int g_p_UAV_iterator = 0;

bool expecting_collision = false;

/* ******************* */
/* STUCTURE */
/* ******************* */

// structure to link UAVs with their position
struct UAV_location
{
    int ID;
    geometry_msgs::Point position;
}global_position_of_UAVs [num_of_positions_kept]; 

/* ******************* */
/* FUCTIONS */
/* ******************* */

geometry_msgs::Point local_to_global_coords(int ID, geometry_msgs::Point pos){

    if (ID == 0){
        pos.x += 15;
    }
    else if (ID == 1){
        pos.x -= 15;
    }
    else if (ID == 2){
        pos.y += 15;
    }
    else if (ID == 3){
        pos.y -= 15;
    }

    return pos;
}

/* ******************* */
/* CLASS */
/* ******************* */

class position_subscriber
{
    private:
    int ID;
    
    public:
    position_subscriber(int ID){
        this->ID = ID;
    }

    void position_local_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){ // current local position
        position_local = *msg;
        global_position_of_UAVs[g_p_UAV_iterator].position = local_to_global_coords(this->ID, position_local.pose.position);
        global_position_of_UAVs[g_p_UAV_iterator].ID = this->ID;

        if(g_p_UAV_iterator < num_of_positions_kept-1){
            g_p_UAV_iterator++;
        }else{
            g_p_UAV_iterator = 0;
        }
    }
};

pair<int, int> check_for_collision(double obstacleRange){
    for(int i = 0; i < num_of_positions_kept; i++){
        for(int j = 0; j < num_of_positions_kept; j++){
            if(global_position_of_UAVs[i].ID != global_position_of_UAVs[j].ID){
                if(abs(global_position_of_UAVs[i].position.x - global_position_of_UAVs[j].position.x) < obstacleRange+3){
                    if(abs(global_position_of_UAVs[i].position.y - global_position_of_UAVs[j].position.y) < obstacleRange+3){
                        return make_pair(global_position_of_UAVs[i].ID, global_position_of_UAVs[j].ID);
                    }
                }
            }
            return make_pair(0, 0); // in case there is no collision detected
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_avoidance_node");
    ros::NodeHandle nh;

    int swarmPopulation;
    if (ros::param::get("/swarmPopulation", swarmPopulation)){} // param /swarmPopulation declared in simulation.launch file of motion package

    double obstacleRange;
    if (ros::param::get("/obstacleRange", obstacleRange)){} // param /obstacleRange declared in simulation.launch file of motion package


    // array of position_subscriber objects
    position_subscriber* position_sub = (position_subscriber*)malloc(sizeof(position_subscriber) * swarmPopulation); 

    string uav;

    /* Subscribers */
    
    for(int ID = 0; ID = swarmPopulation; ID++){

        uav = "uav" + to_string(ID);
        position_sub[ID] = position_subscriber(ID); // initialize ID values of the object via the class constructor

        // subscribers to uav + "/mavros/local_position/pose" topics, to get current positions of the UAVs
        ros::Subscriber position_local_sub = nh.subscribe<geometry_msgs::PoseStamped> 
        (uav + "/mavros/local_position/pose", 10, &position_subscriber::position_local_cb, &position_sub[ID]);
    }

    ros::Rate rate(20.0);

    while(ros::ok()){

        pair<int, int> UAVS_to_collide = check_for_collision(obstacleRange); //when it returns different values(hench not (0, 0), there is a collision detected)

        expecting_collision = (UAVS_to_collide.first != UAVS_to_collide.second) ? true : false; 

        if (expecting_collision){
            ROS_INFO_STREAM("UAV" << UAVS_to_collide.first << " and UAV" << UAVS_to_collide.second << " are about to collide" );
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}