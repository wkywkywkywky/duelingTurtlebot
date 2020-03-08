#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <cmvision/Blob.h>
#include <cmvision/Blobs.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <depth_image_proc/depth_traits.h>
#include <sound_play/sound_play.h>
#include <ctime>

// speed 
const double z_scale_ = 0.3;
const double x_scale_ = 5;

// point clound distance
double pc_distance = 1000;

// states for FSM
enum STATE {RANDOM, RULEEXPLAINATION, PERSONMOVE, ROBOTMOVE, WAITFORPRESENT, DUEL};
STATE state = RANDOM;

// Twist publisher
ros::Publisher pub;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void PointCloud_Callback (const PointCloud::ConstPtr& cloud){
    const double goal_z_ = 0.6, max_z_ = 1.2, min_y_ = 0.1, max_y_ = 0.5, min_x_ = -0.2, max_x_ = 0.2;
    double x = 0.0, z = 10.0;
    double count = 0;

    //Iterate through all the points in the image
    //Convert from pcl to cm
    for(int k = 0; k < 240; k++){
        for(int i = 0; i < 640; i++){
            const pcl::PointXYZ & pt=cloud->points[640*(180+k)+(i)];
            if((pt.z < max_z_ && min_x_ < pt.x && pt.x < max_x_ && min_y_ < pt.y && pt.y < max_y_)){   
                x += pt.x;
                count += 1;

                if(pt.z < z)
                    z = pt.z;
            }
        }
    }

    std::cout << z << std::endl;

    if(count < 4000){
    	pc_distance = 1000;
    }

    pc_distance = z;

}

void RandomState(){
	const double z_max = 1.2;
	if(pc_distance < z_max ){
		std::cout << "start rule exlaination" << std::endl;
		state = RULEEXPLAINATION;
	}
}

void RuleExplainationState(){
}

void PersonMoveState(){
}

void RobotMoveState(){
}

void WaitForPresentState(){
}

void DuelState(){

}


int main(int argc, char **argv){
    ros::init(argc, argv, "publish_velocity");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);
    ros::Subscriber objsub = nh.subscribe<PointCloud>("/camera/depth/points", 1, PointCloud_Callback);
    sound_play::SoundClient sc;
    srand(time(0));
    ros::Rate rate(2);
    while(ros::ok()){
        if(state == RANDOM){
        	RandomState();
        }
        else if(state == RULEEXPLAINATION){
        	RuleExplainationState();
        }
        else if(state == PERSONMOVE){
        	PersonMoveState();
        }    
        else if(state == ROBOTMOVE){
        	RobotMoveState();
        }    
        else if(state == WAITFORPRESENT){
        	WaitForPresentState();
        }        
        else if(state == DUEL){
        	DuelState();
        }        
        rate.sleep(); 
        ros::spinOnce();
    }
}
