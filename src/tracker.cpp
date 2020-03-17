//coded by Kunyu Wang and Cora Coleman for UCSD CSE 276B final project
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
#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>

// speed 
const double z_scale_ = 0.3;
const double x_scale_ = 5;

// point clound distance
double pc_distance = 1000;

// states for FSM
enum STATE {RANDOM, RULEEXPLAINATION, PERSONMOVE, ROBOTMOVE, WAITFORPRESENT, DUEL, IWIN, YOUWIN};
STATE state = RANDOM;

// Twist publisher
ros::Publisher pub;

// UI publisher
ros::Publisher UI_pub;

// rule explaination timer
time_t rule_starting_time = 0;

//fire timer
time_t fire_starting_time = 0;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void PointCloud_Callback (const PointCloud::ConstPtr& cloud){
    const double goal_z_ = 0.6, max_z_ = 5, min_y_ = 0.1, max_y_ = 0.5, min_x_ = -0.1, max_x_ = 0.1;
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

//check if sufficient points are detected
    if(count < 5000){
//        set distance to inf if not
    	pc_distance = 1000;
    }
    else{
//        set distace to the closest point detected
        pc_distance = z;
    }

}

void BlobCallBack(cmvision::Blobs blobsIn){

	double blue_area = 0;
	double pink_area = 0;
	for(int i = 0; i < blobsIn.blob_count; i++){
//        getting rid of the small areas
		if(blobsIn.blobs[i].area < 10)
			continue;
        
//        get area sum for pink color
		if(blobsIn.blobs[i].name == "Pink"){
			pink_area += blobsIn.blobs[i].area;

		}
//        get ares sum for blue color
		else if (blobsIn.blobs[i].name == "Blue"){
			blue_area += blobsIn.blobs[i].area;
		}
	}
    
//    gun detected
	if(pink_area > 500){
		state = YOUWIN;
	}

//    green board detected
	if(state == WAITFORPRESENT && blue_area > 500){
		state = DUEL;
	}

//    green board goes away = ready to fire
	if(state == DUEL && blue_area <= 500){
		state = IWIN;
		fire_starting_time = time(NULL);
	}

}

void RandomState(){
//    publishing UI
	std_msgs::String msg;
    std::stringstream ss;
    ss << "/home/turtlebot/turtlebot_ws/src/duelingturtlebot/src/random.jpg";
    msg.data = ss.str();
    UI_pub.publish(msg);

//    start rule explanation when person is close enough
	const double z_max = 1.0;
	if(pc_distance < z_max ){
		std::cout << "start rule exlaination" << std::endl;
		state = RULEEXPLAINATION;
		rule_starting_time = time(NULL);
	}
}

void RuleExplainationState(){
//    publishing UI
	std_msgs::String msg;
    std::stringstream ss;
    ss << "/home/turtlebot/turtlebot_ws/src/duelingturtlebot/src/ruleexplaination.jpg";
    msg.data = ss.str();
    UI_pub.publish(msg);

//    person can move after five 5sec
    time_t cur_time = time(NULL); 
	if(cur_time - rule_starting_time > 5){
		std::cout << "person can move" << std::endl;
		state = PERSONMOVE;
	}
}

void PersonMoveState(){
    //    publishing UI
	std_msgs::String msg;
    std::stringstream ss;
    ss << "/home/turtlebot/turtlebot_ws/src/duelingturtlebot/src/personmove.jpg";
    msg.data = ss.str();
    UI_pub.publish(msg);

    const double z_min = 4;
    static int count = 0;
    std::cout<<pc_distance<<std::endl;

    // point cloud far enough indicating person finished stepping back
    if( 1.3 > pc_distance && pc_distance > 1.15 && pc_distance != 1000){
        count ++;

        if(count == 5){
            std::cout << "robot can move" << std::endl;
            state = ROBOTMOVE;
            count = 0;
        }
    }
}

void RobotMoveState(){
    //    publishing UI
	std_msgs::String msg;
    std::stringstream ss;
    ss << "/home/turtlebot/turtlebot_ws/src/duelingturtlebot/src/robotmove.jpg";
    msg.data = ss.str();
    UI_pub.publish(msg);

    static int step = 0;

    //robot move
    if(step < 5){
    	geometry_msgs::Twist tmsg;
    	tmsg.linear.x = - z_scale_;
    	pub.publish(tmsg);
    	step++;
    }
    else{
    	std::cout << "robot can move" << std::endl;
        state = WAITFORPRESENT;
        step = 0;
    }

}

void WaitForPresentState(){
    //    publishing UI
	std_msgs::String msg;
    std::stringstream ss;
    ss << "/home/turtlebot/turtlebot_ws/src/duelingturtlebot/src/waitforpresent.jpg";
    msg.data = ss.str();
    UI_pub.publish(msg);
}

void DuelState(){
    //    do nothing, just wait for green board to go away
}

void IWinState(){
	std_msgs::String msg;
    std::stringstream ss;
    time_t cur_time = time(NULL);
//    fire first
    if(cur_time - fire_starting_time < 2){
    	ss << "/home/turtlebot/turtlebot_ws/src/duelingturtlebot/src/fire.jpg";
    }
//    show i win after 5 secs
    else{
    	ss << "/home/turtlebot/turtlebot_ws/src/duelingturtlebot/src/iwin.jpg";
    }
    msg.data = ss.str();
    UI_pub.publish(msg);

}

void YouWinState(){
    //    publishing UI
	std_msgs::String msg;
    std::stringstream ss;
    ss << "/home/turtlebot/turtlebot_ws/src/duelingturtlebot/src/youwin.jpg";
    msg.data = ss.str();
    UI_pub.publish(msg);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "publish_velocity");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);
    UI_pub = nh.advertise<std_msgs::String>("duelingturtlebotUI", 1000);
    ros::Subscriber objsub = nh.subscribe<PointCloud>("/camera/depth/points", 1, PointCloud_Callback);
    ros::Subscriber blobsub = nh.subscribe<cmvision::Blobs>("/blobs", 100, BlobCallBack);
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
        else if(state == IWIN){
        	IWinState();
        }   
        else if(state == YOUWIN){
        	YouWinState();
        }        
        rate.sleep(); 
        ros::spinOnce();
    }
}
