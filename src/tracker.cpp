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
#include <people_msgs/PositionMeasurementArray.h>
#include <people_msgs/PositionMeasurement.h>
#include <ctime>

// speed 
const double z_scale_ = 0.3;
const double x_scale_ = 5;

// position of the target person
double target_x = 0;
double target_z = 0;

// timer for changing between the two random states
time_t ro_starting_time = 0;
time_t trans_starting_time = 0;
int t = 5;

// timer for sound_play
time_t wru_starting_time = 0;
time_t follow_starting_time = 0;

// people detection result is not consistant, change state only when the following variables reach the max count
int empty_count = 0;
const int max_empty_count = 5;
int single_count = 0;
const int max_single_count = 3;
int multiple_count = 0;
const int max_multiple_count = 2;

// states for FSM
enum STATE {RANDOM_RO, RANDOM_TRANS, FOLLOW_PEOPLE};
STATE state = RANDOM_RO;

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

    if(state ==  RANDOM_TRANS && count > 4000){
         state = RANDOM_RO;
        ro_starting_time = time(NULL);
    }

    else if(state == FOLLOW_PEOPLE){
        target_x = x/count;
        target_z = z - goal_z_;
    }
}

void People_Callback(const people_msgs::PositionMeasurementArray::ConstPtr& peopleMsg){
    double people_pos_threshold = 7;
    sound_play::SoundClient people_sc;

    if(peopleMsg->people.size() > 0){
        std::cout<< peopleMsg->people.size() <<" leg(s) detected" << std::endl;

        double count = 0;

        // find the cloest person
        for(int i = 0; i < peopleMsg->people.size(); i++){
            double distance;

            distance = sqrt(pow(peopleMsg->people[i].pos.x, 2)+pow(peopleMsg->people[i].pos.y, 2));
            if(distance <= people_pos_threshold){
                count++;
            }
        }

        if(count == 0){
            empty_count++;
            single_count = 0;
            multiple_count = 0;
            if(empty_count >= max_empty_count && state == FOLLOW_PEOPLE){
                state = RANDOM_RO;
                people_sc.say("I lost you!");
                ro_starting_time = time(NULL);
            }
        }

        else if(count <= 2){
            empty_count = 0;
            single_count++;
            multiple_count = 0;
            if(single_count >= max_single_count && state!=FOLLOW_PEOPLE){
                state = FOLLOW_PEOPLE;
                people_sc.say("I found you!");
                std::cout<< "changing state to follow people" << std::endl;
            }
        }

        else{
            empty_count = 0;
            single_count = 0;
            multiple_count++;
            if(multiple_count >= max_multiple_count && state == FOLLOW_PEOPLE){
                state = RANDOM_RO;
                people_sc.say("Ahhh too many people!");
                ro_starting_time = time(NULL);
            }
        }
    }
    else{
        empty_count++;
        single_count = 0;
        multiple_count = 0;
        if(empty_count >= max_empty_count && state == FOLLOW_PEOPLE){
            state = RANDOM_RO;
            people_sc.say("I lost you!");
            ro_starting_time = time(NULL);
        }
        std::cout<< "empty" << std::endl;
    }
}

void RandomRotationState(){
    time_t cur_time = time(NULL);
    if(cur_time - ro_starting_time > t){
        state = RANDOM_TRANS;
        trans_starting_time = cur_time;
    }

    geometry_msgs::Twist msg;
    
    msg.angular.z = 0.08 * x_scale_;
    pub.publish(msg);
}

void RandomTransState(){
    time_t cur_time = time(NULL);
    if(cur_time - trans_starting_time > t){
        state = RANDOM_TRANS;
        ro_starting_time = cur_time;
    }

    geometry_msgs::Twist msg;
    
    msg.linear.x = 0.5 * z_scale_;
    pub.publish(msg);

}

void FollowPeopleState(double x, double z){
    geometry_msgs::Twist msg;
    
    msg.angular.z = -x * x_scale_;
    msg.linear.x = z * z_scale_;
    pub.publish(msg);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "publish_velocity");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);
    ros::Subscriber objsub = nh.subscribe<PointCloud>("/camera/depth/points", 1, PointCloud_Callback);
    ros::Subscriber pplsub = nh.subscribe<people_msgs::PositionMeasurementArray>("/leg_tracker_measurements", 1, People_Callback);
    sound_play::SoundClient sc;
    srand(time(0));
    ros::Rate rate(2);
    ro_starting_time = time(NULL);
    wru_starting_time = time(NULL);
    while(ros::ok()){
        if(state == RANDOM_RO){
        	time_t cur_time = time(NULL);
        	if(cur_time - wru_starting_time >= t){
        		// sc.say("where are you");
        		wru_starting_time = cur_time; 
        	}
            RandomRotationState();
        }
        if(state == RANDOM_TRANS){
        	time_t cur_time = time(NULL);
        	if(cur_time - wru_starting_time >= t){
        		// sc.say("where are you");
        		wru_starting_time = cur_time; 
        	}
            RandomTransState();
        }
        else if(state == FOLLOW_PEOPLE){
        	time_t cur_time = time(NULL);
        	if(cur_time - follow_starting_time >= t){
        		// sc.say("following you");
        		follow_starting_time = cur_time; 
        	}
            FollowPeopleState(target_x, target_z);
        }    
        rate.sleep(); 
        ros::spinOnce();
    }
}
