#include <iostream>
#include <chrono>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include "vision_msgs/Detection2DArray.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <glog/logging.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include "../../../include/System.h"
#include "../../../include/Map.h"

using namespace std;

void ImageBoxesCallback(ORB_SLAM3::System *pSLAM, ros::Publisher *fire_spots_pub, ros::Publisher *camera_pose_pub,
                        const sensor_msgs::ImageConstPtr &msg,
                        const vision_msgs::Detection2DArrayConstPtr &msg_fire_spot) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // convert dectection2DArray to vector<cv::Rect>
    vector<cv::Rect> fire_spots;
//    LOG(INFO) << "fire spots number: " << msg_fire_spot->detections.size();
    for (auto &box: msg_fire_spot->detections) {
        fire_spots.emplace_back(box.bbox.center.x - box.bbox.size_x / 2,
                                box.bbox.center.y - box.bbox.size_y / 2,
                                box.bbox.size_x, box.bbox.size_y);
    }

    pSLAM->TrackMonocularAndFire(cv_ptr->image, cv_ptr->header.stamp.toSec(), fire_spots);
    ORB_SLAM3::Map *current_map = pSLAM->GetActiveMap();

    // Publish the fire spots
    geometry_msgs::PoseArray fire_spots_msg;
    fire_spots_msg.header.stamp = msg->header.stamp;
    fire_spots_msg.header.frame_id = "map";
    for (Eigen::Vector3f &fire_spot: current_map->mvFireSpots) {
        geometry_msgs::Pose pose;
        pose.position.x = fire_spot[0];
        pose.position.y = fire_spot[1];
        pose.position.z = fire_spot[2];
        fire_spots_msg.poses.push_back(pose);
    }
    fire_spots_pub->publish(fire_spots_msg);

    // Publish the camera pose
    geometry_msgs::PoseStamped camera_pose_msg;
    camera_pose_msg.header.stamp = msg->header.stamp;
    camera_pose_msg.header.frame_id = "map";
    Eigen::Matrix4f Twc = pSLAM->GetCurrentPose().inverse();
    camera_pose_msg.pose.position.x = Twc(0, 3);
    camera_pose_msg.pose.position.y = Twc(1, 3);
    camera_pose_msg.pose.position.z = Twc(2, 3);
    Eigen::Quaternionf q(Twc.block<3, 3>(0, 0));
    camera_pose_msg.pose.orientation.x = q.x();
    camera_pose_msg.pose.orientation.y = q.y();
    camera_pose_msg.pose.orientation.z = q.z();
    camera_pose_msg.pose.orientation.w = q.w();
    camera_pose_pub->publish(camera_pose_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fire_localization");
    ros::start();

    if (argc != 3) {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    ros::NodeHandle nodeHandler;

    // message filter for images
    image_transport::ImageTransport it(nodeHandler);
    image_transport::SubscriberFilter sub_image(it, "/dji_osdk_ros/main_wide_RGB", 1);
    message_filters::Subscriber<vision_msgs::Detection2DArray> sub_fire_spot;
    sub_fire_spot.subscribe(nodeHandler, "/bounding_boxes/fire_spots", 1);

    // Publish fire spots
    ros::Publisher fire_spots_pub = nodeHandler.advertise<geometry_msgs::PoseArray>("/position/fire_spots", 10);

    // Publish camera pose
    ros::Publisher camera_pose_pub = nodeHandler.advertise<geometry_msgs::PoseStamped>("/position/camera_pose", 10);

    // Sync the subscribed data
    message_filters::TimeSynchronizer<sensor_msgs::Image, vision_msgs::Detection2DArray>
            sync(sub_image, sub_fire_spot, 100);
    sync.registerCallback(boost::bind(&ImageBoxesCallback, &SLAM, &fire_spots_pub, &camera_pose_pub, _1, _2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}