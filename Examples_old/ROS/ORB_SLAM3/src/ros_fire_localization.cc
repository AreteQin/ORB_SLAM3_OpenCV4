/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include "vision_msgs/Detection2DArray.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <glog/logging.h>

#include"../../../include/System.h"
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

using namespace std;

void ImageBoxesCallback(ORB_SLAM3::System *pSLAM, const sensor_msgs::ImageConstPtr &msg,
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

//    pSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
    pSLAM->TrackMonocularAndFire(cv_ptr->image, cv_ptr->header.stamp.toSec(), fire_spots);
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

//    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    // message filter for images
    image_transport::ImageTransport it(nodeHandler);
    image_transport::SubscriberFilter sub_image(it, "/dji_osdk_ros/main_wide_RGB", 1);
    message_filters::Subscriber <vision_msgs::Detection2DArray> sub_fire_spot;
    sub_fire_spot.subscribe(nodeHandler, "/bounding_boxes/fire_spots", 1);

    // Sync the subscribed data
    message_filters::TimeSynchronizer <sensor_msgs::Image, vision_msgs::Detection2DArray>
            sync(sub_image, sub_fire_spot, 100);
    sync.registerCallback(boost::bind(&ImageBoxesCallback, &SLAM, _1, _2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}