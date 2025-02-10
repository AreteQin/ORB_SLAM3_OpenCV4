#include <iostream>
#include <chrono>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include "vision_msgs/Detection2DArray.h"
#include <message_filters/subscriber.h>
#include <glog/logging.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <geometry_msgs/PoseArray.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "MapPoint.h"
#include "System.h"
#include "Map.h"

using namespace std;

visualization_msgs::Marker drone_model;

sensor_msgs::PointCloud2 convertMapPointsToPointCloud2(const std::vector<ORB_SLAM3::MapPoint*>& mapPoints,
                                                       const std_msgs::Header& header)
{
    sensor_msgs::PointCloud2 pointCloudMsg;
    pointCloudMsg.header = header;
    pointCloudMsg.height = 1;
    pointCloudMsg.width = mapPoints.size();
    pointCloudMsg.is_dense = false;
    pointCloudMsg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(pointCloudMsg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(mapPoints.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(pointCloudMsg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pointCloudMsg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pointCloudMsg, "z");

    for (const auto& mapPoint : mapPoints)
    {
        if (mapPoint && !mapPoint->isBad())
        {
            const Eigen::Vector3f& pos = mapPoint->GetWorldPos();
            *iter_x = pos.x();
            *iter_y = pos.y();
            *iter_z = pos.z();
        }
        else
        {
            *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        }
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }
    return pointCloudMsg;
}

void ImageBoxesCallback(ORB_SLAM3::System* pSLAM, ros::Publisher* fire_spots_pub, tf2_ros::TransformBroadcaster& br,
                        ros::Publisher* point_cloud_pub, ros::Publisher* marker_pub, const sensor_msgs::ImageConstPtr& msg,
                        const vision_msgs::Detection2DArrayConstPtr& msg_fire_spot)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // convert dectection2DArray to vector<cv::Rect>
    vector<cv::Rect> fire_spots;
    //    LOG(INFO) << "fire spots number: " << msg_fire_spot->detections.size();
    for (auto& box : msg_fire_spot->detections)
    {
        fire_spots.emplace_back(box.bbox.center.x - box.bbox.size_x / 2,
                                box.bbox.center.y - box.bbox.size_y / 2,
                                box.bbox.size_x, box.bbox.size_y);
    }

    pSLAM->TrackMonocularAndFire(cv_ptr->image, cv_ptr->header.stamp.toSec(), fire_spots);
    ORB_SLAM3::Map* current_map = pSLAM->GetAtlas()->GetCurrentMap();
    if (!current_map)
        return;

    // Publish the fire spots
    geometry_msgs::PoseArray fire_spots_msg;
    fire_spots_msg.header.stamp = msg->header.stamp;
    fire_spots_msg.header.frame_id = "map";
    for (Eigen::Vector3f& fire_spot : current_map->mvFireSpots)
    {
        geometry_msgs::Pose pose;
        pose.position.x = fire_spot[0];
        pose.position.y = fire_spot[1];
        pose.position.z = fire_spot[2];
        fire_spots_msg.poses.push_back(pose);
    }
    fire_spots_pub->publish(fire_spots_msg);

    // Publish the camera pose as tf2
    geometry_msgs::TransformStamped camera_pose_msg;
    camera_pose_msg.header.stamp = msg->header.stamp;
    camera_pose_msg.header.frame_id = "map";
    camera_pose_msg.child_frame_id = "H20T";
    Eigen::Matrix4f Twc = pSLAM->GetCurrentPose().inverse();
    camera_pose_msg.transform.translation.x = Twc(0, 3);
    camera_pose_msg.transform.translation.y = Twc(1, 3);
    camera_pose_msg.transform.translation.z = Twc(2, 3);
    Eigen::Quaternionf q(Twc.block<3, 3>(0, 0));
    camera_pose_msg.transform.rotation.x = q.x();
    camera_pose_msg.transform.rotation.y = q.y();
    camera_pose_msg.transform.rotation.z = q.z();
    camera_pose_msg.transform.rotation.w = q.w();

    br.sendTransform(camera_pose_msg);

    // broadcast the static transform of map which is -45 degree rotated around x-axis
    geometry_msgs::TransformStamped map;
    map.header.stamp = msg->header.stamp;
    map.header.frame_id = "world";
    map.child_frame_id = "map";
    map.transform.translation.x = 0;
    map.transform.translation.y = 0;
    map.transform.translation.z = 0.1;
    tf2::Quaternion q_map;
    q_map.setRPY(-2.1, 0.05, 0);
    map.transform.rotation.x = q_map.x();
    map.transform.rotation.y = q_map.y();
    map.transform.rotation.z = q_map.z();
    map.transform.rotation.w = q_map.w();
    br.sendTransform(map);

    // Publish the point cloud
    const vector<ORB_SLAM3::MapPoint*>& vpMPs = current_map->GetAllMapPoints();
    if (vpMPs.empty())
        return;
    sensor_msgs::PointCloud2 point_cloud_msg;
    point_cloud_msg.header.stamp = msg->header.stamp;
    point_cloud_msg.header.frame_id = "map";
    point_cloud_msg = convertMapPointsToPointCloud2(vpMPs, point_cloud_msg.header);
    point_cloud_pub->publish(point_cloud_msg);

    drone_model.header.frame_id = "H20T";
    drone_model.header.stamp = msg->header.stamp;
    drone_model.ns = "fbx_marker";
    drone_model.id = 0;
    drone_model.type = visualization_msgs::Marker::MESH_RESOURCE;
    if (std::ifstream("/home/qin/ORB_SLAM3_Ubuntu_20/drone.dae"))
    {
        drone_model.mesh_resource = "file:///home/qin/ORB_SLAM3_Ubuntu_20/drone.dae";
    }
    else
    {
        LOG(ERROR) << "Mesh resource file not found: /home/qin/ORB_SLAM3_Ubuntu_20/drone.dae";
    }
    drone_model.action = visualization_msgs::Marker::ADD;
    drone_model.pose.position.x = 0.0;
    drone_model.pose.position.y = -0.2;
    drone_model.pose.position.z = -0.5;
    drone_model.color.r = 0.5;
    drone_model.color.g = 0.5;
    drone_model.color.b = 0.5;
    drone_model.color.a = 1.0;
    drone_model.scale.x = 0.1;
    drone_model.scale.y = 0.1;
    drone_model.scale.z = 0.1;
    // rotate the model
    // Create a quaternion representing the rotation
    tf2::Quaternion quaternion;
    quaternion.setRPY(1.0, 3.14, 0.0); // Roll, Pitch, Yaw
    // Convert the quaternion to a geometry_msgs::Quaternion
    geometry_msgs::Quaternion q_msg;
    q_msg.x = quaternion.x();
    q_msg.y = quaternion.y();
    q_msg.z = quaternion.z();
    q_msg.w = quaternion.w();
    // Set the orientation of the drone model marker
    drone_model.pose.orientation = q_msg;
    marker_pub->publish(drone_model);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fire_localization");
    ros::start();

    if (argc != 3)
    {
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
    // ros::Publisher camera_pose_pub = nodeHandler.advertise<geometry_msgs::PoseStamped>("/position/camera_pose", 10);
    tf2_ros::TransformBroadcaster br;
    // Publish point cloud
    ros::Publisher point_cloud_pub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/point_cloud", 10);
    // Initialize marker publisher
    ros::Publisher marker_pub = nodeHandler.advertise<visualization_msgs::Marker>("/visualization_marker/drone", 10);

    // Sync the subscribed data
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, vision_msgs::Detection2DArray>
        MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_image, sub_fire_spot);
    sync.registerCallback(boost::bind(&ImageBoxesCallback, &SLAM, &fire_spots_pub, boost::ref(br), &point_cloud_pub,
                                      &marker_pub, _1, _2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}
