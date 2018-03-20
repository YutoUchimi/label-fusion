// Copyright (2018) Yuto Uchimi

#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

#include <boost/assign.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <jsk_topic_tools/log_utils.h>
#include <octomap/CountingOcTree.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "label_fusion_ros/mask_fusion_nodelet.h"


namespace label_fusion_ros {

  void MaskFusion::onInit() {
    DiagnosticNodelet::onInit();
    pnh_->param<std::string>("frame_id", frame_id_, "map");
    pnh_->param("approximate_sync", approximate_sync_, true);
    pnh_->param("queue_size", queue_size_, 10);
    pnh_->param("use_depth", use_depth_, false);
    pnh_->param("resolution", resolution_, 0.01);
    pnh_->param("threshold", threshold_, 0.95);
    pnh_->param("ksize", ksize_, 10);
    pub_cloud_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    int n_views = 0;
    resolution = resolution_;
    threshold = threshold_;
    ksize = ksize_;
    octree = new octomap::CountingOcTree(resolution);
    onInitPostProcess();
  }

  void MaskFusion::subscribe() {
    sub_mask_.subscribe(*pnh_, "input", 1);
    sub_info_.subscribe(*pnh_, "input/info", 1);
    sub_transform_.subscribe(*pnh_, "input/transform", 1);
    if (use_depth_) {
      sub_depth_.subscribe(*pnh_, "input/depth", 1);
      if (approximate_sync_) {
        async_depth_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicyWithDepth> >(queue_size_);
        async_depth_->connectInput(sub_mask_, sub_info_, sub_transform_, sub_depth_);
        async_depth_->registerCallback(boost::bind(&MaskFusion::fusion, this, _1, _2, _3, _4));
      } else {
        sync_depth_ = boost::make_shared<message_filters::Synchronizer<SyncPolicyWithDepth> >(queue_size_);
        sync_depth_->connectInput(sub_mask_, sub_info_, sub_transform_, sub_depth_);
        sync_depth_->registerCallback(boost::bind(&MaskFusion::fusion, this, _1, _2, _3, _4));
      }
    } else {
      if (approximate_sync_) {
        async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(queue_size_);
        async_->connectInput(sub_mask_, sub_info_, sub_transform_);
        async_->registerCallback(boost::bind(&MaskFusion::fusion, this, _1, _2, _3));
      } else {
        sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);
        sync_->connectInput(sub_mask_, sub_info_, sub_transform_);
        sync_->registerCallback(boost::bind(&MaskFusion::fusion, this, _1, _2, _3));
      }
    }
    ros::V_string names = boost::assign::list_of("~input")("~input/info")("~input/transform");
    jsk_topic_tools::warnNoRemap(names);
  }

  void MaskFusion::unsubscribe() {
    sub_mask_.unsubscribe();
    sub_info_.unsubscribe();
    sub_transform_.unsubscribe();
    if (use_depth_) {
      sub_depth_.unsubscribe();
    }
    delete octree;
  }

  void MaskFusion::fusion(const sensor_msgs::Image::ConstPtr& mask_msg,
                          const sensor_msgs::CameraInfo::ConstPtr& info_msg,
                          const geometry_msgs::TransformStamped::ConstPtr& transform_msg) {
    fusion(mask_msg, info_msg, transform_msg, sensor_msgs::ImageConstPtr());
  }

  void MaskFusion::fusion(const sensor_msgs::Image::ConstPtr& mask_msg,
                          const sensor_msgs::CameraInfo::ConstPtr& info_msg,
                          const geometry_msgs::TransformStamped::ConstPtr& transform_msg,
                          const sensor_msgs::Image::ConstPtr& depth_msg) {
    n_views++;

    // cam_info: intrinsic parameter of color camera
    Eigen::Matrix3f cam_K;
    cam_K <<
      info_msg->K[0], info_msg->K[1], info_msg->K[2],
      info_msg->K[3], info_msg->K[4], info_msg->K[5],
      info_msg->K[6], info_msg->K[7], info_msg->K[8];

    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    cv::Mat mask = cv_bridge::toCvShare(mask_msg, "mono8")->image;

    cv::Mat depth;
    if (use_depth_) {
      depth = cv_bridge::toCvShare(depth_msg, depth_msg->encoding)->image;
    }

    // cam_pose: world -> camera
    float tx = transform_msg->transform.translation.x;
    float ty = transform_msg->transform.translation.y;
    float tz = transform_msg->transform.translation.z;
    float rx = transform_msg->transform.rotation.x;
    float ry = transform_msg->transform.rotation.y;
    float rz = transform_msg->transform.rotation.z;
    float rw = transform_msg->transform.rotation.w;
    float r00 = 1 - 2*ry*ry - 2*rz*rz;
    float r01 = 2*rx*ry - 2*rz*rw;
    float r02 = 2*rx*rz + 2*ry*rw;
    float r10 = 2*rx*ry + 2*rz*rw;
    float r11 = 1 - 2*rx*rx - 2*rz*rz;
    float r12 = 2*ry*rz - 2*rw*rx;
    float r20 = 2*rx*rz - 2*ry*rw;
    float r21 = 2*ry*rz + 2*rw*rx;
    float r22 = 1 - 2*rx*rx - 2*ry*ry;
    Eigen::Matrix4f cam_pose;
    cam_pose <<
      r00, r01, r02, tx,
      r10, r11, r12, ty,
      r20, r21, r22, tz,
      0,   0,   0,   1;

    // camera origin
    Eigen::Vector4f origin_(0, 0, 0, 1);
    origin_ = cam_pose * origin_;
    Eigen::Vector3f origin(origin_(0), origin_(1), origin_(2));

    // visualize camera origin
    pcl::PointXYZRGB pt_red(255, 0, 0);
    pt_red.x = origin(0);
    pt_red.y = origin(1);
    pt_red.z = origin(2);
    cloud.push_back(pt_red);

    octomap::KeySet occupied_cells;
    octomap::KeySet unoccupied_cells;
#pragma omp parallel for
    for (int v = 0; v < mask.rows; v += ksize) {
      for (int u = 0; u < mask.cols; u += ksize) {
        float d = std::numeric_limits<float>::quiet_NaN();
        if (use_depth_) {
          d = depth.at<float>(v, u);
        }

        Eigen::Vector3f uv(u, v, 1);
        uv = cam_K.inverse() * uv;
        Eigen::Vector4f direction_(uv(0), uv(1), uv(2), 1);  // with depth
        Eigen::Vector4f direction_far_(direction_(0), direction_(1), direction_(2), 1);  // without depth
        if (!std::isnan(d)) {
          direction_(0) *= d;
          direction_(1) *= d;
          direction_(2) = d;
        }

        direction_ = cam_pose * direction_;
        direction_far_ = cam_pose * direction_far_;
        Eigen::Vector3f direction(direction_(0), direction_(1), direction_(2));
        Eigen::Vector3f direction_far(direction_far_(0), direction_far_(1), direction_far_(2));

        // visualize ray direction
        pcl::PointXYZRGB pt_blue(0, 0, 255);
        pt_blue.x = direction_far(0);
        pt_blue.y = direction_far(1);
        pt_blue.z = direction_far(2);
#pragma omp critical
        cloud.push_back(pt_blue);

        octomap::point3d pt_origin(origin(0), origin(1), origin(2));
        octomap::point3d pt_direction(direction(0), direction(1), direction(2));
        octomap::point3d pt_direction_far(direction_far(0), direction_far(1), direction_far(2));
        if (mask.at<unsigned char>(v, u) > 127) {
          octomap::KeyRay key_ray;
          octree->computeRayKeys(pt_origin, pt_direction_far, key_ray);
#pragma omp critical
          occupied_cells.insert(key_ray.begin(), key_ray.end());
        }
        if (!std::isnan(d)) {
          octomap::KeyRay key_ray;
          if (octree->computeRayKeys(pt_origin, pt_direction, key_ray)) {
#pragma omp critical
            unoccupied_cells.insert(key_ray.begin(), key_ray.end());
          }
        }
      }  // for (int u = 0; ...)
    }  // for (int v = 0; ...)

    for (octomap::KeySet::iterator it = unoccupied_cells.begin(), end = unoccupied_cells.end(); it != end; ++it) {
      octree->updateNode(*it, /*hit=*/false, /*reset=*/true);
    }
    for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it) {
      if (unoccupied_cells.find(*it) == unoccupied_cells.end()) {
        octree->updateNode(*it, /*hit=*/true);
      }
    }

    // visualize 3d segmentation
    octomap::point3d_list node_centers;
    if (octree->size() != 0) {
      octree->getCentersMinHits(node_centers, static_cast<int>(threshold * n_views));
    }
    for (octomap::point3d_list::iterator it = node_centers.begin(), end = node_centers.end(); it != end; ++it) {
      pcl::PointXYZRGB pt_green(0, 255, 0);
      pt_green.x = (*it).x();
      pt_green.y = (*it).y();
      pt_green.z = (*it).z();
      cloud.push_back(pt_green);
    }

    sensor_msgs::PointCloud2 output_cloud_msg;
    pcl::toROSMsg(cloud, output_cloud_msg);
    output_cloud_msg.header = info_msg->header;
    output_cloud_msg.header.frame_id = frame_id_;
    pub_cloud_.publish(output_cloud_msg);
    cloud.clear();
  }

}  // namespace label_fusion_ros

PLUGINLIB_EXPORT_CLASS(label_fusion_ros::MaskFusion, nodelet::Nodelet);
