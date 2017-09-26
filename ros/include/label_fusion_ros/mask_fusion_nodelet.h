#ifndef LABEL_FUSION_ROS_MASK_FUSION_NODELET_H_
#define LABEL_FUSION_ROS_MASK_FUSION_NODELET_H_

#include <geometry_msgs/TransformStamped.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

namespace label_fusion_ros
{

  class MaskFusion: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,
      sensor_msgs::CameraInfo,
      geometry_msgs::TransformStamped > ApproximateSyncPolicy;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Image,
      sensor_msgs::CameraInfo,
      geometry_msgs::TransformStamped > SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,
      sensor_msgs::CameraInfo,
      geometry_msgs::TransformStamped,
      sensor_msgs::Image > ApproximateSyncPolicyWithDepth;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Image,
      sensor_msgs::CameraInfo,
      geometry_msgs::TransformStamped,
      sensor_msgs::Image > SyncPolicyWithDepth;
    MaskFusion(): DiagnosticNodelet("MaskFusion") { }
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void fusion(const sensor_msgs::Image::ConstPtr& mask_msg,
                        const sensor_msgs::CameraInfo::ConstPtr& info_msg,
                        const geometry_msgs::TransformStamped::ConstPtr& transform_msg);
    virtual void fusion(const sensor_msgs::Image::ConstPtr& mask_msg,
                        const sensor_msgs::CameraInfo::ConstPtr& info_msg,
                        const geometry_msgs::TransformStamped::ConstPtr& transform_msg,
                        const sensor_msgs::Image::ConstPtr& depth_msg);

    boost::mutex mutex_;

    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > async_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicyWithDepth> > sync_depth_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicyWithDepth> > async_depth_;

    message_filters::Subscriber<sensor_msgs::Image> sub_mask_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
    message_filters::Subscriber<geometry_msgs::TransformStamped> sub_transform_;
    message_filters::Subscriber<sensor_msgs::Image> sub_depth_;

    // ros::Subscriber sub_mask_;
    // ros::Subscriber sub_info_;
    // ros::Subscriber sub_transform_;
    // ros::Subscriber sub_depth_;
    ros::Publisher pub_cloud_;

    std::string frame_id_;
    bool approximate_sync_;
    int queue_size_;
    bool use_depth_;
    int n_views_;
    double resolution_;
    double threshold_;
    int ksize_;
    int n_views;
    double resolution;
    double threshold;
    int ksize;
  private:
  };

}  // namespace label_fusion_ros

#endif // LABEL_FUSION_ROS_MASK_FUSION_NODELET_H_
