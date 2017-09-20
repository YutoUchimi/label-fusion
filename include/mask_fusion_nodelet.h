#ifndef LABEL_FUSION_MASK_FUSION_NODELET_H_
#define LABEL_FUSION_MASK_FUSION_NODELET_H_

#include <geometry_msgs/TransformStamped.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

namespace label_fusion
{

  class MaskFusion: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
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

    ros::Subscriber sub_mask_;
    ros::Subscriber sub_info_;
    ros::Subscriber sub_transform_;
    ros::Subscriber sub_depth_;
    ros::Publisher pub_cloud_;
  private:
  };

}  // namespace label_fusion

#endif // LABEL_FUSION_MASK_FUSION_NODELET_H_
