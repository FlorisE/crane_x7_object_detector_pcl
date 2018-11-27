#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>

template <typename PointT>
class FrameRadiusSubtractor {
public:

    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;

    FrameRadiusSubtractor(const ros::NodeHandle nh, bool use_rgb) : nh_(nh), tf_listener_(tf_buffer_), use_rgb_(use_rgb) {
        nh_.param("base_frame", base_frame_, std::string());
        nh_.param("subtraction_frame", subtraction_frame_, std::string());
        nh_.param("radius", radius_, 0.0f);
        nh_.param("max_queue_size", max_queue_size_, 100);
        pub_ = nh_.advertise<PointCloud>("output", 1);
        ROS_DEBUG("Waiting for transform between base frame and subtraction frame");
        base_frame_to_subtraction_frame_transform_ = tf_buffer_.lookupTransform(subtraction_frame_, base_frame_, ros::Time(0), ros::Duration(3.0));
        point_.x = base_frame_to_subtraction_frame_transform_.transform.translation.x; 
        point_.y = base_frame_to_subtraction_frame_transform_.transform.translation.z; 
        point_.z = base_frame_to_subtraction_frame_transform_.transform.translation.y; 
        sub_ = nh_.subscribe<PointCloud>("cloud_in", max_queue_size_, &FrameRadiusSubtractor::cloud_callback, this);
        extract_.setNegative(true);
    } 

    void cloud_callback(const typename PointCloud::ConstPtr& input) {
	pcl::PointIndices::Ptr indicesPtr(new pcl::PointIndices);
        std::vector<float> pointRadiusSquaredDistance;

        kdtree_.setInputCloud(input);

	//ROS_DEBUG_STREAM("base_frame: " << base_frame_ << ", subtraction_frame: " << subtraction_frame_ << ", radius: " << radius_ << ", max_queue_size: " << max_queue_size_);
        //ROS_DEBUG_STREAM("x: " << point_.x << ", y: " << point_.y << ", z: " << point_.z);
        if (kdtree_.radiusSearch(point_, radius_, indicesPtr->indices, pointRadiusSquaredDistance) > 0) {
            PointCloudPtr outputPtr(new PointCloud);
            extract_.setInputCloud(input);
            extract_.setIndices(indicesPtr);
            extract_.filter(*outputPtr);
            pub_.publish(outputPtr);
        } else {
            ROS_DEBUG("No indices found");
        }
    }

private:
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    bool use_rgb_;
    geometry_msgs::TransformStamped base_frame_to_subtraction_frame_transform_;
    ros::NodeHandle nh_;
    std::string base_frame_;
    std::string subtraction_frame_;
    float radius_;
    int max_queue_size_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    PointT point_;
    pcl::KdTreeFLANN<PointT> kdtree_;
    pcl::ExtractIndices<PointT> extract_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "clamp_extractor");
    ros::NodeHandle nh("~");
    bool use_rgb;
    nh.param("use_rgb", use_rgb, false);
    if (use_rgb) {
        FrameRadiusSubtractor<pcl::PointXYZRGB> frame_radius_subtractor(nh, use_rgb);
        ros::spin();
    } else {
        FrameRadiusSubtractor<pcl::PointXYZ> frame_radius_subtractor_rgb(nh, use_rgb);
        ros::spin();
    }

}
