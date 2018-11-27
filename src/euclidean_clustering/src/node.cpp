#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/point_cloud.h>
#include <dynamic_reconfigure/server.h>
#include <euclidean_clustering/EuclideanClusteringConfig.h>
#include <std_msgs/Int32.h>

using euclidean_clustering::EuclideanClusteringConfig;

template <typename PointT>
class EuclideanClustering
{
public:

  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;

  EuclideanClustering(const ros::NodeHandle nh, bool use_rgb) : nh_(nh), use_rgb_(use_rgb)
  {
    nh_.param("max_queue_size", max_queue_size_, 100);
    f_ = [this](EuclideanClusteringConfig& config, uint32_t level) { return this->config_callback(config, level); };
    server_.setCallback(f_);
    clustering_tolerance_ = 0.02;
    min_cluster_size_ = 100;
    max_cluster_size_ = 100000;
    pub_ = nh_.advertise<PointCloud>("centroids", 1);
    sub_ = nh_.subscribe<PointCloud>("cloud_in", 1, &EuclideanClustering::cloud_callback, this);
  }

  void config_callback(EuclideanClusteringConfig& config, uint32_t level) {
    clustering_tolerance_ = config.cluster_tolerance;
    min_cluster_size_ = config.min_cluster_size;
    max_cluster_size_ = config.max_cluster_size;
  }

  void cloud_callback(const typename PointCloud::ConstPtr& input)
  {
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    PointCloudPtr output_ptr(new PointCloud);
    output_ptr->header = input->header;

    ec.setClusterTolerance(clustering_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input);
    ec.extract(cluster_indices);

    for (auto cit = cluster_indices.begin(); cit != cluster_indices.end(); ++cit)
    {
      pcl::CentroidPoint<PointT> centroid;
      PointT output_point;
      for (auto pit = cit->indices.begin(); pit != cit->indices.end(); ++pit)
      {
        centroid.add(input->points[*pit]);
      }
      centroid.get(output_point);
      output_ptr->push_back(output_point);
    }
    pub_.publish(output_ptr);
  }

private:
  bool use_rgb_;
  ros::NodeHandle nh_;
  int max_queue_size_;
  dynamic_reconfigure::Server<EuclideanClusteringConfig>::CallbackType f_;
  dynamic_reconfigure::Server<EuclideanClusteringConfig> server_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  double clustering_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "euclidean_clustering");
  ros::NodeHandle nh("~");
  bool use_rgb;
  nh.param("use_rgb", use_rgb, false);
  if (use_rgb)
  {
    EuclideanClustering<pcl::PointXYZRGB> euclidean_clustering(nh, use_rgb);
    ros::spin();
  }
  else
  {
    EuclideanClustering<pcl::PointXYZ> euclidean_clustering(nh, use_rgb);
    ros::spin();
  }
  return 0;
}
