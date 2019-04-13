#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

ros::Publisher pub1;
ros::Publisher pub2;

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
sensor_msgs::PointCloud2::Ptr final_cloud (new sensor_msgs::PointCloud2);

double m_groundFilterPlaneDistance = .5;

PCLPointCloud ground, nonground;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);

  pcl::fromROSMsg (*input, *cloud_p);

  // pcl::ExtractIndices<pcl::PointXYZ> extract;

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold(.5);
  seg.setAxis(Eigen::Vector3f(0,0,1));
  seg.setDistanceThreshold (0.01);
 
  // TODO
  PCLPointCloud& pc = cloud;
  //
  PCLPointCloud cloud_filtered(pc);

  pcl::ExtractIndices<PCLPoint> extract;
  bool groundPlaneFound = false;


  while (cloud_filtered.size() > 10 && !groundPlaneFound) {
      seg.setInputCloud(cloud_filtered.makeShared());
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0){
        ROS_INFO("PCL segmentation did not find any plane.");
        break;
      }

      extract.setInputCloud(cloud_filtered.makeShared());
      extract.setIndices(inliers);

      if (std::abs(coefficients->values.at(3)) < m_groundFilterPlaneDistance){
        ROS_DEBUG("Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(), cloud_filtered.size(),
                  coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
        extract.setNegative (false);
        extract.filter (ground);

        // remove ground points from full pointcloud:
        // workaround for PCL bug:
        if(inliers->indices.size() != cloud_filtered.size()){
          extract.setNegative(true);
          PCLPointCloud cloud_out;
          extract.filter(cloud_out);
          nonground += cloud_out;
          cloud_filtered = cloud_out;
        }

        groundPlaneFound = true;
      } else{
        ROS_DEBUG("Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(), cloud_filtered.size(),
                  coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
        pcl::PointCloud<PCLPoint> cloud_out;
        extract.setNegative (false);
        extract.filter(cloud_out);

        // remove current plane from scan for next iteration:
        // workaround for PCL bug:
        if(inliers->indices.size() != cloud_filtered.size()){
          extract.setNegative(true);
          cloud_out.points.clear();
          extract.filter(cloud_out);
          cloud_filtered = cloud_out;
        } else{
          cloud_filtered.points.clear();
        }
      }

    }
    // TODO: also do this if overall starting pointcloud too small?
    if (!groundPlaneFound){ // no plane found or remaining points too small
      ROS_WARN("No ground plane found in scan");

      // do a rough fitlering on height to prevent spurious obstacles
      pcl::PassThrough<PCLPoint> second_pass;
      second_pass.setFilterFieldName("z");
      second_pass.setFilterLimits(-m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);
      second_pass.setInputCloud(pc.makeShared());
      second_pass.filter(ground);

      second_pass.setFilterLimitsNegative (true);
      second_pass.filter(nonground);
    }

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(cloud_filtered, output);
  pub1.publish(cloud_filtered);


  /*
  seg.setInputCloud (cloud.makeShared ());
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
  }

  extract.setInputCloud (cloud.makeShared());
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud_p);
  std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

  pcl::toROSMsg (*cloud_p, *final_cloud);

  pub1.publish (final_cloud); // publish the new pcl

  */

  // Publish the model coefficients
  /*pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(coefficients, ros_coefficients);
  pub2.publish (ros_coefficients);
  */

  /*
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);
 
  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.5, 0.5, 0.5);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_out, output);

  // Publish the data
  pub.publish (output);
  */
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub1 = nh.advertise<sensor_msgs::PointCloud2> ("output2", 1);
  pub2 = nh.advertise<pcl_msgs::ModelCoefficients> ("testing", 1);

  // Spin
  ros::spin ();
}
