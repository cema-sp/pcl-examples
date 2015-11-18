#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/lmeds.h>
// works fine (~500)
#include <pcl/sample_consensus/mlesac.h>
// gives errors (find all points)
#include <pcl/sample_consensus/msac.h>
// gives errors (Estimated distances (0) differs than the normal of indices)
#include <pcl/sample_consensus/rmsac.h>
// gives errors (Estimated distances (0) differs than the normal of indices)
#include <pcl/sample_consensus/ransac.h>
// works fine (1000+)
#include <pcl/sample_consensus/prosac.h>
// works fine (1100+)
#include <pcl/sample_consensus/rransac.h>
// finds nothing
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_handlers.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr initial_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr matched_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointXYZ o;
int r = -1;

void
viewer_matched_cloud(pcl::visualization::PCLVisualizer& viewer)
{
	int text_id(0);
  long cloud_size = matched_cloud->width * matched_cloud->height;
	std::stringstream info;
	info << "Points in matched cloud: " << cloud_size;
  std::cout << "Matched cloud rendered with " << cloud_size << " points" << std::endl;
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(initial_cloud, 0, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ> (matched_cloud, single_color, "matched cloud");
  viewer.addText(info.str(), 10, 20, "points", text_id);
  if (r > 0)
  {
    viewer.addSphere (o, r, "sphere", 0);
  }
}

void
viewer_initial_cloud(pcl::visualization::PCLVisualizer& viewer)
{
	//int text_id(0);
  long cloud_size = initial_cloud->width * initial_cloud->height;
	std::stringstream info;
	info << "Points in initial cloud: " << cloud_size;
  std::cout << "Initial cloud rendered with " << cloud_size << " points" << std::endl;
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(initial_cloud, 255, 0, 0);
  viewer.addPointCloud<pcl::PointXYZ> (initial_cloud, single_color, "initial cloud");
  viewer.addText(info.str(), 10, 20, "points", 0);
}

void
print_help (const char* name)
{
  std::cout << std::endl
            << "Usage: " << name << "[FILE.pcd] [options]" << std::endl
            << "Options:" << std::endl
            << "-------------------------------------------" << std::endl
            << "-h           This help" << std::endl
            << "-j           Just Visualize example" << std::endl
            << "-m           Match and visualize matched" << std::endl
            << "-e           Match and visualize remaining" << std::endl
            << "-b           Match and visualize both" << std::endl
            << "-a           -" << std::endl
            << "-v           -" << std::endl
            << "-i           -" << std::endl
            << std::endl;
}

int
main (int argc, char** argv)
{
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    print_help (argv[0]);
    return 0;
  }

  //pcl::PointCloud<pcl::PointXYZ>::Ptr initial_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr matched_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  //cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<int> pcd_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

  if (pcd_file_indices.size () != 1)
  {
    std::cout << "Need input PCD file" << std::endl;
    return (-1);
  }
  
  std::string pcd_file_name = argv[pcd_file_indices[0]];

  std::cout << "Reading file \"" << pcd_file_name << "\"" << std::endl;
  
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file_name, *initial_cloud) == -1) //* load the file
  {
    std::cerr << "Couldn't read file " << pcd_file_name << std::endl;
    return (-1);
  }

  std::cout << "Loaded "
            << initial_cloud->width * initial_cloud->height
            << " data points from "
            << pcd_file_name
            << std::endl;

  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

  if (pcl::console::find_argument (argc, argv, "-j") >= 0)
  {
    std::cout << "Just Visualize example" << std::endl;
    viewer.runOnVisualizationThreadOnce(viewer_initial_cloud);
  }
  else if (pcl::console::find_argument (argc, argv, "-m") >= 0)
  {
    std::cout << "Match and visualize matched" << std::endl;

		std::vector<int> inliers;
		//boost::shared_ptr< std::vector<int> > inliers_ptr(inliers);

		pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
		  model_sphere(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (initial_cloud, true));

		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_sphere);
		ransac.setDistanceThreshold (2);
		ransac.computeModel(1);
		ransac.getInliers(inliers);

		std::cout << "Found "
		          << inliers.size()
		          << " inliers with RANSAC "
		          << std::endl;

    pcl::copyPointCloud<pcl::PointXYZ>(*initial_cloud, inliers, *matched_cloud);

    viewer.runOnVisualizationThreadOnce(viewer_matched_cloud);
  }
  else if (pcl::console::find_argument (argc, argv, "-e") >= 0)
  {
    std::cout << "Match and visualize remaining" << std::endl;

		std::vector<int> inliers;
		//boost::shared_ptr< std::vector<int> > inliers_ptr(inliers);

		pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
		  model_sphere(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (initial_cloud, true));

		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_sphere);
		ransac.setDistanceThreshold (2);
		ransac.computeModel(1);
		ransac.getInliers(inliers);

		std::cout << "Found "
		          << inliers.size()
		          << " inliers with RANSAC "
		          << std::endl;

		pcl::PointIndices::Ptr inliers_ptr (new pcl::PointIndices());
		inliers_ptr->indices = inliers;

		pcl::ExtractIndices<pcl::PointXYZ> ei_filter (true);
		ei_filter.setInputCloud (initial_cloud);
		ei_filter.setIndices (inliers_ptr);
		ei_filter.setNegative (true);
		ei_filter.filter (*matched_cloud);
		//ei_filter.setNegative (true);
		//ei_filter.filterDirectly (initial_cloud);

    //pcl::copyPointCloud<pcl::PointXYZ>(*initial_cloud, inliers, *matched_cloud);

    viewer.runOnVisualizationThreadOnce(viewer_matched_cloud);
    //viewer.runOnVisualizationThreadOnce(viewer_initial_cloud);
  }
  else if (pcl::console::find_argument (argc, argv, "-b") >= 0)
  {
    std::cout << "Match and visualize both" << std::endl;

		std::vector<int> inliers;
		//boost::shared_ptr< std::vector<int> > inliers_ptr(inliers);

		pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
		  model_sphere(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (initial_cloud, true));

    double min_r = 0;
    double max_r = 0;
    model_sphere->setRadiusLimits(20, 120);
    model_sphere->getRadiusLimits(min_r, max_r);

    std::cout << "Min R = " << min_r
      << " Max R = " << max_r << std::endl;

    //pcl::LeastMedianSquares<pcl::PointXYZ> ransac (model_sphere);
    //pcl::MaximumLikelihoodSampleConsensus<pcl::PointXYZ> ransac (model_sphere);
    //pcl::MEstimatorSampleConsensus<pcl::PointXYZ> ransac (model_sphere);
    //pcl::RandomizedMEstimatorSampleConsensus<pcl::PointXYZ> ransac (model_sphere);
		//pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_sphere);
		//pcl::ProgressiveSampleConsensus<pcl::PointXYZ> ransac (model_sphere);
    pcl::RandomizedRandomSampleConsensus<pcl::PointXYZ> ransac (model_sphere);

		ransac.setDistanceThreshold (3);
    ransac.setMaxIterations(10000);
		ransac.computeModel();
		ransac.getInliers(inliers);

		std::cout << "Found "
		          << inliers.size()
		          << " inliers with RANSAC "
		          << std::endl;
/*
    std::vector<int> inliers4;
		inliers4.push_back(inliers[0]);
    inliers4.push_back(inliers[1]);
    inliers4.push_back(inliers[2]);
    inliers4.push_back(inliers[3]);

    Eigen::VectorXf model_coefficients;

    for (size_t i = 4; i < (inliers.size() - 4); i++)
    {
      bool coeffs = model_sphere->computeModelCoefficients(inliers4, model_coefficients);
      if (coeffs)
      {
        std::cout << "Valid coefficients: " << model_coefficients << std::endl;
        //pcl::PointXYZ o;
        o.x = model_coefficients[0];
        o.y = model_coefficients[1];
        o.z = model_coefficients[2];
        r = model_coefficients[3];
        //viewer.addSphere (o, r, "sphere", 0);
        if (r > 20 && r < 120) break;
      }
      else
      {
        //std::cout << "Invalid coefficients: " << model_coefficients<< std::endl;
      }
      inliers4.pop_back();
      inliers4.push_back(inliers[i]);
    }
*/

    //pcl::computeCentroid(*initial_cloud, inliers, o);
    //r = 30;

    Eigen::VectorXf model_coefficients;
    ransac.getModelCoefficients(model_coefficients);
    std::cout << "Valid coefficients: " << model_coefficients << std::endl;
    o.x = model_coefficients[0];
    o.y = model_coefficients[1];
    o.z = model_coefficients[2];
    r = model_coefficients[3];

		pcl::PointIndices::Ptr inliers_ptr (new pcl::PointIndices());
		inliers_ptr->indices = inliers;

		pcl::ExtractIndices<pcl::PointXYZ> ei_filter (true);
		ei_filter.setInputCloud (initial_cloud);
		ei_filter.setIndices (inliers_ptr);
		ei_filter.setNegative (false);
		ei_filter.filter (*matched_cloud);
		ei_filter.setNegative (true);
		ei_filter.filter (*initial_cloud);
		//ei_filter.setNegative (true);
		//ei_filter.filterDirectly (initial_cloud);

    //pcl::copyPointCloud<pcl::PointXYZ>(*initial_cloud, inliers, *matched_cloud);

    viewer.runOnVisualizationThreadOnce(viewer_matched_cloud);
    viewer.runOnVisualizationThreadOnce(viewer_initial_cloud);
  }
  else if (pcl::console::find_argument (argc, argv, "-a") >= 0)
  {
    //shapes = true;
    std::cout << "Shapes visualisation example" << std::endl;
  }
  else if (pcl::console::find_argument (argc, argv, "-v") >= 0)
  {
    //viewports = true;
    std::cout << "Viewports example" << std::endl;
  }
  else if (pcl::console::find_argument (argc, argv, "-i") >= 0)
  {
    //interaction_customization = true;
    std::cout << "Interaction Customization example" << std::endl;
  }
  else
  {
    print_help (argv[0]);
    return 0;
  }


  //pcl::copyPointCloud<pcl::PointXYZ>(*initial_cloud, inliers, *matched_cloud);
  //pcl::IndicesPtr inliers_ptr(new std::vector<int>(inliers));

//indices_rem = eifilter.getRemovedIndices ();
// The indices_rem array indexes all points of cloud_in that are not indexed by indices_in
  //ei_filter.setNegative (true);
  //ei_filter.filter (*indices_out);
// Alternatively: the indices_out array is identical to indices_rem
//eifilter.setNegative (false);
//eifilter.setUserFilterValue (1337.0);
//eifilter.filterDirectly (cloud_in);
// This will directly modify cloud_in instead of creating a copy of the cloud
// It will overwrite all fields of the filtered points by the user value: 1337

/*
  std::cout << "Extracted "
            << matched_cloud->width * matched_cloud->height
            << " data points with filter"
            << std::endl;

  std::cout << "Left "
            << initial_cloud->width * initial_cloud->height
            << " data points"
            << std::endl;
*/
  //pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

  //boost::function2<void, pcl::visualization::PCLVisualizer&, int> viewer_green_cloud_f = &viewer_green_cloud;

  //viewer.runOnVisualizationThreadOnce(viewer_initial_cloud);
  //viewer.runOnVisualizationThreadOnce(viewer_matched_cloud);

  while (!viewer.wasStopped ())
  {
  }

  return (0);
}

