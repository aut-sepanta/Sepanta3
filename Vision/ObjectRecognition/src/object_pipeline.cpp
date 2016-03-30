
#include <sstream>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/shot_omp.h>

#include <ros/ros.h>

#include <object_pipeline.hpp>
#include <parameters.hpp>

/*
*/

ObjectPipeline::ObjectPipeline() {
    this->trained_objects = boost::shared_ptr<std::vector<Object>>(new std::vector<Object>());
}

ObjectPipeline::ObjectPipeline(boost::shared_ptr<std::vector<Object>> tobjs) : trained_objects(tobjs) {
}

void ObjectPipeline::computeCloudResolution (Object* object)
{
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices (2);
    std::vector<float> sqr_distances (2);
    pcl::search::KdTree<pcl::PointXYZRGB> tree;
    tree.setInputCloud(object->cloud);

    for (size_t i = 0; i < object->cloud->size (); ++i)
    {
        if (! pcl_isfinite ((*(object->cloud))[i].x))
        {
            continue;
        }
        //Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
        if (nres == 2)
        {
            res += sqrt (sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0)
    {
        res /= n_points;
    }

    object->cloud_resolution = res;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjectPipeline::passthroughPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pass.setFilterFieldName ("z");
    pass.setFilterLimits (object_params::z_filter_min, object_params::z_filter_max);
    pass.setInputCloud (cloud);
    pass.filter (*cloud_filtered);

    if (cloud_filtered->points.size() < object_params::min_cluster_size)
    {
        ROS_WARN_STREAM("filtered points are lower than minimum cluster size" << cloud_filtered->points.size());
        return nullptr;
    }

    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjectPipeline::findTableHull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered) {

    // Down sampling
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
    voxel_grid.setLeafSize (object_params::plane_detection_voxel_size, object_params::plane_detection_voxel_size, object_params::plane_detection_voxel_size);
    voxel_grid.setFilterFieldName ("z");
    voxel_grid.setFilterLimits (object_params::z_filter_min, object_params::z_filter_max);
    voxel_grid.setDownsampleAllData (false);
    voxel_grid.setInputCloud (cloud_filtered);
    voxel_grid.filter (*cloud_downsampled);
    if (cloud_downsampled->points.size() < object_params::min_cluster_size)
    {
        ROS_WARN_STREAM("down sampled points are lower than minimum cluster size" << cloud_downsampled->points.size());
        return nullptr;
    }

    //  Normal estimation
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr normals_tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> normal_estimation;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
    normal_estimation.setKSearch (object_params::normal_estimation_min_near_neighs);
    normal_estimation.setSearchMethod (normals_tree);
    normal_estimation.setInputCloud (cloud_downsampled);
    normal_estimation.compute (*cloud_normals);

    // Planar segmentation
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::PointNormal> sac_segmentation;
    sac_segmentation.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    sac_segmentation.setMethodType (pcl::SAC_RANSAC);
    sac_segmentation.setDistanceThreshold(object_params::sac_distance_threshold);
    sac_segmentation.setMaxIterations(object_params::sac_max_iters);
    sac_segmentation.setNormalDistanceWeight(object_params::sac_normal_distance_weight);
    sac_segmentation.setProbability (object_params::sac_probability);
    sac_segmentation.setOptimizeCoefficients(true);

    pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr table_coefficients(new pcl::ModelCoefficients);

    sac_segmentation.setInputCloud (cloud_downsampled);
    sac_segmentation.setInputNormals (cloud_normals);
    sac_segmentation.segment (*table_inliers, *table_coefficients);

    if (table_inliers->indices.size() < object_params::inlier_threshold)
    {
        ROS_WARN_STREAM("table inlier indices are lower than thershold" << cloud_downsampled->points.size());
        return nullptr;
    }

    if (table_coefficients->values.size() <= 3)
    {
        ROS_WARN_STREAM("table coefficients are not enough to solve for project inliers" << cloud_downsampled->points.size());
        return nullptr;
    }

    // Project inliers on the table
    pcl::ProjectInliers<pcl::PointXYZRGB> project_inliers;
    project_inliers.setModelType (pcl::SACMODEL_PLANE);
    project_inliers.setInputCloud (cloud_downsampled);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
    project_inliers.setIndices (table_inliers);
    project_inliers.setModelCoefficients (table_coefficients);
    project_inliers.filter (*table_projected);

    // Convex Hull
    pcl::ConvexHull<pcl::PointXYZRGB> convex_hull;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
    convex_hull.setInputCloud (table_projected);
    convex_hull.reconstruct (*table_hull);

    return table_hull;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjectPipeline::createObjectCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_hull) {
    pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;

    pcl::PointIndices::Ptr object_indices(new pcl::PointIndices);
    prism.setInputCloud(input_cloud);
    prism.setInputPlanarHull(table_hull);

    prism.setHeightLimits (object_params::table_z_filter_min, object_params::table_z_filter_max);
    prism.segment(*object_indices);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    extract_indices.setInputCloud(input_cloud);
    extract_indices.setIndices(object_indices);
    extract_indices.filter(*objects_cloud);

    ROS_INFO_STREAM("Number of object points found " << objects_cloud->points.size());

    if (objects_cloud->points.empty())
    {
        ROS_WARN_STREAM("No objects found on the table");
        return nullptr;
    }

    return objects_cloud;
}

boost::shared_ptr<std::vector<Object>> ObjectPipeline::createObjectClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects_cloud) {
    std::vector<pcl::PointIndices> object_clusters;

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> cluster_extraction;
    cluster_extraction.setClusterTolerance(object_params::cluster_distance);
    cluster_extraction.setMinClusterSize(object_params::min_cluster_size);
    cluster_extraction.setSearchMethod(search_tree);
    cluster_extraction.setInputCloud(objects_cloud);
    cluster_extraction.extract(object_clusters);

    boost::shared_ptr<std::vector<Object>> objects = boost::shared_ptr<std::vector<Object>>(new std::vector<Object>(object_clusters.size()));
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
    extract_indices.setInputCloud(objects_cloud);
    for (unsigned int i=0; i<object_clusters.size(); i++) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        extract_indices.setIndices(boost::make_shared<const pcl::PointIndices>(object_clusters[i]));
        extract_indices.filter(*(objects->at(i).cloud));
    }

    return objects;
}

void ObjectPipeline::keyPointExtraction(boost::shared_ptr<std::vector<Object>> objects) {
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());

    double iss_salient_radius;
    double iss_non_max_radius;
    double iss_normal_radius;
    double iss_border_radius;

    pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_detector;
    iss_detector.setSearchMethod (tree);
    iss_detector.setThreshold21 (object_params::iss_gamma_21);
    iss_detector.setThreshold32 (object_params::iss_gamma_32);
    iss_detector.setMinNeighbors (object_params::iss_min_neighbors);
    iss_detector.setNumberOfThreads (object_params::iss_threads);

    for (unsigned int i=0; i<objects->size(); i++) { 
        this->computeCloudResolution(&(objects->at(i)));
        iss_salient_radius = 6 * objects->at(i).cloud_resolution;
        iss_non_max_radius = 4 * objects->at(i).cloud_resolution;
        iss_normal_radius = 4 * objects->at(i).cloud_resolution;
        iss_border_radius = 1 * objects->at(i).cloud_resolution;

        iss_detector.setSalientRadius (iss_salient_radius);
        iss_detector.setNonMaxRadius (iss_non_max_radius);
        iss_detector.setNormalRadius (iss_normal_radius);
        iss_detector.setBorderRadius (iss_border_radius);

        iss_detector.setInputCloud (objects->at(i).cloud);
        iss_detector.compute (*(objects->at(i).keypoints));

        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr keypoints = boost::make_shared<const pcl::PointCloud<pcl::PointXYZRGB>>(*(objects->at(i).keypoints));
        ROS_INFO_STREAM("Keypoints found: " << keypoints->size());
    }
}

void ObjectPipeline::createObjectDescriptors(boost::shared_ptr<std::vector<Object>> objects) {
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> normal_estimation;
    normal_estimation.setKSearch(object_params::normal_estimation_k_search);
    for (unsigned int i=0; i<objects->size(); i++) {
        normal_estimation.setInputCloud(objects->at(i).cloud);
        normal_estimation.compute(*(objects->at(i).normals));
    }
   
    pcl::SHOTColorEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal, pcl::SHOT1344> shot_color_estimation;
    shot_color_estimation.setRadiusSearch (object_params::description_radius);
    
    for (unsigned int i=0; i<objects->size(); i++) {
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        shot_color_estimation.setSearchMethod(search_tree);
        shot_color_estimation.setSearchSurface(objects->at(i).cloud);
        shot_color_estimation.setInputNormals(objects->at(i).normals);
        shot_color_estimation.setInputCloud(objects->at(i).keypoints);
        shot_color_estimation.compute (*(objects->at(i).descriptors));
    }
}

void ObjectPipeline::objectMatching(boost::shared_ptr<std::vector<Object>> scene_objects) {
    pcl::KdTreeFLANN<pcl::SHOT1344> match_search;
    unsigned int max_correspondences=0;
    std::string current_label = "unknown";
    for (unsigned int i=0; i<scene_objects->size(); i++) {
        max_correspondences=0;
        scene_objects->at(i).label = current_label;
        for (unsigned int j=0; j<trained_objects->size(); j++) {
            pcl::CorrespondencesPtr correspondences (new pcl::Correspondences ());
            match_search.setInputCloud (trained_objects->at(j).descriptors);
            for (unsigned int k=0; k<scene_objects->at(i).descriptors->size(); k++) {
                std::vector<int> neigh_indices(1);
                std::vector<float> neigh_sqr_dists(1);
                if (!pcl_isfinite (scene_objects->at(i).descriptors->at(k).descriptor[0])) {
                    continue;
                }
                int found_neighs = match_search.nearestKSearch(scene_objects->at(i).descriptors->at(k), 1, neigh_indices, neigh_sqr_dists);
                if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) {
                    pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
                    correspondences->push_back (corr);
                }
            }
            if (max_correspondences < correspondences->size()) {
                max_correspondences = correspondences->size();
                scene_objects->at(i).label = trained_objects->at(j).label;
                scene_objects->at(i).correspondences = correspondences;
            }
        }
        ROS_INFO_STREAM("Object \"" << scene_objects->at(i).label << "\" found.(correspondences: " << max_correspondences << ")");
    }
}

void ObjectPipeline::correspondanceGrouping() {
}

void ObjectPipeline::absoluteOrientation() {
}

void ObjectPipeline::icpRefinement() {
}

void ObjectPipeline::hypothesisVerification() {
}

