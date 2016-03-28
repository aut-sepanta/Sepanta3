#ifndef _OBJECT_PARAMS
#define _OBJECT_PARAMS

namespace object_params {
    // ******** Table top parameters*************
     
    unsigned int inlier_threshold = 300;
    double plane_detection_voxel_size = 0.01;
    double clustering_voxel_size = 0.003;
    double z_filter_min = 0.4;
    double z_filter_max = 1.25;
    double table_z_filter_min = 0.01;
    double table_z_filter_max = 0.5;
    double cluster_distance = 0.02;
    unsigned int min_cluster_size = 200;
    unsigned int normal_estimation_min_near_neighs(10);
    double sac_distance_threshold(0.05);
    unsigned int sac_max_iters(10000);
    double sac_normal_distance_weight(0.1);
    double sac_probability(0.99);
    //*******************  ISS parameter ******************
     
    double iss_gamma_21 (0.975);
    double iss_gamma_32 (0.975);
    double iss_min_neighbors (5);
    int iss_threads (4);

    double normal_estimation_k_search(10);
    double description_radius(0.04);
}
#endif
