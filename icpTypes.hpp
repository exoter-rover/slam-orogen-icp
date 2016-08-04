#ifndef icp_TYPES_HPP
#define icp_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <string>
#include <base/Time.hpp>
#include <base/Eigen.hpp>

namespace icp
{
    struct GICPConfiguration
    {
        unsigned maximum_iterations; //the maximum number of iterations the internal optimization should run for
        unsigned maximum_optimizer_iterations; //maximum number of iterations at the optimization step
        unsigned ransac_iterations; //is the number of iterations RANSAC should run for
        double ransac_oulier_threshold; //the inlier distance threshold for the internal RANSAC outlier rejection loop. The value is set by default to 0.05m.
        double transformation_epsilon; //the transformation epsilon in order for an optimization to be considered as having converged to the final solution.
        double rotation_epsilon; //the rotation epsilon in order for an optimization to be considered as having converged to the final solution.
        double euclidean_fitness_epsilon; //the maximum allowed distance error before the algorithm will be considered to have converged. error between two consecutive steps in the ICP loop
        double max_correspondence_distance; //distance threshold between two correspondent points in source <-> targe. If the distance is larger than this threshold, the points will be ignored in the alignment process
        unsigned correspondence_randomness; //the number of neighbors to use when computing covariances
        double max_fitness_score; //maximum allowed fitness score
    };

    struct PassThroughFilterConfiguration
    {
        bool filterOn;
        std::string axis_name;
        base::Vector2d limit;
    };


    struct BilateralFilterConfiguration
    {
        bool filterOn;
        float spatial_width; //size of the window bilateral filter
        float range_sigma; // the standard deviation of the Gaussian for the intensity difference
    };

    enum OutlierFilterType
    {
        NONE,
        STATISTICAL,
        RADIUS
    };


    struct OutlierRemovalFilterConfiguration
    {
        OutlierFilterType type;

        //STATISTICAL: the number of nearest neighbors to use for mean distance estimation (nr_k)
        //RADIUS: Get the radius of the sphere that will determine which points are neighbors (radiu).
        float parameter_one;

        //STATISTICAL: the standard deviation multiplier for the distance threshold calculation.(stddev_null)
        //RADIUS: number of neighbors that need to be present in order to be classified as an inlier(min_pts)
        float parameter_two;
    };


    struct ICPInfo
    {
        base::Time time;
        double fitness_score;
        base::Time compute_time;
    };

}

#endif

