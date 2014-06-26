#ifndef icp_TYPES_HPP
#define icp_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <string>
#include <base/time.h>
#include <base/eigen.h>

namespace icp
{
    struct GICPConfiguration
    {
        double max_correspondence_distance;
        unsigned maximum_iterations;
        double transformation_epsilon;
        double euclidean_fitness_epsilon;
        unsigned correspondence_randomness;
        unsigned maximum_optimizer_iterations;
        double rotation_epsilon;
        double max_fitness_score;
        double position_sigma;
        double orientation_sigma;
        double max_sensor_distance;
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
        STATISTICAL,
        RADIUS,
        NONE
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


    struct ICPDebug
    {
        base::Time time;
        double fitness_score;
    };

}

#endif

