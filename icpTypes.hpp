#ifndef icp_TYPES_HPP
#define icp_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <base/time.h>

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
        double point_cloud_density;
        double max_fitness_score;
        double position_sigma;
        double orientation_sigma;
        double max_sensor_distance;
    };

    struct ICPDebug
    {
        base::Time time;
        double fitness_score;
    };

}

#endif

