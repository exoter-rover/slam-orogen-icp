/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "GIcp.hpp"

using namespace icp;

GIcp::GIcp(std::string const& name)
    : GIcpBase(name)
{
}

GIcp::GIcp(std::string const& name, RTT::ExecutionEngine* engine)
    : GIcpBase(name, engine)
{
}

GIcp::~GIcp()
{
}

void GIcp::point_cloud_modelTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &point_cloud_model_sample)
{
    /** Convert to pcl point cloud **/

    /** Set the target point cloud **/
    icp.setInputTarget(target_cloud);

}

void GIcp::point_cloud_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &point_cloud_samples_sample)
{

    ::base::samples::RigidBodyState delta_pose; /** delta pose **/

    /** Convert to pcl point cloud **/

    /** Set the source/input point cloud **/
    icp.setInputSource(source_cloud);

    /** Perform the alignment **/
    pcl::PointCloud<pcl::PointXYZ> cloud_source_registered;
    icp.align(cloud_source_registered);

    double fitness_score = icp.getFitnessScore();

    if(icp.hasConverged() && (fitness_score <= gicp_config.max_fitness_score))
    {
        Eigen::Isometry3f transformation(icp.getFinalTransformation());

        /** Check for nan values **/
        if(!base::isnotnan(transformation.matrix()))
        {
            std::cerr << "Measurement from ICP contains not numerical values." << std::endl;
        }

    }

    /** Debug Output **/
    if (_output_debug.value())
    {
        icp::ICPDebug icp_debug;
        icp_debug.time = delta_pose.time;
        icp_debug.fitness_score = fitness_score;
    }

}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See GIcp.hpp for more detailed
// documentation about them.

bool GIcp::configureHook()
{
    if (! GIcpBase::configureHook())
        return false;

    /** Read configuration **/
    gicp_config = _config_parameters.get();

    /** Configure GICP object **/
    icp.setMaxCorrespondenceDistance(gicp_config.max_correspondence_distance);
    icp.setMaximumIterations(gicp_config.maximum_iterations);
    icp.setTransformationEpsilon(gicp_config.transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(gicp_config.euclidean_fitness_epsilon);
    icp.setCorrespondenceRandomness(gicp_config.correspondence_randomness);
    icp.setMaximumOptimizerIterations(gicp_config.maximum_optimizer_iterations);
    icp.setRotationEpsilon(gicp_config.rotation_epsilon);

    return true;
}

bool GIcp::startHook()
{
    if (! GIcpBase::startHook())
        return false;

    /** Allocate memory for the point clouds **/
    source_cloud.reset (new PCLPointCloud);
    target_cloud.reset (new PCLPointCloud);

    return true;
}

void GIcp::updateHook()
{
    GIcpBase::updateHook();
}

void GIcp::errorHook()
{
    GIcpBase::errorHook();

}

void GIcp::stopHook()
{

    GIcpBase::stopHook();
}

void GIcp::cleanupHook()
{
    GIcpBase::cleanupHook();
}
