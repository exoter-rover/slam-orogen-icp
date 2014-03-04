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
   pc_model = point_cloud_model_sample;
}

void GIcp::point_cloud_sourceTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &point_cloud_source_sample)
{
    ::base::samples::RigidBodyState initial_deltapose, delta_pose;
    pc_source = point_cloud_source_sample;

    if (_initial_alignment.readNewest(initial_deltapose) == RTT::NewData)
    {
        /** Transform the cloud **/

    }
    else
    {
        /** TO-DO: estimate initial transform using sample consensus **/

    }

    /** Convert to pcl point clouds **/

    /** Bilateral filter **/

    /** Perform align **/

    /** Get the transformation **/

    /** Debug Output **/
    if (_output_debug.value())
    {
        /** Debug info **/
        icp::ICPDebug icp_debug;
        icp_debug.time = delta_pose.time;
        //icp_debug.fitness_score = fitness_score;

        /** Filtered point cloud **/
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


::base::samples::RigidBodyState GIcp::performAlign (const ::base::samples::Pointcloud & target, const ::base::samples::Pointcloud & model, ::base::samples::RigidBodyState &initial_delta, icp::GICPConfiguration &gicp_config)
{
    ::base::samples::RigidBodyState delta_pose; /** delta pose **/
    PCLPointCloudPtr source_cloud (new PCLPointCloud); /** Input **/
    PCLPointCloudPtr target_cloud (new PCLPointCloud); /** Target **/

    /** Convert to pcl point cloud **/

    /** Set the source/input point cloud **/
    icp.setInputSource(source_cloud);

    /** Convert to pcl point cloud **/

    /** Set the target point cloud **/
    icp.setInputTarget(target_cloud);

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

    /** Store the pcl point cloud back into the original format **/

    /** Free shared pointer **/
    source_cloud.reset();
    target_cloud.reset();

    return delta_pose;
}

void GIcp::toPCLPointCloud(const std::vector< Eigen::Vector3d >& points, pcl::PointCloud< pcl::PointXYZ >& pcl_pc, double density)
{
    pcl_pc.clear();
    std::vector<bool> mask;
    unsigned sample_count = (unsigned)(density * points.size());

    if(density <= 0.0 || points.size() == 0)
    {
        return;
    }
    else if(sample_count >= points.size())
    {
        mask.resize(points.size(), true);
    }
    else
    {
        mask.resize(points.size(), false);
        unsigned samples_drawn = 0;

        while(samples_drawn < sample_count)
        {
            unsigned index = rand() % points.size();
            if(mask[index] == false)
            {
                mask[index] = true;
                samples_drawn++;
            }
        }
    }

    for(size_t i = 0; i < points.size(); ++i)
    {
        if(mask[i])
            pcl_pc.push_back(pcl::PointXYZ(points[i].x(), points[i].y(), points[i].z()));
    }
}

void GIcp::fromPCLPointCloud(std::vector< Eigen::Vector3d >& points, const pcl::PointCloud< pcl::PointXYZ >& pcl_pc, double density)
{
    std::vector<bool> mask;
    unsigned sample_count = (unsigned)(density * pcl_pc.size());

    if(density <= 0.0 || pcl_pc.size() == 0)
    {
        return;
    }
    else if(sample_count >= pcl_pc.size())
    {
        mask.resize(pcl_pc.size(), true);
    }
    else
    {
        mask.resize(pcl_pc.size(), false);
        unsigned samples_drawn = 0;

        while(samples_drawn < sample_count)
        {
            unsigned index = rand() % pcl_pc.size();
            if(mask[index] == false)
            {
                mask[index] = true;
                samples_drawn++;
            }
        }
    }

    for(size_t i = 0; i < pcl_pc.size(); ++i)
    {
        if(mask[i])
            points.push_back(::base::Point(pcl_pc.points[i].x, pcl_pc.points[i].y, pcl_pc.points[i].z));
    }
}

void GIcp::transformPointCloud(const std::vector< Eigen::Vector3d >& points, std::vector< Eigen::Vector3d >& transformed_pc, const Eigen::Affine3d& transformation)
{
    transformed_pc.clear();
    for(std::vector< Eigen::Vector3d >::const_iterator it = points.begin(); it != points.end(); it++)
    {
        transformed_pc.push_back(transformation * (*it));
    }
}

void GIcp::transformPointCloud(std::vector< Eigen::Vector3d >& points, const Eigen::Affine3d& transformation)
{
    for(std::vector< Eigen::Vector3d >::iterator it = points.begin(); it != points.end(); it++)
    {
        *it = (transformation * (*it));
    }
}

