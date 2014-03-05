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

void GIcp::point_cloud_targetTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &point_cloud_target_sample)
{
    target_pc = point_cloud_target_sample;

    /** Convert to pcl point clouds **/
    this->toPCLPointCloud(target_pc, *target_cloud.get(), gicp_config.point_cloud_density);

    /** Bilateral filter **/
    bilateral_filter.setInputCloud(target_cloud);
    PCLPointCloudPtr filtered_cloud(new PCLPointCloud);
    bilateral_filter.applyFilter(*filtered_cloud.get());
    target_cloud =  filtered_cloud;

}

void GIcp::point_cloud_sourceTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &point_cloud_source_sample)
{
    ::base::samples::RigidBodyState initial_deltapose, delta_pose;
    source_pc = point_cloud_source_sample;

    if (_initial_alignment.readNewest(initial_deltapose) == RTT::NewData)
    {
        /** Transform the cloud **/
        Eigen::Affine3d init_transform(initial_deltapose.orientation);
        init_transform.translation() = initial_deltapose.position;
        this->transformPointCloud(source_pc, init_transform);
    }
    else
    {
        /** TO-DO: estimate initial transform using sample consensus **/
        pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;

    }

    /** Convert to pcl point clouds **/
    this->toPCLPointCloud(source_pc, *source_cloud.get(), gicp_config.point_cloud_density);

    /** Bilateral filter **/
    bilateral_filter.setInputCloud(source_cloud);
    PCLPointCloudPtr filtered_cloud(new PCLPointCloud);
    bilateral_filter.applyFilter(*filtered_cloud.get());
    source_cloud =  filtered_cloud;

    /** Perform align **/
    icp.setInputCloud(source_cloud);
    icp.setInputTarget(target_cloud);
    pcl::PointCloud<pcl::PointXYZ> cloud_source_registered;
    icp.align(cloud_source_registered);

    double fitness_score = icp.getFitnessScore();


    /** Get the transformation **/
    Eigen::Isometry3f transformation;
    if(icp.hasConverged() && (fitness_score <= gicp_config.max_fitness_score))
    {
        transformation = icp.getFinalTransformation();
    }
    else
    {
        transformation.matrix() = base::NaN<double>() * Eigen::Matrix4f::Ones();
    }

    /** Debug Output **/
    if (_output_debug.value())
    {
        /** Debug info **/
        icp::ICPDebug icp_debug;
        icp_debug.time = delta_pose.time;
        icp_debug.fitness_score = fitness_score;

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

    /** Configure ICP **/
    icp.setMaxCorrespondenceDistance(gicp_config.max_correspondence_distance);
    icp.setMaximumIterations(gicp_config.maximum_iterations);
    icp.setTransformationEpsilon(gicp_config.transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(gicp_config.euclidean_fitness_epsilon);
    icp.setCorrespondenceRandomness(gicp_config.correspondence_randomness);
    icp.setMaximumOptimizerIterations(gicp_config.maximum_optimizer_iterations);
    icp.setRotationEpsilon(gicp_config.rotation_epsilon);

    /** Configure Bilateral filter **/

    return true;
}

bool GIcp::startHook()
{
    if (! GIcpBase::startHook())
        return false;

    source_cloud.reset(new PCLPointCloud);
    target_cloud.reset(new PCLPointCloud);

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

    source_cloud.reset();
    target_cloud.reset();
}

void GIcp::cleanupHook()
{
    GIcpBase::cleanupHook();
}


void GIcp::toPCLPointCloud(const ::base::samples::Pointcloud & pc, pcl::PointCloud< pcl::PointXYZ >& pcl_pc, double density)
{
    pcl_pc.clear();
    std::vector<bool> mask;
    unsigned sample_count = (unsigned)(density * pc.points.size());

    if(density <= 0.0 || pc.points.size() == 0)
    {
        return;
    }
    else if(sample_count >= pc.points.size())
    {
        mask.resize(pc.points.size(), true);
    }
    else
    {
        mask.resize(pc.points.size(), false);
        unsigned samples_drawn = 0;

        while(samples_drawn < sample_count)
        {
            unsigned index = rand() % pc.points.size();
            if(mask[index] == false)
            {
                mask[index] = true;
                samples_drawn++;
            }
        }
    }

    for(size_t i = 0; i < pc.points.size(); ++i)
    {
        if(mask[i])
        {
            /** Depth info **/
            pcl_pc.push_back(pcl::PointXYZ(pc.points[i].x(), pc.points[i].y(), pc.points[i].z()));

            /** Color info **/

        }
    }
}

void GIcp::fromPCLPointCloud(::base::samples::Pointcloud & pc, const pcl::PointCloud< pcl::PointXYZ >& pcl_pc, double density)
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
        {
            pc.points.push_back(::base::Point(pcl_pc.points[i].x, pcl_pc.points[i].y, pcl_pc.points[i].z));
        }
    }
}

void GIcp::transformPointCloud(const ::base::samples::Pointcloud & pc, ::base::samples::Pointcloud & transformed_pc, const Eigen::Affine3d& transformation)
{
    transformed_pc.points.clear();
    for(std::vector< ::base::Point >::const_iterator it = pc.points.begin(); it != pc.points.end(); it++)
    {
        transformed_pc.points.push_back(transformation * (*it));
    }
}

void GIcp::transformPointCloud(::base::samples::Pointcloud & pc, const Eigen::Affine3d& transformation)
{
    for(std::vector< ::base::Point >::iterator it = pc.points.begin(); it != pc.points.end(); it++)
    {
        *it = (transformation * (*it));
    }
}

