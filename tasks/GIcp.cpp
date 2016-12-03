/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "GIcp.hpp"
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <cmath>

using namespace icp;

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

#define DEBUG_PRINTS 1

GIcp::GIcp(std::string const& name)
    : GIcpBase(name)
{
    source_cloud.reset(new PCLPointCloud);
    target_cloud.reset(new PCLPointCloud);
}

GIcp::GIcp(std::string const& name, RTT::ExecutionEngine* engine)
    : GIcpBase(name, engine)
{
}

GIcp::~GIcp()
{
    source_cloud.reset();
    target_cloud.reset();
}

void GIcp::point_cloud_sourceTransformerCallback(const base::Time &ts, const ::envire::core::SpatioTemporal<pcl::PCLPointCloud2> &point_cloud_source_sample)
{
    #ifdef DEBUG_PRINTS
    std::cout<<"** [POINT_CLOUD_SOURCE]point_cloud_source at ("<<point_cloud_source_sample.time.toMicroseconds()<< ")**\n";
    #endif

    /** Convert to pcl point clouds **/
    pcl::fromPCLPointCloud2(point_cloud_source_sample.data, *source_cloud.get());

    Eigen::Affine3d tf_sensor_k_sensor_k_1; 
    ::base::samples::RigidBodyState initial_transform; // Tworld_sensor

    if (_initial_alignment.connected())
    {
        if(_initial_alignment.readNewest(initial_transform) == RTT::NewData)
        {
            /** Transform the cloud to world_frame **/
            //this->transformPointCloud(*source_cloud.get(), initial_transform.getTransform());
        }

        if (!base::isnotnan(tf_world_sensor_k_1.matrix()))
        {
            tf_world_sensor_k_1 = initial_transform.getTransform();
        }

        /** Delta transform sensor_k_sensor_k_1 **/
        tf_sensor_k_sensor_k_1 = initial_transform.getTransform().inverse() * tf_world_sensor_k_1;
    }

    /** Bilateral filter **/
    if (bfilter_config.filterOn)
    {
        bilateral_filter.setInputCloud(source_cloud);
        PCLPointCloud filtered_cloud;
        filtered_cloud.width = source_cloud->width;
        filtered_cloud.height = source_cloud->height;
        bilateral_filter.filter(filtered_cloud);
        source_cloud = boost::make_shared<PCLPointCloud>(filtered_cloud);
    }

    /** Pass filter **/
    if (passfilter_config.filterOn)
    {
        pass.setInputCloud(source_cloud);
        PCLPointCloud filtered_cloud;
        filtered_cloud.width = source_cloud->width;
        filtered_cloud.height = source_cloud->height;
        pass.filter(filtered_cloud);
        source_cloud = boost::make_shared<PCLPointCloud>(filtered_cloud);
    }

    /** Outlier filter in case of not NULL **/
    if (outlierfilter_config.type == STATISTICAL)
    {

        #ifdef DEBUG_PRINTS
        std::cout<<"STATISTICAL FILTER\n";
        #endif
        PCLPointCloud filtered_cloud;
        filtered_cloud.width = source_cloud->width;
        filtered_cloud.height = source_cloud->height;
        sor.setInputCloud(source_cloud);
        sor.filter (filtered_cloud);
        source_cloud = boost::make_shared<PCLPointCloud>(filtered_cloud);
    }
    else if (outlierfilter_config.type == RADIUS)
    {
        #ifdef DEBUG_PRINTS
        std::cout<<"RADIUS FILTER\n";
        #endif
        PCLPointCloud filtered_cloud;
        filtered_cloud.width = source_cloud->width;
        filtered_cloud.height = source_cloud->height;
        ror.setInputCloud(source_cloud);
        ror.filter (filtered_cloud);
        source_cloud = boost::make_shared<PCLPointCloud>(filtered_cloud);
    }

    /** Check before ICP alignment **/
    if (target_cloud->empty())
    {
        target_cloud = boost::make_shared<PCLPointCloud>(*source_cloud);
    }
    else
    {
        //base::Quaterniond qt_sensor_sensor_k_1 (tf_sensor_k_sensor_k_1.rotation());
        //Eigen::Vector3d euler = base::getEuler(qt_sensor_sensor_k_1);
        //#ifdef DEBUG_PRINTS
        //std::cout<< "Roll: "<<euler[2]*R2D<<" Pitch: "<<euler[1]*R2D<<" Yaw: "<<euler[0]*R2D<<"\n";
        //#endif

        //if (std::abs(euler[1])*R2D > 0.3 && std::abs(euler[1])*R2D < 1.5)
        //{
            /** Perform align **/
            icp.setInputSource(source_cloud);
            icp.setInputTarget(target_cloud);
            PCLPointCloud source_cloud_registered;
            std::clock_t begin = std::clock();
            icp.align(source_cloud_registered);
            std::clock_t end = std::clock();
            double elapsed_secs = static_cast<double>(end - begin) / CLOCKS_PER_SEC;

            #ifdef DEBUG_PRINTS
            std::cout<<"GICP alignment in "<<elapsed_secs<<" [seconds]\n";
            #endif

            double fitness_score = icp.getFitnessScore();

            /** Get the transformation **/
            Eigen::Affine3d transformation(Eigen::Affine3d::Identity());
            if(icp.hasConverged())
            {
                transformation.matrix() = icp.getFinalTransformation().cast<double>();
            }

            #ifdef DEBUG_PRINTS
            if(icp.hasConverged())
            {
                std::cout<<"** [POINT_CLOUD_SOURCE] CONVERGED ";
            }
            else
            {
                std::cout<<"** [POINT_CLOUD_SOURCE] NO CONVERGED ";
            }
            std::cout<<"** Fitness_score: "<<fitness_score<<"\n";
            std::cout<<"** [POINT_CLOUD_SOURCE] Transformation alignment:\n"<<transformation.matrix()<<"\n";
            #endif

            if (fitness_score <= gicp_config.max_fitness_score)
            {
                /** Delta Pose Output port **/
                ::base::samples::RigidBodyState delta_pose;
                transformation.makeAffine();
                Eigen::Affine3d tf;
                tf.matrix() = transformation.cast<double>().matrix();
                delta_pose.invalidate();
                delta_pose.time = point_cloud_source_sample.time;
                delta_pose.orientation = Eigen::Quaternion <double>(tf.rotation());
                delta_pose.position = tf.translation();
                _delta_pose_samples_out.write(delta_pose);

                /** Cumulative Pose **/
                Eigen::Affine3d poseTrans = pose.getTransform();
                poseTrans = poseTrans * delta_pose;
                pose.setTransform(poseTrans);
                _pose_samples_out.write(pose);

                /** ICP info **/
                icp::ICPInfo icp_info;
                icp_info.time = delta_pose.time;
                icp_info.fitness_score = fitness_score;
                icp_info.compute_time = ::base::Time::fromSeconds(elapsed_secs);
                _icp_info.write(icp_info);

                /** Accumulative point cloud in target **/
                *target_cloud += source_cloud_registered;

                PCLPointCloudPtr downsample_point_cloud (new PCLPointCloud);
                this->downsample (target_cloud, this->downsample_size, downsample_point_cloud);

                *target_cloud = *downsample_point_cloud;

                /** Update the last tf_world_sensor_k_1 transformation **/
                tf_world_sensor_k_1 = initial_transform.getTransform();

                /** Debug Output **/
                if (_output_debug.value())
                {
                    pcl::PCLPointCloud2 point_cloud_out;
                    //pcl::toPCLPointCloud2(source_cloud_registered, point_cloud_out);
                    pcl::toPCLPointCloud2(*target_cloud, point_cloud_out);
                    _point_cloud_samples_out.write(point_cloud_out);
                }
            }
        //}
        //else if (std::abs(euler[1])*R2D > 1.5)
        //{
        //    /** Update the last tf_world_sensor_k_1 transformation **/
        //    tf_world_sensor_k_1 = initial_transform.getTransform();

        //}
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
    gicp_config = _gicp_config.get();
    passfilter_config = _passfilter_config.get();
    bfilter_config = _bfilter_config.get();
    outlierfilter_config = _outlierfilter_config.get();

    /** Configure ICP **/
    icp.setMaximumIterations(gicp_config.maximum_iterations);
    icp.setMaximumOptimizerIterations(gicp_config.maximum_optimizer_iterations);
    icp.setRANSACIterations(gicp_config.ransac_iterations);
    icp.setMaxCorrespondenceDistance(gicp_config.max_correspondence_distance);
    icp.setTransformationEpsilon(gicp_config.transformation_epsilon);
    icp.setRotationEpsilon(gicp_config.rotation_epsilon);
    icp.setEuclideanFitnessEpsilon(gicp_config.euclidean_fitness_epsilon);
    icp.setCorrespondenceRandomness(gicp_config.correspondence_randomness);
    icp.setRANSACOutlierRejectionThreshold(gicp_config.ransac_oulier_threshold);

    /** Configure Bilateral filter **/
    pass.setFilterFieldName(passfilter_config.axis_name);
    pass.setFilterLimits(passfilter_config.limit[0], passfilter_config.limit[1]);
    pass.setNegative(true);

    /** Configure Bilateral filter **/
    bilateral_filter.setSigmaS(bfilter_config.spatial_width);
    bilateral_filter.setSigmaR(bfilter_config.range_sigma);

    /** Configure Outlier filter **/
    if (outlierfilter_config.type == icp::STATISTICAL)
    {
        sor.setMeanK(outlierfilter_config.parameter_one);
        sor.setStddevMulThresh(outlierfilter_config.parameter_two);
    }
    else if (outlierfilter_config.type == icp::RADIUS)
    {
        ror.setRadiusSearch(outlierfilter_config.parameter_one);
        ror.setMinNeighborsInRadius(outlierfilter_config.parameter_two);
    }

    this->downsample_size = _downsample_size.value();

    /** Initial cumulative pose **/
    pose.invalidate();
    pose.sourceFrame = _icp_odometry_source_frame.value();
    pose.targetFrame = _icp_odometry_target_frame.value();
    pose.position.setZero();
    pose.velocity.setZero();

    /** Assume well known starting position **/
    pose.cov_position = Eigen::Matrix3d::Zero();
    pose.cov_velocity = Eigen::Matrix3d::Zero();

    /** Orientation with respect to the relative navigation frame**/
    pose.orientation = Eigen::Quaterniond::Identity();
    pose.angular_velocity.setZero();

    /** Assume very well known initial attitude **/
    pose.cov_orientation = Eigen::Matrix3d::Zero();
    pose.cov_angular_velocity = Eigen::Matrix3d::Zero();

    tf_world_sensor_k_1.matrix() = base::NaN<double>() * Eigen::Matrix<double, 4, 4>::Ones();

    #ifdef DEBUG_PRINTS
    std::cout<<"Max iterations: "<<icp.getMaximumIterations()<<"\n";
    std::cout<<"Max Optimizer Iterations: "<<icp.getMaximumOptimizerIterations()<<"\n";
    std::cout<<"RANSAC iterations: "<<icp.getRANSACIterations()<<"\n";
    std::cout<<"RANSAC Oulier Threshold: "<<icp.getRANSACOutlierRejectionThreshold()<<"\n";
    std::cout<<"Transformation epsilon: "<<icp.getTransformationEpsilon()<<"\n";
    std::cout<<"Rotation Epsilon: "<<icp.getRotationEpsilon()<<"\n";
    std::cout<<"Euclidean Fitness epsilon: "<<icp.getEuclideanFitnessEpsilon()<<"\n";
    std::cout<<"Max correspondence distance: "<<icp.getMaxCorrespondenceDistance()<<"\n";
    std::cout<<"Correspondence Randomness: "<<icp.getCorrespondenceRandomness()<<"\n";
    #endif

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
    #ifdef DEBUG_PRINTS
    std::cout<<"Writing complete point cloud with "<< target_cloud->size()<<" points\n";
    #endif

   // GIcpBase::stopHook();
   // pcl::PCLPointCloud2 point_cloud_out;
   // pcl::toPCLPointCloud2(*complete_point_cloud.get(), point_cloud_out);
   // _point_cloud_samples_out.write(point_cloud_out);

    #ifdef DEBUG_PRINTS
    std::cout<<"DONE!\n";
    #endif
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
            if (base::isnotnan<base::Point>(pc.points[i]))
            {
                /** Depth info **/
                pcl_pc.push_back(pcl::PointXYZ(pc.points[i].x(), pc.points[i].y(), pc.points[i].z()));
            }
        }
    }

    /** All data points are finite (no NaN or Infinite) **/
    pcl_pc.is_dense = true;
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

void GIcp::transformPointCloud(pcl::PointCloud<pcl::PointXYZ> &pcl_pc, const Eigen::Affine3d& transformation)
{
    for(std::vector< pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >::iterator it = pcl_pc.begin(); it != pcl_pc.end(); it++)
    {
        Eigen::Vector3d point (it->x, it->y, it->z);
        point = transformation * point;
        *it = pcl::PointXYZ (point[0], point[1], point[2]);
    }
}

void GIcp::downsample (PCLPointCloud::Ptr &points, float leaf_size, PCLPointCloud::Ptr &downsampled_out)
{

  pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
  vox_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
  vox_grid.setInputCloud (points);
  vox_grid.filter (*downsampled_out);

  return;
}

