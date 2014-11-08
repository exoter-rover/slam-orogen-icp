/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "GIcp.hpp"

using namespace icp;

//#define DEBUG_PRINTS 1

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

void GIcp::point_cloud_targetTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &point_cloud_target_sample)
{
    target_pc = point_cloud_target_sample;

    #ifdef DEBUG_PRINTS
    std::cout<<"** [POINT_CLOUD_TARGET]point_cloud_target at ("<<point_cloud_target_sample.time.toMicroseconds()<< ")**\n";
    #endif


    /** Convert to pcl point clouds **/
    this->toPCLPointCloud(target_pc, *target_cloud.get());
    target_cloud->height =  static_cast<int>(_point_cloud_height.value());
    target_cloud->width =  static_cast<int>(_point_cloud_width.value());

    /** Bilateral filter **/
    if (bfilter_config.filterOn)
    {
        bilateral_filter.setInputCloud(target_cloud);
        PCLPointCloud filtered_cloud;
        filtered_cloud.width = target_cloud->width;
        filtered_cloud.height = target_cloud->height;
        bilateral_filter.filter(filtered_cloud);
        target_cloud = boost::make_shared<PCLPointCloud>(filtered_cloud);
    }

}

void GIcp::point_cloud_sourceTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &point_cloud_source_sample)
{
    ::base::samples::RigidBodyState initial_deltapose, delta_pose;
    source_pc = point_cloud_source_sample;

    #ifdef DEBUG_PRINTS
    std::cout<<"** [POINT_CLOUD_SOURCE]point_cloud_source at ("<<point_cloud_source_sample.time.toMicroseconds()<< ")**\n";
    #endif

    if (_initial_alignment.connected())
    {
        if(_initial_alignment.readNewest(initial_deltapose) == RTT::NewData)
        {
            /** Transform the cloud **/
            Eigen::Affine3d init_transform(initial_deltapose.orientation);
            init_transform.translation() = initial_deltapose.position;
            this->transformPointCloud(source_pc, init_transform);
        }
    }

    /** Convert to pcl point clouds **/
    this->toPCLPointCloud(source_pc, *source_cloud.get());
    source_cloud->height = static_cast<int>(_point_cloud_height.value());
    source_cloud->width = static_cast<int>(_point_cloud_width.value());

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

    /** First iteration when there is not point cloud in the other port **/
    if ((!_point_cloud_target.connected()) && (target_cloud->size() == 0))
    {
        target_cloud = boost::make_shared<PCLPointCloud>(*source_cloud);
    }

    /** Initial alignment **/
//    if (!_initial_alignment.connected())
//    {
//        Eigen::Matrix4d svd_transform;
//        std::clock_t begin = std::clock();
//        svd.estimateRigidTransformation (*source_cloud, *target_cloud, svd_transform);
//        std::clock_t end = std::clock();
//        Eigen::Affine3d init_transform;
//        init_transform.matrix() = svd_transform;
//        this->transformPointCloud(*source_cloud, init_transform);
//
//        double elapsed_secs = static_cast<double>(end - begin) / CLOCKS_PER_SEC;
//        std::cout<<"SVD alignment in "<<elapsed_secs<<" [seconds]\n";
//    }
//
    /** Perform align **/
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);
    pcl::PointCloud<pcl::PointXYZ> cloud_source_registered;
    std::clock_t begin = std::clock();
    icp.align(cloud_source_registered);
    std::clock_t end = std::clock();
    double elapsed_secs = static_cast<double>(end - begin) / CLOCKS_PER_SEC;

    #ifdef DEBUG_PRINTS
    std::cout<<"GICP alignment in "<<elapsed_secs<<" [seconds]\n";
    #endif

    double fitness_score = icp.getFitnessScore();

    /** Get the transformation **/
    Eigen::Isometry3f transformation;
    if(icp.hasConverged() && (fitness_score <= gicp_config.max_fitness_score))
    {
        transformation = icp.getFinalTransformation();
    }
    else
    {
        transformation = base::NaN<float>() * Eigen::Matrix4f::Ones();
    }

    #ifdef DEBUG_PRINTS
    std::cout<<"** [POINT_CLOUD_SOURCE] Fitness_score: "<<fitness_score<<"\n";
    std::cout<<"** [POINT_CLOUD_SOURCE] Transformation alignment:\n"<<transformation.matrix()<<"\n";
    #endif

    /** Delta Pose Output port **/
    transformation.makeAffine();
    Eigen::Affine3d tf;
    tf.matrix() = transformation.cast<double>().matrix();
    delta_pose.invalidate();
    delta_pose.time = source_pc.time;
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

    /** Debug Output **/
    if (_output_debug.value())
    {
        /** Filtered point cloud **/
        ::base::samples::Pointcloud filtered_pc;
        filtered_pc.time = source_pc.time;
        this->fromPCLPointCloud(filtered_pc, *source_cloud.get());
        _point_cloud_samples_out.write(filtered_pc);
    }

    /** In case target port is not connected **/
    if (!_point_cloud_target.connected())
    {
        /** The source is now the target for the next iteration **/
        target_cloud = boost::make_shared<PCLPointCloud>(*source_cloud);

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


    /** Organize point cloud **/
    if (source_cloud != NULL && !source_cloud->isOrganized())
    {
        source_cloud->height = _point_cloud_height.value();
        source_cloud->width = _point_cloud_width.value();
    }
    if (target_cloud != NULL && !target_cloud->isOrganized())
    {
        target_cloud->height =  _point_cloud_height.value();
        target_cloud->width =  _point_cloud_width.value();
    }


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
    GIcpBase::stopHook();
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

void GIcp::transformPointCloud(pcl::PointCloud<pcl::PointXYZ> &pcl_pc, const Eigen::Affine3d& transformation)
{
    for(std::vector< pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >::iterator it = pcl_pc.begin(); it != pcl_pc.end(); it++)
    {
        Eigen::Vector3d point (it->x, it->y, it->z);
        point = transformation * point;
        *it = pcl::PointXYZ (point[0], point[1], point[2]);
    }
}


