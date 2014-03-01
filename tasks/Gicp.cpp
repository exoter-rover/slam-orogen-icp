/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Gicp.hpp"

using namespace icp;

Gicp::Gicp(std::string const& name)
    : GicpBase(name)
{
}

Gicp::Gicp(std::string const& name, RTT::ExecutionEngine* engine)
    : GicpBase(name, engine)
{
}

Gicp::~Gicp()
{
}

void Gicp::point_cloud_modelTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &point_cloud_model_sample)
{
    throw std::runtime_error("Transformer callback for point_cloud_model not implemented");
}

void Gicp::point_cloud_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &point_cloud_samples_sample)
{
    throw std::runtime_error("Transformer callback for point_cloud_samples not implemented");
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Gicp.hpp for more detailed
// documentation about them.

bool Gicp::configureHook()
{
    if (! GicpBase::configureHook())
        return false;
    return true;
}
bool Gicp::startHook()
{
    if (! GicpBase::startHook())
        return false;
    return true;
}
void Gicp::updateHook()
{
    GicpBase::updateHook();
}
void Gicp::errorHook()
{
    GicpBase::errorHook();
}
void Gicp::stopHook()
{
    GicpBase::stopHook();
}
void Gicp::cleanupHook()
{
    GicpBase::cleanupHook();
}
