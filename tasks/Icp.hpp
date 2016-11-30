/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ICP_Icp_TASK_HPP
#define ICP_Icp_TASK_HPP

#include "icp/IcpBase.hpp"

/** Standard libraries **/
#include <cstdlib>
#include <ctime>

/** PCL library **/
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

/** Rock libraries **/
#include <base/samples/RigidBodyState.hpp>
#include <base/Point.hpp>

/** Eigen library **/
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace icp {

    typedef pcl::PointXYZ PCLPointT;
    typedef pcl::PointCloud<PCLPointT> PCLPointCloud;
    typedef typename PCLPointCloud::Ptr PCLPointCloudPtr;
    typedef pcl::PointNormal PCLPointNormalT;
    typedef pcl::PointCloud<PCLPointNormalT> PCLPointCloudWithNormals;

    class MyPointRepresentation : public pcl::PointRepresentation <PCLPointNormalT>
    {
        using pcl::PointRepresentation<PCLPointNormalT>::nr_dimensions_;
        public:

        MyPointRepresentation ()
        {
            // Define the number of dimensions
            nr_dimensions_ = 4;
        }

        // Override the copyToFloatArray method to define our feature vector
        virtual void copyToFloatArray (const PCLPointNormalT &p, float * out) const
        {
            // < x, y, z, curvature >
            out[0] = p.x;
            out[1] = p.y;
            out[2] = p.z;
            out[3] = p.curvature;
        }
    };


    /*! \class Icp 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * If this project uses data types that are defined in other oroGen projects,
    these projects should be imported there as well.
    import_types_from "base"
    Declare a new task context (i.e., a component)

    The corresponding C++ class can be edited in tasks/Task.hpp and
    tasks/Task.cpp, and will be put in the icp namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','icp::Icp')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Icp : public IcpBase
    {
    friend class IcpBase;

    protected:
        /**************************/
        /*** Property Variables ***/
        /**************************/

        /***************************/
        /** Input port variables **/
        /***************************/

        /*******************************/
        /** General Purpose variables **/
        /*******************************/
        base::samples::RigidBodyState rbs_world_sensor;
        Eigen::Affine3d tf_world_sensor; /** Tworld_sensor **/
        Eigen::Affine3d tf_sensor_world; /** Previous transformation Tsensor_world **/

        PCLPointCloudPtr source_cloud; /** Input **/
        PCLPointCloudPtr target_cloud; /** Target **/

        /***************************/
        /** Output port variables **/
        /***************************/
        PCLPointCloudPtr source_cloud_registered;
        ::base::samples::RigidBodyState pose; /** Cumulative pose **/

    public:
        /** TaskContext constructor for Icp
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Icp(std::string const& name = "icp::Icp");

        /** TaskContext constructor for Icp 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Icp(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Icp
         */
        ~Icp();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        void transformPointCloud(pcl::PointCloud<PCLPointT> &pcl_pc, const Eigen::Affine3d& transformation);

        void pairAlign (const PCLPointCloudPtr cloud_src, const PCLPointCloudPtr cloud_tgt, PCLPointCloudPtr output, Eigen::Matrix4f &final_transform, bool downsample = false);
    };
}

#endif

