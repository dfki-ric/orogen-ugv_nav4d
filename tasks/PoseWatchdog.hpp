/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef UGV_NAV4D_POSEWATCHDOG_TASK_HPP
#define UGV_NAV4D_POSEWATCHDOG_TASK_HPP

#include "ugv_nav4d/PoseWatchdogBase.hpp"
#include <trajectory_follower/SubTrajectory.hpp>
#include <ugv_nav4d/ObstacleMapGenerator3D.hpp>
#include <functional>
#include <memory>

namespace ugv_nav4d{

    class PoseWatchdog : public PoseWatchdogBase
    {
    friend class PoseWatchdogBase;
    protected:
        
        std::vector<trajectory_follower::SubTrajectory> currentTrajectory;
        base::samples::RigidBodyState pose;
        envire::core::SpatioTemporal<maps::grid::MLSMapKalman> map;
        base::commands::Motion2D haltCommand; //this command is send when the robot should be halted
        base::commands::Motion2D nanCommand; //this command is send when everything is fine
        base::Vector3d lastMapGenPos;//last pose that was used to generate a map
        double mapGenerationRadius;
        double robotHalfLength;
        
        //true if new data has been received during this update loop
        bool gotNewMap;
        bool gotNewPose;
        bool gotNewTraj;
        
        //true if at least one sample has been received
        bool gotInitialMap;
        bool gotInitialPose;
        bool gotInitialTraj;
        
        bool mapGenerated;
        
        bool resetState;//signal that internal state should be resetted
        
        bool gotPoseAfterReset;
        bool gotTrajAfterReset;
        
        std::unique_ptr<ObstacleMapGenerator3D> obsMapGen; //is pointer because we need to lazy initialize it

        /**contains all execute functions for the internal state machine.
         * Use States enum as index!*/
        std::vector<std::function<void(void)>> stateExecutes;
        
        /* Resets the watchdog after it was triggered */
        virtual void reset();
        
        //internal state machine state execute functions
        void execRunning();
        void execWaitForData();
        void execWatching();
        void execAborted();
        void execResetted();
        void execIgnoring();
        
        /**Generate the obstacle map in a small radius around @p startPos.
         * @p startPos body frame*/
        void updateMap(const Eigen::Vector3d& startPos);
        
        

    public:
        /** TaskContext constructor for PoseWatchdog
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        PoseWatchdog(std::string const& name = "ugv_nav4d::PoseWatchdog");

        /** TaskContext constructor for PoseWatchdog
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices.
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
         * 
         */
        PoseWatchdog(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of PoseWatchdog
         */
	~PoseWatchdog();

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
        
        /** @return true if pose is ok */
        bool checkPose();
    };
}

#endif

