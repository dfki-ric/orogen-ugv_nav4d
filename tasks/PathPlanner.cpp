/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PathPlanner.hpp"
#include "ugv_nav4dTypes.hpp"
#include <ugv_nav4d/Planner.hpp>
#include <envire_core/items/SpatioTemporal.hpp>
#include <vizkit3d_debug_drawings/DebugDrawing.hpp>
#include <trajectory_follower/SubTrajectory.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

using namespace ugv_nav4d;

using Eigen::Vector3d;
using Eigen::Affine3d;
using Eigen::Translation3d;

PathPlanner::PathPlanner(std::string const& name)
    : PathPlannerBase(name), planner(nullptr)
{
}

PathPlanner::PathPlanner(std::string const& name, RTT::ExecutionEngine* engine)
    : PathPlannerBase(name, engine), planner(nullptr)
{
}

PathPlanner::~PathPlanner()
{
}

void PathPlanner::setIfNotSet(const PathPlannerBase::States& newState)
{
    if(state() != newState)
        state(newState);
}


int32_t PathPlanner::triggerPathPlanning(const base::samples::RigidBodyState& start_position, const base::samples::RigidBodyState& goal_position)
{
    if(!gotMap)
        return 0;

    start_pose = start_position;
    stop_pose = goal_position;

    translateStartStopPose();
    
    _planning_start.write(start_pose);
    _planning_goal.write(stop_pose);
    
    executePlanning = true;
    
    return 1;
}

int32_t PathPlanner::triggerPathPlanning2(const base::samples::RigidBodyState& goal_position)
{
    if(!gotMap)
        return 0;

    // read start pose from input port
    _start_pose_samples.read(start_pose);

    stop_pose = goal_position;

    translateStartStopPose();
    
    _planning_start.write(start_pose);
    _planning_goal.write(stop_pose);
    
    executePlanning = true;
    
    return 1;
}

int32_t PathPlanner::triggerPathPlanningNoArgs()
{
    if(!gotMap)
        return 0;

    // read start pose from input port
    _start_pose_samples.read(start_pose);
    _goal_pose_samples.read(stop_pose);

    translateStartStopPose();

    _planning_start.write(start_pose);
    _planning_goal.write(stop_pose);
    
    executePlanning = true;
    
    return 1;
}

bool PathPlanner::configureHook()
{
    std::vector<std::string> channels = V3DD::GET_DECLARED_CHANNELS();
    std::vector<std::string> channels_filtered;
    
    for(const std::string& channel : channels)
    {
        //check if it contains ugv
        if(channel.find("ugv") != std::string::npos)
        {
            channels_filtered.push_back(channel);
        }
    }
    
    V3DD::CONFIGURE_DEBUG_DRAWINGS_USE_PORT(this, channels_filtered);

    planner.reset(new Planner(_primConfig.get(), _travConfig.get(), _mobilityConfig.get(), _plannerConfig.get()));
    
    
    planner->setTravMapCallback([&] () 
    {
        //this callback will be called whenever the planner has generated a new travmap.
    });
    
    V3DD::FLUSH_DRAWINGS();
    
    if (! PathPlannerBase::configureHook())
        return false;
    return true;

}
bool PathPlanner::startHook()
{
    if (! PathPlannerBase::startHook())
        return false;
    
    initalPatchAdded = false;
    executePlanning = false;
    gotMap = false;
    return true;
}

void PathPlanner::updateHook()
{
    
    envire::core::SpatioTemporal<maps::grid::MLSMapKalman> map;
    auto map_status = _map.readNewest(map, false);

    if(map_status == RTT::NoData)
    {
        setIfNotSet(NO_MAP);
        return;
    } else if(map_status == RTT::NewData)
    {
        gotMap = true;
        map.data.moveBy(gridTranslation());
        planner->updateMap(map.getData());
        setIfNotSet(GOT_MAP);
    } 
    
    if(executePlanning)
    {
        if(!initalPatchAdded)
        {
            planner->setInitialPatch(start_pose.getTransform(), _initialPatchRadius.get());
            initalPatchAdded = true;
        }
        
        setIfNotSet(PLANNING);
        std::vector<base::Trajectory> trajectory2D;
        std::vector<base::Trajectory> trajectory3D;
        
        
        Planner::PLANNING_RESULT res = planner->plan(_maxTime.value(), start_pose, stop_pose, trajectory2D, trajectory3D, _dumpOnError.get());

        switch(res)
        {
            case Planner::FOUND_SOLUTION:
                _trajectory2D.write(trajectory2D);
                _trajectory3D.write(trajectory3D);
                setIfNotSet(FOUND_SOLUTION);
                break;
            
            case Planner::GOAL_INVALID:
                setIfNotSet(ugv_nav4d::PathPlannerBase::GOAL_INVALID);
                break;
            case Planner::START_INVALID:
                setIfNotSet(ugv_nav4d::PathPlannerBase::START_INVALID);
                break;
            case Planner::INTERNAL_ERROR:
                setIfNotSet(ugv_nav4d::PathPlannerBase::INTERNAL_ERROR);
                break;
            case Planner::NO_SOLUTION:
                setIfNotSet(ugv_nav4d::PathPlannerBase::NO_SOLUTION);
                break;
            case Planner::NO_MAP:
                setIfNotSet(ugv_nav4d::PathPlannerBase::NO_MAP);
                break;
        }
        
        executePlanning = false;
    }
    
    V3DD::FLUSH_DRAWINGS();
    PathPlannerBase::updateHook();
}
void PathPlanner::errorHook()
{
    PathPlannerBase::errorHook();
}
void PathPlanner::stopHook()
{
    PathPlannerBase::stopHook();
}
void PathPlanner::cleanupHook()
{
    planner.reset();
    PathPlannerBase::cleanupHook();
}

Eigen::Vector2i PathPlanner::gridTranslation() const
{
    const auto &transform = _gridOffset.rvalue();
    return Eigen::Vector2i(transform.translation[0], transform.translation[1]);
}

void PathPlanner::translateStartStopPose()
{
    const double gridResolution = _travConfig.rvalue().gridResolution;
    Vector3d translationVector;
    translationVector << gridTranslation().cast<double>() * gridResolution, .0;
    const Affine3d gridTransform{Translation3d(translationVector)};
    start_pose.setTransform(gridTransform * start_pose);
    stop_pose.setTransform(gridTransform * stop_pose);
}
