/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PathPlanner.hpp"
#include <ugv_nav4d/Planner.hpp>
#include <vizkit3d_debug_drawings/DebugDrawing.hpp>
#include <trajectory_follower/SubTrajectory.hpp>
#include <base-logging/Logging.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <algorithm>

using namespace ugv_nav4d;

using Eigen::Vector3d;
using Eigen::Affine3d;
using Eigen::Translation3d;
using maps::grid::TraversabilityNodeBase;
using maps::grid::TraversabilityBaseMap3d;
using trajectory_follower::SubTrajectory;

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
   
    _planning_start.write(start_position);
    _planning_goal.write(goal_position);
    
    executePlanning = true;
    
    return 1;
}

int32_t PathPlanner::generateTravMap() 
{
    if(!gotMap)
        return 0;

    base::samples::RigidBodyState pose;
    if(_start_pose_samples.readNewest(pose, false) == RTT::NoData) 
    {
        pose = start_pose;
    }

    if(!initalPatchAdded)
    {
        planner->setInitialPatch(pose.getTransform(), _initialPatchRadius.get());
        initalPatchAdded = true;
    }

    LOG_INFO_S << "PathPlanner: Manually generating travMap...";
    planner->genTravMap(pose);
    return 0;
}

boost::int32_t PathPlanner::findTrajectoryOutOfObstacle()
{
    base::Vector3d new_start_position;
    double new_start_theta;
    std::shared_ptr<trajectory_follower::SubTrajectory> trajectory2D;
    Eigen::Affine3d ground2Body(Eigen::Affine3d::Identity());

    _start_pose_samples.read(start_pose);

    if(!base::samples::RigidBodyState::isValidValue(start_pose.position) || 
            !base::samples::RigidBodyState::isValidValue(start_pose.orientation)){

        LOG_INFO_S << "Invalid start_pose_samples. Recovery behavior is aborted!!";
        return 0;
    }

    //TODO: Use the closest surface patch instead of the hardcoded distToGround param. The param makes sense to be used only for the initial patch generation. 
    ground2Body.translation() = Eigen::Vector3d(0, 0, -_travConfig.get().distToGround);
    trajectory2D = planner->getEnv()->findTrajectoryOutOfObstacle(start_pose.position, start_pose.getYaw(), ground2Body, new_start_position, new_start_theta);

    if (trajectory2D != nullptr){
        LOG_INFO_S << "FOUND WAY OUT!";
        _detailedTrajectory2D.write(std::vector<trajectory_follower::SubTrajectory>{*trajectory2D});
        return 1;
    }

    LOG_INFO_S << "NO WAY OUT, ROBOT IS STUCK!";
    return 0;
}

bool PathPlanner::isTraversable(::base::Vector3d const & patch_position){

    if(!gotMap){
        LOG_INFO_S << "PathPlanner::getPatchType: No map is generated !";        
        return false;
    }    

    double z{0.0};
    ::base::Vector3d temp = patch_position;
    if (planner->getEnv()->getMlsMap().getClosestSurfacePos(patch_position, z)){
        temp.z() = z;
    }

    ::maps::grid::Index idx;
    if(!planner->getTraversabilityMap().toGrid(temp, idx))
    {
        LOG_INFO_S << "PathPlanner::getPatchType: position outside of map !";
        return false;
    }

    auto &trList(planner->getTraversabilityMap().at(idx));

    //check if we got an existing node
    for(traversability_generator3d::TravGenNode *snode : trList)
    {
        if (snode->getType() == TraversabilityNodeBase::TRAVERSABLE){
            return true;
        }
    }
    return false;
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
        _tr_map.write(planner->getTraversabilityMap().copyCast<maps::grid::TraversabilityNodeBase*>());
        _ob_map.write(planner->getObstacleMap().copyCast<maps::grid::TraversabilityNodeBase*>());
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

    maps::grid::MLSMapSloped map;
    auto map_status = _map.readNewest(map, false);

    if(map_status == RTT::NoData)
    {
        setIfNotSet(NO_MAP);
        return;
    } else if(map_status == RTT::NewData)
    {
        gotMap = true;
        setIfNotSet(SET_UP_MAP_AND_SPLINES);
        planner->updateMap(map);
        setIfNotSet(GOT_MAP);
    }

    // start planning if there is a new relative goal in port
    if (_goal_pose_relative.readNewest(stop_pose, false) == RTT::NewData) {
        _start_pose_samples.read(start_pose);
        // transform stop pose to slam frame
        stop_pose.setTransform(start_pose.getTransform() * stop_pose.getTransform());

        executePlanning = true;
    }

    // start planning if there is a new absolute goal in port
    if (_goal_pose_absolute.readNewest(stop_pose, false) == RTT::NewData) {
        _start_pose_samples.read(start_pose);
       
        executePlanning = true;
    }

    if(executePlanning)
    {
        LOG_INFO_S << "PathPlanner: Executing planning...";
        _planning_start.write(start_pose);
        _planning_goal.write(stop_pose);

        if(!initalPatchAdded)
        {
            planner->setInitialPatch(start_pose.getTransform(), _initialPatchRadius.get());
            initalPatchAdded = true;
        }

        setIfNotSet(PLANNING);
        std::vector<SubTrajectory> trajectory2D, trajectory3D;

        Planner::PLANNING_RESULT res = planner->plan(_maxTime.value(), start_pose, stop_pose, trajectory2D, trajectory3D, _dumpOnError.get(), _dumpOnSuccess.get());

        // Create vectors of base trajectories for obtained trajectories which can be written
        // to the original output ports.
        std::vector<base::Trajectory>
            trajectory2DBase(trajectory2D.size()),
            trajectory3DBase(trajectory3D.size());
        std::transform(trajectory2D.cbegin(), trajectory2D.cend(), trajectory2DBase.begin(),
                [](const SubTrajectory& t) { return t.toBaseTrajectory(); });
        std::transform(trajectory3D.cbegin(), trajectory3D.cend(), trajectory3DBase.begin(),
                [](const SubTrajectory& t) { return t.toBaseTrajectory(); });

        switch(res)
        {
            case Planner::FOUND_SOLUTION:
                _trajectory2D.write(trajectory2DBase);
                _trajectory3D.write(trajectory3DBase);
                _detailedTrajectory2D.write(trajectory2D);
                _detailedTrajectory3D.write(trajectory3D);
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
