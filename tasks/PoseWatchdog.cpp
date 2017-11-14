#include "PoseWatchdog.hpp"
#include <base-logging/Logging.hpp>
#include <maps/grid/TraversabilityMap3d.hpp>
#include <ugv_nav4d/TravGenNode.hpp>
#include <vizkit3d_debug_drawings/DebugDrawing.h>
#include <vizkit3d_debug_drawings/DebugDrawingColors.h>
#include <ugv_nav4d/EnvironmentXYZTheta.hpp>

using namespace ugv_nav4d;
using namespace trajectory_follower;

PoseWatchdog::PoseWatchdog(std::string const& name)
    : PoseWatchdogBase(name)
{
}

PoseWatchdog::PoseWatchdog(std::string const& name, RTT::ExecutionEngine* engine)
    : PoseWatchdogBase(name, engine)
{
}

PoseWatchdog::~PoseWatchdog()
{
}

bool PoseWatchdog::configureHook()
{
    if (! PoseWatchdogBase::configureHook())
        return false;
    
    const double robotLongerLength = std::max(_travConfig.value().robotSizeX, _travConfig.value().robotSizeY);
    
    mapGenerationRadius = robotLongerLength * 3.0;
    robotHalfLength = robotLongerLength / 2.0;
    
    resetState = false;
    
    mapGenerated = false;
    
    gotInitialMap = false;
    gotInitialPose = false;
    gotInitialTraj = false;
    gotNewMap = false;
    gotNewPose = false;
    gotNewTraj = false;
    
    gotPoseAfterReset = false;
    gotTrajAfterReset = false;
    
    haltCommand.heading = base::Angle::fromRad(0);
    haltCommand.translation = 0;
    haltCommand.rotation = 0;
    
    nanCommand.heading = base::Angle::fromRad(std::nan(""));
    nanCommand.translation = std::nan("");
    nanCommand.rotation = std::nan("");
    
    
    stateExecutes.resize(13); //NOTE increase if new states added
    stateExecutes[INIT] = nullptr;
    stateExecutes[PRE_OPERATIONAL] = nullptr;
    stateExecutes[FATAL_ERROR] = nullptr;
    stateExecutes[EXCEPTION] = nullptr;
    stateExecutes[STOPPED] = nullptr;
    stateExecutes[RUNNING] = std::bind(&PoseWatchdog::execRunning, this);
    stateExecutes[RUNTIME_ERROR] = nullptr;
    stateExecutes[ERROR] = nullptr;
    stateExecutes[RESETTED] = std::bind(&PoseWatchdog::execResetted, this);
    stateExecutes[TRAJECTORY_ABORTED] = std::bind(&PoseWatchdog::execAborted, this);
    stateExecutes[WAITING_FOR_DATA] = std::bind(&PoseWatchdog::execWaitForData, this);
    stateExecutes[WATCHING] = std::bind(&PoseWatchdog::execWatching, this);
    stateExecutes[IGNORING] = std::bind(&PoseWatchdog::execIgnoring, this);

    obsMapGen.reset(new ObstacleMapGenerator3D(_travConfig.get()));
    
    _currentTrajectory.clear();
    _map.clear();
    _currentTrajectory.clear();
    
    return true;
}
bool PoseWatchdog::startHook()
{
    if (! PoseWatchdogBase::startHook())
        return false;
    return true;
}
void PoseWatchdog::updateHook()
{
    CONFIGURE_DEBUG_DRAWINGS_USE_PORT_NO_THROW(this);
    
    //FIXME why is currentTrajectory a vector? Shouldn't it be just one SubTrajectory?
    
    gotNewTraj = _currentTrajectory.readNewest(currentTrajectory, false) == RTT::NewData;
    gotNewPose = _robot_pose.readNewest(pose, false) == RTT::NewData;
    gotNewMap  = _map.readNewest(map, false) == RTT::NewData;
    
    
    //got first pose, configure initial patch
    if(gotNewPose && !gotInitialPose)
    {
        Eigen::Affine3d startPos = pose.getTransform();
        startPos.translate(Eigen::Vector3d(0, 0, - _travConfig.value().distToGround));
        obsMapGen->setInitialPatch(startPos, _initialPatchRadius.value());
    } 
    
    gotInitialPose |= gotNewPose;
    gotInitialTraj |= gotNewTraj;
    gotInitialMap |= gotNewMap;
    
    if((mapGenerated && gotNewMap) || //<< this triggers map regeneration if we get a new mls
       (!mapGenerated && gotInitialMap && gotInitialPose) ||//<< this triggers the first map generation as soon as we got a pose and a map
       (mapGenerated && gotNewPose && (pose.position.topRows(2) - lastMapGenPos.topRows(2)).norm() >= mapGenerationRadius - robotHalfLength - 0.2))//<<this triggers a regen of the map if we get close to the boarder of the map
    {
        lastMapGenPos = pose.position;
        updateMap(pose.position);
    }
    
    //enables turn on/off during runtime
    if(!_abortTrajectory.get())
        state(IGNORING);
        
    //execute the internal state machine
    if(stateExecutes[state()] != nullptr)
        stateExecutes[state()]();
    else
        throw std::runtime_error("State execute function missing for State: " + state());
    
    PoseWatchdogBase::updateHook();
    
    FLUSH_DRAWINGS();
}

void PoseWatchdog::execIgnoring()
{
     _motion_command_override.write(nanCommand);
}


void PoseWatchdog::execRunning()
{
    //first call to updateHook() after startHook was executed. We do nothing in here.
    //this could be used for initialization later on

    state(WAITING_FOR_DATA);
}

void PoseWatchdog::execWaitForData()
{
    if(gotInitialMap && gotInitialPose && gotInitialTraj)
    {
        state(WATCHING);
        
        std::cout << "got initial data. state -> WATCHING" << std::endl;
    }
    else
    {
        //the robot shouldn't move while we wait for data
        _motion_command_override.write(haltCommand);
        
        std::cout << "waiting for data: map: " << gotInitialMap << ", pose: " << gotInitialPose << ", traj: " << gotInitialTraj << std::endl;
    }
}

void PoseWatchdog::execWatching()
{
    if(checkPose())
    {
        //everything is fine, writing nan tells the safety control that we have nothing to complain about
        _motion_command_override.write(nanCommand);
    }
    else
    {
        //pose is wrong, trigger safety control
        _motion_command_override.write(haltCommand);
        state(TRAJECTORY_ABORTED);
        
        std::cout << "Pose error. Stopping robot" << std::endl;
        DRAW_CYLINDER("watchdog_triggered", pose.position, base::Vector3d(0.03, 0.03, 0.4), vizkit3dDebugDrawings::Color::cyan);
    }
}

void PoseWatchdog::execAborted()
{
    //keep overriding until someone externally resetst he stateMachine
    if(resetState)
    {
        state(RESETTED);
        resetState = false;
        _robot_pose.clear();//clear all old poses that might have accumulated
        _currentTrajectory.clear(); //clear old trajectories
    }
    
    _motion_command_override.write(haltCommand);
}

void PoseWatchdog::execResetted()
{
    //do not allow the robot to move until we get new data
    _motion_command_override.write(haltCommand);
    
    //FIXME remain here until we have left the obstacle?!
    
    //wait here until we got at least one new pose
    gotPoseAfterReset |= gotNewPose;
    gotTrajAfterReset |= gotNewTraj;
    
    if(gotPoseAfterReset && gotTrajAfterReset)
        state(WATCHING);
    
}

bool PoseWatchdog::checkPose()
{
    //if we are not driving we cannot leave the map
    if(currentTrajectory.empty())
        return true;
    
    //FIXME this assumes that the kind is the same for all SubTrajectories in the vector.
    switch(currentTrajectory.front().kind)
    {
        //while rescuing from an unstable/unsafe position leaving the map is ok.
        case trajectory_follower::TRAJECTORY_KIND_RESCUE:
            std::cout << "ignoring rescue trajectory" << std::endl;
            return true;
        case trajectory_follower::TRAJECTORY_KIND_NORMAL:
        {
            //FIXME use real transform?
            const base::Vector3d pos(pose.position.x(), pose.position.y(),
                                     pose.position.z() - _travConfig.value().distToGround);
            
            return EnvironmentXYZTheta::obstacleCheck(pos, pose.getYaw(), *obsMapGen,
                                                      _travConfig.value(), _primConfig.value());
        }
            break;
        default:
            throw std::runtime_error("unknown trajectory kind: " + currentTrajectory.front().kind);
    }
    std::cout << "checkPose failed" << std::endl;
    return false;
}

void PoseWatchdog::reset()
{
    resetState = true;
    gotPoseAfterReset = false;
    gotTrajAfterReset = false;
}

void PoseWatchdog::updateMap(const Eigen::Vector3d& startPos)
{
    
    //FIXME this needlesly copies the map... the interface of the ObstacleMapGenerator3D should be fixed
    std::shared_ptr<maps::grid::MLSMapKalman> pMap(new maps::grid::MLSMapKalman(map.getData()));
    obsMapGen->setMLSGrid(pMap);
    
    //FIXME use Affine3D for transformation?!
    const Eigen::Vector3d posInMap(startPos.x(), startPos.y(),startPos.z() -_travConfig.value().distToGround);
    
    //HACK expand everything all the time. This is a hack for a demo, should not remain here once the demo is done
    oldStartPoses.push_back(posInMap);
    for(int i = oldStartPoses.size() - 1; i >= 0; --i)
    {
        obsMapGen->expandAll(oldStartPoses[i]);
    }
    
    
    //use this instead of the hack
//     obsMapGen->expandAll(posInMap, mapGenerationRadius); //FIXME radius should be parameter
    
    //output map for debugging purpose
//     envire::core::SpatioTemporal<maps::grid::TraversabilityBaseMap3d> obsMap;
//     obsMap.data = obsMapGen->getTraversabilityBaseMap(); //FIXME this copies the map 
//     obsMap.frame_id = "ObstacleMap";
//     _obstacle_map.write(obsMap);
    
    mapGenerated = true;
}


void PoseWatchdog::errorHook()
{
    PoseWatchdogBase::errorHook();
}
void PoseWatchdog::stopHook()
{
    PoseWatchdogBase::stopHook();
}
void PoseWatchdog::cleanupHook()
{
    PoseWatchdogBase::cleanupHook();
}
