#include "PoseWatchdog.hpp"
#include <base-logging/Logging.hpp>
#include <maps/grid/TraversabilityMap3d.hpp>
#include <ugv_nav4d/TravGenNode.hpp>
#include <vizkit3d_debug_drawings/DebugDrawing.h>
#include <vizkit3d_debug_drawings/DebugDrawingColors.h>

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
    
    resetState = false;
    
    mapGenerated = false;
    
    gotInitialMap = false;
    gotInitialPose = false;
    gotInitialTraj = false;
    gotNewMap = false;
    gotNewPose = false;
    gotNewTraj = false;
    
    haltCommand.heading = base::Angle::fromRad(0);
    haltCommand.translation = 0;
    haltCommand.rotation = 0;
    
    nanCommand.heading = base::Angle::fromRad(std::nan(""));
    nanCommand.translation = std::nan("");
    nanCommand.rotation = std::nan("");
    
    
    stateExecutes.resize(12); //NOTE increase if new states added
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

    obsMapGen.reset(new ObstacleMapGenerator3D(_travConfig.get()));
    
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
    
    if((mapGenerated && gotNewMap) || 
       (!mapGenerated && gotInitialMap && gotInitialPose))//<< this triggers the first map generation
    {
        updateMap();
    }
    
    //execute the internal state machine
    if(stateExecutes[state()] != nullptr)
        stateExecutes[state()]();
    else
        throw std::runtime_error("State execute function missing for State: " + state());
    
    PoseWatchdogBase::updateHook();
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
        _motion_command_override.write(nanCommand);
    }
    else
    {
        _motion_command_override.write(haltCommand);
    }
}

void PoseWatchdog::execResetted()
{
    //FIXME remain here until we have left the obstacle?!
    state(WATCHING);
    
    
    /*
           if (! PoseWatchdogBase::configureHook())
            std::cout << "waiting for valid pose" << std::endl;
            //until we get a valid pose we simply belive that everything is fine... HACK
            _motion_command_override.write(nanCommand);//tell safety that we are still alive
            
            if((_robot_pose.readNewest(pose, false) == RTT::NewData) |
//                (_tr_map.readNewest(map, false) == RTT::NewData) |
               (_currentTrajectory.readNewest(currentTrajectory, false)) == RTT::NewData)
            {
                if(checkPose())
                {
                    std::cout << "got valid pose, WATCHING" << std::endl;
                    state(WATCHING);
                }
            }
            
            */
    
}

bool PoseWatchdog::checkPose()
{
    //HACK for testing
    return true;
    
    
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
            maps::grid::TraversabilityNodeBase* node;// = map.getData().getClosestNode(pose.position);
            if(node)
            {
                //subtract distToGround to get from robot frame to map frame
                const double zDist = fabs(node->getHeight() - (pose.position.z() - _travConfig.value().distToGround));
                if(zDist > _travConfig.value().maxStepHeight)
                {
                    std::cout << "checkPose: node too far away. Distance: " << zDist << ", allowed distance: " << _travConfig.value().maxStepHeight  << std::endl;
                    return false;
                }
                
                
                std::deque<maps::grid::TraversabilityNodeBase*> nodes;
                nodes.push_back(node);
                std::unordered_set<maps::grid::TraversabilityNodeBase*> visited;
                const double maxDist = 0.1;
                const double gridRes = map.getData().getResolution().x();
                while(nodes.size() > 0)
                {
                    maps::grid::TraversabilityNodeBase* currNode = nodes.front();
                    nodes.pop_front();
                    visited.insert(currNode);
                    
                    if(currNode->getType() == maps::grid::TraversabilityNodeBase::TRAVERSABLE)
                    {
                        for(maps::grid::TraversabilityNodeBase* neighbor : currNode->getConnections())
                        {
                            const double dist = (neighbor->getVec3(gridRes).topRows(2) - node->getVec3(gridRes).topRows(2)).norm();
                            if(dist <= maxDist && visited.find(neighbor) == visited.end())
                                nodes.push_back(neighbor);
                        }
                    }
                    else
                    {
DRAW_CYLINDER("growCheckFail", currNode->getVec3(gridRes), base::Vector3d(0.1, 0.1, 0.8), vizkit3dDebugDrawings::Color::yellow);
                        std::cout << "checkPose: node not traversable." << std::endl;
                        return false;
                    }
                    
                }
                return true;
            }
            std::cout << "checkPose: no node at position" << std::endl;
            return false;
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
}

void PoseWatchdog::updateMap()
{
    
    //FIXME this needlesly copies the map... the interface of the ObstacleMapGenerator3D should be fixed
    std::shared_ptr<maps::grid::MLSMapKalman> pMap(new maps::grid::MLSMapKalman(map.getData()));
    obsMapGen->setMLSGrid(pMap);
    
    //FIXME use Affine3D for transformation?!
    const Eigen::Vector3d posInMap(pose.position.x(), pose.position.y(), pose.position.z() -_travConfig.value().distToGround);
    
    obsMapGen->expandAll(posInMap, 1.5); //FIXME radius should be parameter
    
    //output map for debugging purpose
    envire::core::SpatioTemporal<maps::grid::TraversabilityBaseMap3d> obsMap;
    obsMap.data = obsMapGen->getTraversabilityBaseMap(); //FIXME this copies the map 
    obsMap.frame_id = "ObstacleMap";
    _obstacle_map.write(obsMap);
    
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
