/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PathPlanner.hpp"
#include <ugv_nav4d/Planner.hpp>
#include <vizkit3d_debug_drawings/DebugDrawing.hpp>
#include <trajectory_follower/SubTrajectory.hpp>
#include <base-logging/Logging.hpp>

#include "pcl/point_cloud.h"
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h> // For removeNaNFromPointCloud

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <algorithm>

#include <fstream>

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

int32_t PathPlanner::triggerPathPlanning(const base::samples::RigidBodyState& start_position,
                                         const base::samples::RigidBodyState& goal_position)
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

bool PathPlanner::findTrajectoryOutOfObstacle()
{
    if (!gotMap){
        return false;
    }
    setIfNotSet(RECOVERING);
    std::shared_ptr<trajectory_follower::SubTrajectory> trajectory2D;
    Eigen::Affine3d ground2Body(Eigen::Affine3d::Identity());
    ground2Body.translation() = Eigen::Vector3d(0, 0, -_travConfig.get().distToGround);

    _start_pose_samples.read(start_pose);

    if(!base::samples::RigidBodyState::isValidValue(start_pose.position) ||
            !base::samples::RigidBodyState::isValidValue(start_pose.orientation)){

        LOG_INFO_S << "Invalid start_pose_samples. Recovery behavior is aborted!!";
        setIfNotSet(FAILED_TO_RECOVER);
        return false;
    }

    //TODO: Use the closest surface patch instead of the hardcoded distToGround param. The param makes sense to be used only for the initial patch generation.

    const Eigen::Affine3d startGround2Mls(start_pose.getTransform() * ground2Body);
    LOG_DEBUG_S << "PathPlanner::findTrajectoryOutOfObstacle(): startGround2Mls " << startGround2Mls.translation();

    trajectory2D = planner->findTrajectoryOutOfObstacle(startGround2Mls.translation(), start_pose.getYaw(), ground2Body);
    if (trajectory2D != nullptr){
        LOG_INFO_S << "FOUND WAY OUT!";
        _detailedTrajectory2D.write(std::vector<trajectory_follower::SubTrajectory>{*trajectory2D});
        setIfNotSet(RECOVERED);
        return true;
    }
    else {
        LOG_INFO_S << "NO WAY OUT, ROBOT IS STUCK!";
        setIfNotSet(FAILED_TO_RECOVER);
        return false;
    }
}

bool PathPlanner::configureHook()
{
#ifdef ENABLE_DEBUG_DRAWINGS
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
    V3DD::FLUSH_DRAWINGS();
#endif

    planner.reset(new Planner(_primConfig.get(), _travConfig.get(), _mobilityConfig.get(), _plannerConfig.get()));
    planner->setTravMapCallback([&] ()
    {
        //this callback will be called whenever the planner has generated a new travmap.
        _tr_map.write(planner->getTraversabilityMap().copyCast<maps::grid::TraversabilityNodeBase*>());
    });

    initalPatchAdded = false;
    executePlanning = false;
    gotMap = false;

    if (_load_mls_from_file.get() == true){
        const std::string mls_file_path = _mls_file_path.get();
        const std::string mls_file_type = _mls_file_type.get();
        if (mls_file_type == "ply"){        
            if (loadPlyAsMLS(mls_file_path)){
                gotMap = true;
            }
        }
        else if (mls_file_type == "bin"){  
            if(loadMLSMapFromBin(mls_file_path)){
                gotMap = true;
            }
        }
        else{
            LOG_ERROR_S << "Invalid MLS File Type: "+ mls_file_type;
        }
    }

    if (! PathPlannerBase::configureHook())
        return false;
    return true;
}

bool PathPlanner::loadPlyAsMLS(const std::string& path){
    std::ifstream fileIn(path);       
    if(path.find(".ply") != std::string::npos)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PLYReader plyReader;
        if(plyReader.read(path, *cloud) >= 0)
        {
            pcl::PointXYZ min, max; 
            pcl::getMinMax3D (*cloud, min, max); 
            
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
  
            const double mls_res = _travConfig.get().gridResolution;
            const double size_x = max.x - min.x;
            const double size_y = max.y - min.y;
            const maps::grid::Vector3d offset(min.x-1.5*mls_res, min.y-1.5*mls_res, 0);

            maps::grid::MLSConfig cfg;
            cfg.gapSize = 0.1; //get_parameter("mls_gap_size").as_double();
            maps::grid::MLSMapSloped mlsMap;
            const maps::grid::Vector2ui numCells(size_x / mls_res + 1, size_y / mls_res + 1);
            mlsMap = maps::grid::MLSMapSloped(numCells, maps::grid::Vector2d(mls_res, mls_res), cfg);
            mlsMap.translate(offset);
            mlsMap.mergePointCloud(*cloud, base::Transform3d::Identity());
            if (_write_mls_to_port.get() == true){
                _mls_map.write(mlsMap);
            }
            planner->updateMap(mlsMap);     
        }
        return true;
    }
    LOG_ERROR_S << "Unabled to load mls. Unknown format!";
    return false;
}

bool PathPlanner::loadMLSMapFromBin(const std::string& filename){
    if (filename.empty()) {
        LOG_WARN_S << "Failed to load MLS Map from empty file: " << filename;
        return false;
    }

    // Open the binary file in input mode
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        LOG_WARN_S << "Failed to open file: " << filename;
        return false;
    }

    try {
        // Load the file contents into the stream and deserialize
        boost::archive::binary_iarchive ia(file);
        maps::grid::MLSMapSloped mlsMap;
        ia >> mlsMap;  // Deserialize into mlsMap
        if (_write_mls_to_port.get() == true){
            _mls_map.write(mlsMap);
        }
        planner->updateMap(mlsMap);      
        LOG_DEBUG_S << "Loaded MLS Map from " << filename;
        return true;
    } catch (const std::exception &e) {
        LOG_ERROR_S << "Error loading MLS Map: " << e.what();
        return false;
    }
}

bool PathPlanner::startHook()
{
    if (! PathPlannerBase::startHook())
        return false;

    if(gotMap){
        setIfNotSet(GOT_MAP);
    }

    return true;
}

void PathPlanner::updateHook()
{
    try{

        maps::grid::MLSMapSloped map;
        auto map_status = _map.readNewest(map, false);

        // The NO_MAP state should only be accessible if no map has ever been received.
        // The planner should still be able to plan on 'old' maps (or least recently received map)
        if((map_status == RTT::NoData) && !gotMap)
        {
            setIfNotSet(NO_MAP);
            return;
        } else if(map_status == RTT::NewData)
        {
            gotMap = true;
            setIfNotSet(UPDATE_MAP);
            if (_write_mls_to_port.get() == true){
                _mls_map.write(map);
            }
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

            planner->enablePathStatistics(_plannerConfig.get().usePathStatistics);

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
#ifdef ENABLE_DEBUG_DRAWINGS
            V3DD::FLUSH_DRAWINGS();
#endif
        }
    }
    catch(const std::exception& ex){
        LOG_ERROR_S << "PathPlanner:UpdateHook(): " << ex.what();
        setIfNotSet(RUNTIME_ERROR);
    }

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
