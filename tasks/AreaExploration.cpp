/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AreaExploration.hpp"
#include "ugv_nav4dTypes.hpp"
#include <ugv_nav4d/AreaExplorer.hpp>
#include <ugv_nav4d/FrontierGenerator.hpp>
#include <vizkit3d_debug_drawings/DebugDrawing.hpp>
#include <envire_core/items/SpatioTemporal.hpp>

#include <orocos_cpp/PkgConfigRegistry.hpp>

#include <maps/operations/CoverageMapGeneration.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

using namespace ugv_nav4d;

using maps::grid::TraversabilityNodeBase;
using maps::grid::TraversabilityBaseMap3d;
using envire::core::SpatioTemporal;

AreaExploration::AreaExploration(std::string const& name)
    : AreaExplorationBase(name)
{
}

AreaExploration::AreaExploration(std::string const& name, RTT::ExecutionEngine* engine)
    : AreaExplorationBase(name, engine)
{
}

AreaExploration::~AreaExploration()
{
}

void AreaExploration::calculateGoals(::ugv_nav4d::OrientedBoxConfig const & area)
{
    if (!explorationMode) {
        generateFrontiers = true;
        this->area = area;
    }
}

void AreaExploration::resumeExploring()
{
    if (!explorationMode) {
        if (!areaValid) {
            std::cout << "Cannot restart ExplorationMode since there was no area set before!" << std::endl;
        } else {
            explorationMode = true;
            std::cout << "Restarting ExplorationMode with previous area..." << std::endl;
        }
    }
}

void AreaExploration::stopExploring()
{
    std::cout << "Stopping exploration mode manually ..." << std::endl;
    explorationMode = false;
    generateFrontiers = false;
}

void AreaExploration::clearPlannerMap()
{

}

bool AreaExploration::configureHook()
{
    std::vector<std::string> channels = V3DD::GET_DECLARED_CHANNELS();
    std::vector<std::string> channels_filtered;
    for(const std::string& channel : channels)
    {
        //check if it contains ugv
        if(channel.find("area_explorer") != std::string::npos)
        {
            channels_filtered.push_back(channel);
        }
    }
    V3DD::CONFIGURE_DEBUG_DRAWINGS_USE_PORT(this, channels_filtered);

    frontGen = std::make_shared<FrontierGenerator>(_travConfig.get(), _costConfig.get());
    explorer = std::make_shared<AreaExplorer>(frontGen);

    if(_coverageRadius.get() > 0)
    {
        coverage = std::make_shared<maps::operations::CoverageTracker>();
    }
    else coverage.reset();

    V3DD::FLUSH_DRAWINGS();
    
    const std::vector<std::string> names{"ugv_nav4d"};
    orocos_cpp::PkgConfigRegistryPtr pkgConfig = orocos_cpp::PkgConfigRegistry::initialize(names, false);
    orocos_cpp::TypeRegistry typeRegistry = orocos_cpp::TypeRegistry(pkgConfig);

    typeRegistry.getStateID("ugv_nav4d::PathPlanner", "GOAL_INVALID", planner_GOAL_INVALID);
    typeRegistry.getStateID("ugv_nav4d::PathPlanner", "NO_SOLUTION", planner_NO_SOLUTION);
    typeRegistry.getStateID("ugv_nav4d::PathPlanner", "EXCEPTION", planner_EXCEPTION);
    typeRegistry.getStateID("ugv_nav4d::PathPlanner", "START_INVALID", planner_START_INVALID);

    std::cout << "PathPlanner state GOAL_INVALID has id " << planner_GOAL_INVALID << std::endl;
    std::cout << "PathPlanner state NO_SOLUTION has id " << planner_NO_SOLUTION << std::endl;

    if (! AreaExplorationBase::configureHook())
        return false;
    return true;
}
bool AreaExploration::startHook()
{
    poseValid = false;
    mapValid = false;
    areaValid = false;
    
    explorationMode = false;
    generateFrontiers = false;
    
    if (! AreaExplorationBase::startHook())
        return false;
    return true;
}
void AreaExploration::updateHook()
{
    AreaExplorationBase::updateHook();

    Eigen::Affine3d mls2Planner(Eigen::Translation3d(_gridOffset.rvalue()));

    if(_map.readNewest(map, false) == RTT::NewData)
    {
        mapValid = true;
        map.data.translate(_gridOffset.rvalue());
        frontGen->updateMap(map.data, coverage ? &coverage->getCoverage() : nullptr);
        if(coverage)
            coverage->setFrame(frontGen->getTraversabilityMap());
    }

    if(_pose_samples.readNewest(curPose, false) == RTT::NewData)
    {
        // translate curPose to planner frame
        curPose.setTransform(mls2Planner * curPose.getTransform());

        if(!poseValid)
        {
            std::cout << "Setting initial patch for area explorer" << std::endl;
            explorer->setInitialPatch(curPose.getTransform(), _initialPatchRadius.get());
            previousPose = curPose;
        }
        poseValid = true;
        
        if(coverage && mapValid && (curPose.position - previousPose.position).norm() > 
        _coverageUpdateDistance.get())
        {
            std::cout << "Adding coverage at " << curPose.position.transpose() << '\n';
            // TODO AngleSegment and orientation are ignored at the moment
            coverage->addCoverage(_coverageRadius.get(), base::AngleSegment{}, curPose.getPose());
            _coverage.write(coverage->getCoverage());
            previousPose = curPose;
        }
    }

    if (_area.readNewest(area, false) == RTT::NewData) {
        std::cout << "New area detected. Starting exploration mode with new area ..." << std::endl;
        explorationMode = true;
        area.center = mls2Planner * area.center;
        area.orientation = base::Quaterniond((mls2Planner * area.orientation).linear());
        if (!areaValid)
            areaValid = true;
    }

    boost::int32_t newPlannerState;
    if (_planner_state.readNewest(newPlannerState, false) == RTT::NewData) {
        if (newPlannerState != planner_state) {
            planner_state = newPlannerState;
            std::cout << "Planner changed to state " << planner_state << std::endl;
            if (explorationMode) {
                if ((planner_state == planner_GOAL_INVALID || planner_state == planner_NO_SOLUTION) && state() == EXPLORING) { // GOAL_INVALID or NO_SOLUTION while exploring
                    if (currentGoals.size() > 1) { // there is an other goal than the latest.
                        std::cout << "Planner says that current best goal is invalid. Writing next best goal on output." << std::endl;
                        currentGoals.erase(currentGoals.begin(), currentGoals.begin() + 1); // erase latestBestGoal
                        setAndOutputBestGoal();
                    } else {
                        std::cout << "There are no goals that the planner accepted. Stopping AreaExploration" << std::endl;
                        generateFrontiers = false;
                        explorationMode = false;
                    }
                } else if (planner_state == planner_EXCEPTION || planner_state == planner_START_INVALID) {
                    std::cout << "Planner reported an error. Stopping AreaExploration..." << std::endl;
                    generateFrontiers = false;
                    explorationMode = false;
                }
            }
        }
    } 
    
    if(!poseValid)
    {
        state(ugv_nav4d::AreaExplorationBase::NO_POSE);
    }
    else if(!mapValid)
    {
        state(ugv_nav4d::AreaExplorationBase::NO_MAP);
    }
    else {
        if (!explorationMode && !generateFrontiers && state() != AREA_EXPLORED) 
        {
            state(GOT_MAP_AND_POSE);
        }

        if(explorationMode) {  
            //std::cout << "Distance to latest goal = " << (curPose.position - latestBestGoal.position).norm() << std::endl;         
            if (state() == EXPLORING && (curPose.position.head(2) - latestBestGoal.position.head(2)).norm() < _goalReachedThresholdDistance.get())
            {
                std::cout << "Goal was reached. Starting to compute frontiers and select new goal." << std::endl;
                // goal was reached
                generateFrontiers = true;
            } else if (state() == GOT_MAP_AND_POSE) {
                std::cout << "Starting to compute frontiers and select first goal." << std::endl;
                generateFrontiers = true;
            }
        }

        if(generateFrontiers) {
            state(PLANNING);
            
            std::vector<base::samples::RigidBodyState> outFrontiers;
            if(explorer->getFrontiers(curPose.position, area, outFrontiers))
            {
                state(GOALS_GENERATED);
                
                for(auto &f : outFrontiers)
                {
                    //convert to ground frame
                    f.position +=  f.orientation * Eigen::Vector3d(0,0, -_travConfig.get().distToGround);
                }
                
                currentGoals = outFrontiers;
                outputAllGoals();

                if (explorationMode) {
                    state(EXPLORING);
                    setAndOutputBestGoal();
                }
            }
            else
            {
                std::cout << "area explored." << std::endl;
                state(AREA_EXPLORED);
                if (explorationMode) {
                    explorationMode = false;
                }
            }
            
            std::cout << "Write updated travMap to output." << std::endl;
            SpatioTemporal<TraversabilityBaseMap3d> st(frontGen->getTraversabilityMap().copyCast<TraversabilityNodeBase*>());
            // planner has static transformation to world by [10, 10, 0]
            st.setFrameID("planner");
            _tr_map.write(st);
                        
            generateFrontiers = false;
            
            V3DD::FLUSH_DRAWINGS();
        }

    }
}
void AreaExploration::errorHook()
{
    AreaExplorationBase::errorHook();
}
void AreaExploration::stopHook()
{
    AreaExplorationBase::stopHook();
}
void AreaExploration::cleanupHook()
{
    AreaExplorationBase::cleanupHook();
}

void AreaExploration::setAndOutputBestGoal()
{
    Eigen::Affine3d mls2Planner(Eigen::Translation3d(_gridOffset.rvalue()));
    latestBestGoal = currentGoals.front();
    base::samples::RigidBodyState bestGoalInMls = latestBestGoal;
    bestGoalInMls.setTransform(mls2Planner.inverse() * bestGoalInMls.getTransform());
    _goal_out_best.write(bestGoalInMls);

    std::cout << "New best goal position is " << latestBestGoal.position << " (in planner frame)" << std::endl;
}

void AreaExploration::outputAllGoals()
{
    Eigen::Affine3d mls2Planner(Eigen::Translation3d(_gridOffset.rvalue()));
    std::vector<base::samples::RigidBodyState> outputGoalsInMls = currentGoals;
    for(auto &goal : outputGoalsInMls) {
        goal.setTransform(mls2Planner.inverse() * goal.getTransform());
    }
    _goals_out.write(outputGoalsInMls);
}