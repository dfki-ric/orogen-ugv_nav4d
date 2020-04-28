/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MapLoader.hpp"
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

using namespace ugv_nav4d;

MapLoader::MapLoader(std::string const& name, TaskCore::TaskState initial_state)
    : MapLoaderBase(name, initial_state)
{
}

MapLoader::~MapLoader()
{
}

void MapLoader::publishMap()
{
    if(mapLoaded)
    {
        _map.write(map);
    }else
    {
        LOG_ERROR_S << "No map loaded!" << std::endl;
    }
}

bool MapLoader::loadMls(const std::string& path)
{
    std::ifstream fileIn(path);       
    
    
    //FIXME this is a bad check. it could be a ply even without the file ending...
    if(path.find(".ply") != std::string::npos)
    {
        std::cout << "Loading PLY" << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PLYReader plyReader;
        if(plyReader.read(path, *cloud) >= 0)
        {
            pcl::PointXYZ mi, ma; 
            pcl::getMinMax3D (*cloud, mi, ma);
      
            const double mls_res = _resolution;
            const double size_x = ma.x - mi.x;
            const double size_y = ma.y - mi.y;
            
            const maps::grid::Vector2ui gridSize(size_x / mls_res + 1, size_y / mls_res + 1);
			const maps::grid::Vector2d cellSize(mls_res, mls_res);
            std::cout << "Grid-Size: " << gridSize[0] << " * " << gridSize[1] << std::endl;
            
            base::Transform3d pclTf = base::Transform3d::Identity();
			const double mid = mls_res / 2.0;
            pclTf.translation() << mid-mi.x, mid-mi.y, mid-mi.z;
			
            map.frame_id = _map_frame;
            map.time = base::Time::now();
            map.data = maps::grid::MLSMapKalman(gridSize, cellSize, _mls_config);
            map.data.mergePointCloud(*cloud, pclTf);
        }
        return true;
    }
    else
    {
        std::cout << "Not a ply file" << std::endl;
        return false;
    }
}



bool MapLoader::configureHook()
{
    if (! MapLoaderBase::configureHook())
        return false;
    
    mapLoaded = loadMls(_path.get());
    if(mapLoaded)
        publishMap();
    
    return true;
}
bool MapLoader::startHook()
{
    if (! MapLoaderBase::startHook())
        return false;
    return true;
}
void MapLoader::updateHook()
{
    MapLoaderBase::updateHook();
}
void MapLoader::errorHook()
{
    MapLoaderBase::errorHook();
}
void MapLoader::stopHook()
{
    MapLoaderBase::stopHook();
}
void MapLoader::cleanupHook()
{
    MapLoaderBase::cleanupHook();
}
