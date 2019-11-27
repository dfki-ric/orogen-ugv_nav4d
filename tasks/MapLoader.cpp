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
    sendMap = true;
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

            //transform point cloud to zero (instead we could also use MlsMap::translate later but that seems to be broken?)
            Eigen::Affine3f pclTf = Eigen::Affine3f::Identity();
            pclTf.translation() << -mi.x, -mi.y, -mi.z;
            pcl::transformPointCloud (*cloud, *cloud, pclTf);
            
            pcl::getMinMax3D (*cloud, mi, ma); 
      
            const double mls_res = _gridResolution.get();
            const double size_x = ma.x;
            const double size_y = ma.y;
            
            const maps::grid::Vector2ui numCells(size_x / mls_res + 1, size_y / mls_res + 1);
            std::cout << "NUM CELLS: " << numCells << std::endl;
            
            maps::grid::MLSConfig cfg;
            cfg.gapSize = _gapSize.get();
            map = maps::grid::MLSMapKalman(numCells, maps::grid::Vector2d(mls_res, mls_res), cfg);
            map.mergePointCloud(*cloud, base::Transform3d::Identity());
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
        sendMap = true;
    
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
    
    if(mapLoaded && sendMap)
    {
        sendMap = false;
        _map.write(map);
    }
    
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
