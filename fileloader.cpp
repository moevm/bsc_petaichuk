#include "fileloader.h"

FileLoader::FileLoader()
{
    next = nullptr;
}

FileLoader::~FileLoader()
{
    delete next;
}

void FileLoader::setNext(FileLoader *next)
{
    this->next = next;
}

void PLYFileLoader::handleFileLoading(std::string fileName, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudToLoad)
{
    if (fileName.find(".ply") != std::string::npos)
    {
        pcl::io::loadPLYFile<pcl::PointXYZRGBA>(fileName, *cloudToLoad);
    }
    else if (next != nullptr)
    {
        next->handleFileLoading(fileName, cloudToLoad);
    }
}


void PCDFileLoader::handleFileLoading(std::string fileName, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudToLoad)
{
    if (fileName.find(".pcd") != std::string::npos)
    {
        pcl::io::loadPCDFile<pcl::PointXYZRGBA>(fileName, *cloudToLoad);
    }
    else if (next != nullptr)
    {
        next->handleFileLoading(fileName, cloudToLoad);
    }
}
