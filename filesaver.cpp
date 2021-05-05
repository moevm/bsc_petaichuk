#include "filesaver.h"

FileSaver::FileSaver()
{
    next = nullptr;
}

FileSaver::~FileSaver()
{
    delete next;
}

void FileSaver::setNext(FileSaver *next)
{
    this->next = next;
}

void PLYFileSaver::handleFileSaving(std::string fileName, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudToLoad)
{
    if (fileName.find(".ply") != std::string::npos)
    {
        pcl::io::savePLYFile<pcl::PointXYZRGBA>(fileName, *cloudToLoad);
    }
    else if (next != nullptr)
    {
        next->handleFileSaving(fileName, cloudToLoad);
    }
}

void PCDFileSaver::handleFileSaving(std::string fileName, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudToLoad)
{
    if (fileName.find(".pcd") != std::string::npos)
    {
        pcl::io::savePCDFile<pcl::PointXYZRGBA>(fileName, *cloudToLoad);
    }
    else if (next != nullptr)
    {
        next->handleFileSaving(fileName, cloudToLoad);
    }
}
