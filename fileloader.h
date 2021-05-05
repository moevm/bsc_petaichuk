#ifndef FILELOADER_H
#define FILELOADER_H

#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class FileLoader
{
public:
    FileLoader();
    virtual ~FileLoader();

    void setNext(FileLoader *next);
    virtual void handleFileLoading(std::string fileName, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudToLoad) = 0;

protected:
    FileLoader *next;
};

class PLYFileLoader : public FileLoader
{
public:
    void handleFileLoading(std::string fileName, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudToLoad) override;
};

class PCDFileLoader : public FileLoader
{
public:
    void handleFileLoading(std::string fileName, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudToLoad) override;
};

#endif // FILELOADER_H
