#ifndef FILESAVER_H
#define FILESAVER_H

#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class FileSaver
{
public:
    FileSaver();
    virtual ~FileSaver();

    void setNext(FileSaver *next);
    virtual void handleFileSaving(std::string fileName, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudToLoad) = 0;

protected:
    FileSaver *next;
};

class PLYFileSaver : public FileSaver
{
public:
    void handleFileSaving(std::string fileName, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudToLoad) override;
};

class PCDFileSaver : public FileSaver
{
    void handleFileSaving(std::string fileName, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudToLoad) override;
};

#endif // FILESAVER_H
