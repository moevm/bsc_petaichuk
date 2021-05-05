#ifndef FILTERPOINTCLOUDSTRATEGY_H
#define FILTERPOINTCLOUDSTRATEGY_H

#include <cmath>

#include <QApplication>
#include <QInputDialog>
#include <QProgressBar>
#include <QProgressDialog>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class FilterPointCloudStrategy
{
public:
    virtual ~FilterPointCloudStrategy() {}

    virtual pcl::PointCloud<pcl::PointXYZRGBA> filterCloud(pcl::PointCloud<pcl::PointXYZRGBA> cloudToFilter, QWidget *parent) = 0;
    virtual void prepareForExecution() = 0;
};

class GuidedFilterPointCloudStrategy : public FilterPointCloudStrategy
{
public:
    virtual pcl::PointCloud<pcl::PointXYZRGBA> filterCloud(pcl::PointCloud<pcl::PointXYZRGBA> cloudToFilter, QWidget *) override;
    virtual void prepareForExecution() override {};
};

class YeFilterPointCloudStrategy : public FilterPointCloudStrategy
{
public:
    virtual pcl::PointCloud<pcl::PointXYZRGBA> filterCloud(pcl::PointCloud<pcl::PointXYZRGBA> cloudToFilter, QWidget *parent) override;
    virtual void prepareForExecution() override;

private:
    double alphaFunction(pcl::PointXYZRGBA pk_s, pcl::PointXYZRGBA pi);
    double betaFunction(pcl::PointXYZRGBA pk_s, pcl::PointXYZRGBA pk_i);

    double thetaFunction(double x);
    double etaFunction(double x);
    double distanceBetweenPoints(pcl::PointXYZRGBA a, pcl::PointXYZRGBA b);

    double radiusParameter;
    double repulsionParameter;
    int iterationNumber;
};

#endif // FILTERPOINTCLOUDSTRATEGY_H
