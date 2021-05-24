#ifndef FILTERPOINTCLOUDSTRATEGY_H
#define FILTERPOINTCLOUDSTRATEGY_H

#include <cmath>
#include <memory>
#include <vector>

#include <QApplication>
#include <QDialogButtonBox>
#include <QDoubleValidator>
#include <QFormLayout>
#include <QInputDialog>
#include <QIntValidator>
#include <QLabel>
#include <QLineEdit>
#include <QList>
#include <QLocale>
#include <QProgressBar>
#include <QProgressDialog>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/types.h>

using std::vector;

using pcl::Indices;
using pcl::PointCloud;
using pcl::PointXYZRGBA;
using pcl::KdTreeFLANN;
using PointCloudConstPtr = pcl::KdTree<PointXYZRGBA>::PointCloudConstPtr;

class FilterPointCloudStrategy
{
public:
    virtual ~FilterPointCloudStrategy() {}

    virtual PointCloud<PointXYZRGBA> filterCloud(PointCloud<PointXYZRGBA> cloudToFilter, QWidget *parent) = 0;
    virtual bool prepareForExecution(QWidget *parent) = 0;
};

class GuidedFilterPointCloudStrategy : public FilterPointCloudStrategy
{
public:
    virtual PointCloud<PointXYZRGBA> filterCloud(PointCloud<PointXYZRGBA> cloudToFilter, QWidget *parent) override;
    virtual bool prepareForExecution(QWidget *parent) override;

private:
    double scalarMult(PointXYZRGBA a, PointXYZRGBA b);

    int neighborCountParameter;
    double filterQualityParameter;
};

class YeFilterPointCloudStrategy : public FilterPointCloudStrategy
{
public:
    virtual PointCloud<PointXYZRGBA> filterCloud(PointCloud<PointXYZRGBA> cloudToFilter, QWidget *parent) override;
    virtual bool prepareForExecution(QWidget *parent) override;

private:
    double alphaFunction(PointXYZRGBA pk_s, PointXYZRGBA pi);
    double betaFunction(PointXYZRGBA pk_s, PointXYZRGBA pk_i);

    double thetaFunction(double x);
    double etaFunction(double x);
    double distanceBetweenPoints(PointXYZRGBA a, PointXYZRGBA b);

    double radiusParameter;
    double repulsionParameter;
    int iterationNumber;
};

#endif // FILTERPOINTCLOUDSTRATEGY_H
