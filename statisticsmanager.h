#ifndef STATISTICSMANAGER_H
#define STATISTICSMANAGER_H

#include <cmath>
#include <limits>
#include <vector>

#include <QApplication>
#include <QProgressDialog>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using std::numeric_limits;
using std::vector;

using pcl::PointXYZRGBA;
using pcl::PointCloud;

class StatisticsManager
{
public:
    StatisticsManager();

    void writeTime(double filterTime);
    double getFilteringTime();

    void resetInformationRelevance();

    double getAverageDistanceBetweenPointsInPointClouds(PointCloud<PointXYZRGBA> filtratedCloud,
                                                        PointCloud<PointXYZRGBA> referenceCloud);
private:
    double calculateAverageDistanceBetweenPoints(PointCloud<PointXYZRGBA> filtratedCloud,
                                                 PointCloud<PointXYZRGBA> referenceCloud);
    double calculateDistanceBetweenPoints(PointXYZRGBA a, PointXYZRGBA b);

    bool isInformationActual;
    double filteringTime;
    double averageDistanceBetweenPoints;
};

#endif // STATISTICSMANAGER_H
