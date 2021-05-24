#include "statisticsmanager.h"

StatisticsManager::StatisticsManager() :
    isInformationActual(false), filteringTime(0.0)
{}

void StatisticsManager::writeTime(double filterTime)
{
    filteringTime = filterTime;
}

double StatisticsManager::getFilteringTime()
{
    return filteringTime;
}

void StatisticsManager::resetInformationRelevance()
{
    isInformationActual = false;
}

double StatisticsManager::getAverageDistanceBetweenPointsInPointClouds(PointCloud<pcl::PointXYZRGBA> filtratedCloud, PointCloud<pcl::PointXYZRGBA> referenceCloud)
{
    if (isInformationActual)
    {
        return averageDistanceBetweenPoints;
    }
    isInformationActual = true;
    return calculateAverageDistanceBetweenPoints(filtratedCloud, referenceCloud);
}

double StatisticsManager::calculateAverageDistanceBetweenPoints(PointCloud<pcl::PointXYZRGBA> filtratedCloud, PointCloud<pcl::PointXYZRGBA> referenceCloud)
{
    double sumDistance = 0.0;
    vector<PointXYZRGBA, Eigen::aligned_allocator<PointXYZRGBA>> referenceCloudPoints = referenceCloud.points;

    QProgressDialog progressDialog("Расчёт среднего расстояния между точками", "", 0, filtratedCloud.size(), nullptr);
    progressDialog.setMinimumWidth(350);
    progressDialog.setWindowModality(Qt::WindowModal);
    progressDialog.setCancelButton(0);
    progressDialog.setValue(0);
    progressDialog.show();
    QApplication::processEvents();

    for (PointXYZRGBA point : filtratedCloud)
    {
        double minDistance = numeric_limits<double>::infinity();
        double nowDistance;
        int referenceCloudPointIndex;
        if (referenceCloudPoints.size() == 0)
        {
            progressDialog.setValue(filtratedCloud.size());
            QApplication::processEvents();
            break;
        }
        for (unsigned long i = 0; i < referenceCloudPoints.size(); i++)
        {
            nowDistance = calculateDistanceBetweenPoints(point, referenceCloudPoints[i]);
            if (nowDistance < minDistance)
            {
                referenceCloudPointIndex = i;
                minDistance = nowDistance;
            }
        }
        sumDistance += minDistance;
        referenceCloudPoints.erase(referenceCloudPoints.begin() + referenceCloudPointIndex);
        progressDialog.setValue(progressDialog.value() + 1);
        QApplication::processEvents();
    }
    progressDialog.cancel();
    averageDistanceBetweenPoints = sumDistance / filtratedCloud.size();
    return averageDistanceBetweenPoints;
}

double StatisticsManager::calculateDistanceBetweenPoints(pcl::PointXYZRGBA a, pcl::PointXYZRGBA b)
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

