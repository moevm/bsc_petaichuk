#include "filterpointcloudstrategy.h"

pcl::PointCloud<pcl::PointXYZRGBA> GuidedFilterPointCloudStrategy::filterCloud(pcl::PointCloud<pcl::PointXYZRGBA> cloudToFilter, QWidget *)
{
    return cloudToFilter;
}

pcl::PointCloud<pcl::PointXYZRGBA> YeFilterPointCloudStrategy::filterCloud(pcl::PointCloud<pcl::PointXYZRGBA> cloudToFilter, QWidget *parent)
{
    pcl::PointCloud<pcl::PointXYZRGBA> resultCloud = cloudToFilter;
    unsigned long cloudSize = resultCloud.size();

    QProgressDialog progressDialog("Выполнение фильтрации облака точек", "", 0, (iterationNumber + 1) * cloudSize, parent);
    progressDialog.setMinimumWidth(350);
    progressDialog.setWindowModality(Qt::WindowModal);
    progressDialog.setCancelButton(0);
    progressDialog.setValue(0);
    progressDialog.show();
    QApplication::processEvents();

    for (unsigned long i = 0; i < resultCloud.size(); i++)
    {
        double numeratorSum = 0.0;
        double denominatorSum = 0.0;
        for (unsigned long s = 0; s < cloudSize; s++)
        {
            numeratorSum += cloudToFilter[s].z * thetaFunction(distanceBetweenPoints(cloudToFilter[s], cloudToFilter[i]));
            denominatorSum += thetaFunction(distanceBetweenPoints(cloudToFilter[s], cloudToFilter[i]));
        }
        resultCloud[i].z = numeratorSum / denominatorSum;
        resultCloud[i].x = resultCloud[i].z * cloudToFilter[i].x / cloudToFilter[i].z;
        resultCloud[i].y = resultCloud[i].z * cloudToFilter[i].y / cloudToFilter[i].z;
        progressDialog.setValue(progressDialog.value() + 1);
        QApplication::processEvents();
    }
    for (int j = 0; j < iterationNumber; j++)
    {
        pcl::PointCloud<pcl::PointXYZRGBA> tempCloud = resultCloud;
        for (unsigned long i = 0; i < cloudSize; i++)
        {
            double firstNumeratorSum = 0.0;
            double firstDenominatorSum = 0.0;
            double secondNumeratorSum = 0.0;
            double secondDenominatorSum = 0.0;
            for (unsigned long s = 0; s < resultCloud.size(); s++)
            {
                firstNumeratorSum += cloudToFilter[s].z * alphaFunction(resultCloud[s], cloudToFilter[i]);
                firstDenominatorSum += alphaFunction(resultCloud[s], cloudToFilter[i]);
                if (s != i)
                {
                    secondNumeratorSum += distanceBetweenPoints(resultCloud[i], resultCloud[s]) * betaFunction(resultCloud[s], resultCloud[i]);
                    secondDenominatorSum += betaFunction(resultCloud[s], resultCloud[i]);
                }
            }
            tempCloud[i].z = firstNumeratorSum / firstDenominatorSum + repulsionParameter * (secondNumeratorSum / secondDenominatorSum);
            tempCloud[i].x = tempCloud[i].z * cloudToFilter[i].x / cloudToFilter[i].z;
            tempCloud[i].y = tempCloud[i].z * cloudToFilter[i].y / cloudToFilter[i].z;
            progressDialog.setValue(progressDialog.value() + 1);
            QApplication::processEvents();
        }
        resultCloud = tempCloud;
    }
    progressDialog.close();
    return resultCloud;
}

void YeFilterPointCloudStrategy::prepareForExecution()
{
    radiusParameter = QInputDialog::getDouble(nullptr, "Ввод параметров", "Введите значения локального радиуса: ", 0.05, 0, 1000, 3);
    repulsionParameter = QInputDialog::getDouble(nullptr, "Ввод параметров", "Введите значения параметра отталкивания: ", 0.35, 0, 1000, 3);
    iterationNumber = QInputDialog::getInt(nullptr, "Ввод параметров", "Введите число итераций: ", 1, 1, 10);
}

double YeFilterPointCloudStrategy::alphaFunction(pcl::PointXYZRGBA pk_s, pcl::PointXYZRGBA pi)
{
    double distance = distanceBetweenPoints(pk_s, pi);
    return thetaFunction(distance) / distance;
}

double YeFilterPointCloudStrategy::betaFunction(pcl::PointXYZRGBA pk_s, pcl::PointXYZRGBA pk_i)
{
    double distance = distanceBetweenPoints(pk_s, pk_i);
    return (thetaFunction(distance) / distance) * abs(etaFunction(distance));
}

double YeFilterPointCloudStrategy::thetaFunction(double x)
{
    return exp(-16 * pow(x, 2) / pow(radiusParameter, 2));
}

double YeFilterPointCloudStrategy::etaFunction(double x)
{
    return -1 / pow(x, 4);
}

double YeFilterPointCloudStrategy::distanceBetweenPoints(pcl::PointXYZRGBA a, pcl::PointXYZRGBA b)
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}
