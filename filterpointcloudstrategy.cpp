#include "filterpointcloudstrategy.h"

PointCloud<PointXYZRGBA> GuidedFilterPointCloudStrategy::filterCloud(PointCloud<PointXYZRGBA> cloudToFilter, QWidget *parent)
{
    PointCloud<PointXYZRGBA> resultCloud;
    PointCloudConstPtr inputCloudPtr(new PointCloud<PointXYZRGBA>(cloudToFilter));
    KdTreeFLANN<PointXYZRGBA> kdTree;
    Indices indices = vector<int>(neighborCountParameter);
    vector<float> distances(neighborCountParameter);

    QProgressDialog progressDialog("Выполнение фильтрации облака точек", "", 0, cloudToFilter.size() + 100, parent);
    progressDialog.setMinimumWidth(350);
    progressDialog.setWindowModality(Qt::WindowModal);
    progressDialog.setCancelButton(0);
    progressDialog.setValue(0);
    progressDialog.show();
    QApplication::processEvents();

    kdTree.setInputCloud(inputCloudPtr);
    progressDialog.setValue(100);
    QApplication::processEvents();
    for (PointXYZRGBA nowPoint : cloudToFilter)
    {
        PointXYZRGBA centroid = nowPoint;
        PointXYZRGBA resultPoint = nowPoint;
        double scalarMultSum = 0.0;
        double coefficientANumerator;
        double coefficientA;
        PointXYZRGBA pointB;

        centroid.x = 0;
        centroid.y = 0;
        centroid.z = 0;
        int foundPointsCount = kdTree.nearestKSearch(nowPoint, neighborCountParameter, indices, distances);
        for (int i = 0; i < foundPointsCount; i++)
        {
            centroid.x += cloudToFilter[indices[i]].x;
            centroid.y += cloudToFilter[indices[i]].y;
            centroid.z += cloudToFilter[indices[i]].z;
        }
        centroid.x *= 1.0 / foundPointsCount;
        centroid.y *= 1.0 / foundPointsCount;
        centroid.z *= 1.0 / foundPointsCount;
        for (int i = 0; i < foundPointsCount; i++)
        {
            scalarMultSum += scalarMult(cloudToFilter[indices[i]], cloudToFilter[indices[i]]);
        }
        coefficientANumerator = scalarMultSum / foundPointsCount + scalarMult(centroid, centroid);
        coefficientA = coefficientANumerator / (coefficientANumerator + filterQualityParameter);
        pointB.x = centroid.x - centroid.x * coefficientA;
        pointB.y = centroid.y - centroid.y * coefficientA;
        pointB.z = centroid.z - centroid.z * coefficientA;
        resultPoint.x = coefficientA * centroid.x + pointB.x;
        resultPoint.y = coefficientA * centroid.y + pointB.y;
        resultPoint.z = coefficientA * centroid.z + pointB.z;
        resultCloud.push_back(resultPoint);
        progressDialog.setValue(progressDialog.value() + 1);
        QApplication::processEvents();
    }
    progressDialog.cancel();
    return resultCloud;
}

bool GuidedFilterPointCloudStrategy::prepareForExecution(QWidget *parent)
{
    QDialog dialog(parent);
    QFormLayout form(&dialog);
    QList<QLineEdit *> fields;
    QList<QString> labels = {
        "Введите значение числа вершин для алгоритма KNN (k-nearest neighbors): ",
        "Введите значение параметра качества фильтрации: "
    };
    QList<QValidator *> validators = {
        new QIntValidator(1, 1000, &dialog),
        new QDoubleValidator(0, 2, 3, &dialog),
    };
    QList<QString> defaultTexts = {"50", "0,04"};

    dialog.setWindowTitle("Ввод параметров");
    for (int i = 0; i < labels.size(); i++)
    {
        QLineEdit *lineEdit = new QLineEdit(&dialog);
        lineEdit->setValidator(validators[i]);
        lineEdit->setText(defaultTexts[i]);
        form.addRow(labels[i], lineEdit);
        fields << lineEdit;
    }
    QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                               Qt::Horizontal, &dialog);
    QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
    QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));
    form.addRow(&buttonBox);
    if (dialog.exec() == QDialog::Accepted)
    {
        QLocale locale(QLocale::Russian, QLocale::CyrillicScript, QLocale::Russia);
        neighborCountParameter = fields[0]->text().toInt();
        filterQualityParameter = locale.toDouble(fields[1]->text());
        return true;
    }
    return false;
}

double GuidedFilterPointCloudStrategy::scalarMult(pcl::PointXYZRGBA a, pcl::PointXYZRGBA b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

PointCloud<PointXYZRGBA> YeFilterPointCloudStrategy::filterCloud(PointCloud<PointXYZRGBA> cloudToFilter, QWidget *parent)
{
    PointCloud<PointXYZRGBA> resultCloud = cloudToFilter;
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
        PointCloud<PointXYZRGBA> tempCloud = resultCloud;
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
    for (unsigned long i = 0; i < resultCloud.size();)
    {
        if (resultCloud[i].x != resultCloud[i].x || resultCloud[i].y != resultCloud[i].y || resultCloud[i].z != resultCloud[i].z)
        {
            resultCloud.erase(resultCloud.begin() + i);
            continue;
        }
        i++;
    }
    progressDialog.close();
    return resultCloud;
}

bool YeFilterPointCloudStrategy::prepareForExecution(QWidget *parent)
{
    QDialog dialog(parent);
    QFormLayout form(&dialog);
    QList<QLineEdit *> fields;
    QList<QString> labels = {
        "Введите значение параметра локального радиуса: ",
        "Введите значение параметра отталкивания: ",
        "Введите число итераций: "
    };
    QList<QValidator *> validators = {
        new QDoubleValidator(0, 1000, 3, &dialog),
        new QDoubleValidator(0, 0.5, 3, &dialog),
        new QIntValidator(1, 10, &dialog)
    };
    QList<QString> defaultTexts = {"1,000", "0,350", "1"};

    dialog.setWindowTitle("Ввод параметров");
    for (int i = 0; i < labels.size(); i++)
    {
        QLineEdit *lineEdit = new QLineEdit(&dialog);
        lineEdit->setValidator(validators[i]);
        lineEdit->setText(defaultTexts[i]);
        form.addRow(labels[i], lineEdit);
        fields << lineEdit;
    }
    QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                               Qt::Horizontal, &dialog);
    QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
    QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));
    form.addRow(&buttonBox);
    if (dialog.exec() == QDialog::Accepted)
    {
        QLocale locale(QLocale::Russian, QLocale::CyrillicScript, QLocale::Russia);
        radiusParameter = locale.toDouble(fields[0]->text());
        repulsionParameter = locale.toDouble(fields[1]->text());
        iterationNumber = fields[2]->text().toInt();
        return true;
    }
    return false;
}

double YeFilterPointCloudStrategy::alphaFunction(PointXYZRGBA pk_s, PointXYZRGBA pi)
{
    double distance = distanceBetweenPoints(pk_s, pi);
    return thetaFunction(distance) / distance;
}

double YeFilterPointCloudStrategy::betaFunction(PointXYZRGBA pk_s, PointXYZRGBA pk_i)
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

double YeFilterPointCloudStrategy::distanceBetweenPoints(PointXYZRGBA a, PointXYZRGBA b)
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}
