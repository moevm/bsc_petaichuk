#include "pclviewer.h"
#include "./ui_pclviewer.h"

PCLViewer::PCLViewer(QWidget *parent)
    : QWidget(parent),
      isCloudFiltrated(false),
      ui(new Ui::PCLViewer),
      filtratedCloud(new PointCloud<PointXYZRGBA>),
      referenceCloud(new PointCloud<PointXYZRGBA>),
      filterStrategy(new YeFilterPointCloudStrategy()),
      statisticsManager(new StatisticsManager())
{
    ui->setupUi(this);

    FileLoader *plyFileLoader = new PLYFileLoader();
    FileLoader *pcdFileLoader = new PCDFileLoader();
    plyFileLoader->setNext(pcdFileLoader);
    fileLoader = plyFileLoader;

    FileSaver *plyFileSaver = new PLYFileSaver();
    FileSaver *pcdFileSaver = new PCDFileSaver();
    plyFileSaver->setNext(pcdFileSaver);
    fileSaver = plyFileSaver;

    const QList<QString> algorythms = {"Accurate 3D Pose Estimation From a Single Depth Image",
                                       "Guided 3D point cloud filtering"};
    const QStringList algorythmsList(algorythms);
    ui->filterAlgorythmsComboBox->addItems(algorythmsList);
    filterStrategy = new YeFilterPointCloudStrategy();
    this->setWindowTitle("Point Cloud Filtering Tool");
}

PCLViewer::~PCLViewer()
{
    delete ui;
    delete fileLoader;
    delete fileSaver;
    delete filterStrategy;
    delete statisticsManager;
}

void PCLViewer::on_openFileButton_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, "Открыть файл", "../", "Point Clouds Files (*.ply *.pcd)");
    ui->fileEdit->setText(fileName);
}

bool PCLViewer::checkFileExistance(QString fileName)
{
    QFile file(fileName);
    if (!file.exists())
    {
        QMessageBox::warning(this, "Ошибка", "Файл по заданному пути не найден.");
    }
    return file.exists();
}

void PCLViewer::on_loadReferenceButton_clicked()
{
    QString fileName = ui->fileEdit->text();
    for (QString format : loadFormats)
    {
        if (fileName.endsWith(format))
        {
            if (checkFileExistance(fileName))
            {
                fileLoader->handleFileLoading(fileName.toStdString(), referenceCloud);
                statisticsManager->resetInformationRelevance();
                QMessageBox::information(this, "Успех", "Эталонное облако точек успешно загружено!");
                return;
            }
            return;
        }
    }
    QMessageBox::warning(this, "Ошибка", "Формат файла не является допустимым (необходим формат .pcd или .ply).");
}

void PCLViewer::on_openFiltratedButton_clicked()
{
    ui->sceneWindow->showCloud(filtratedCloud);
}

void PCLViewer::on_openReferenceButton_clicked()
{
    ui->sceneWindow->showCloud(referenceCloud);
}

void PCLViewer::on_loadFiltratedButton_clicked()
{
    QString fileName = ui->fileEdit->text();
    for (QString format : loadFormats)
    {
        if (fileName.endsWith(format))
        {
            if (checkFileExistance(fileName))
            {
                fileLoader->handleFileLoading(fileName.toStdString(), filtratedCloud);
                statisticsManager->resetInformationRelevance();
                isCloudFiltrated = false;
                QMessageBox::information(this, "Успех", "Облако точек для фильтрации успешно загружено!");
                return;
            }
            return;
        }
    }
    QMessageBox::warning(this, "Ошибка", "Формат файла не является допустимым (необходим формат .pcd или .ply).");
}

void PCLViewer::on_filterAlgorythmsComboBox_currentIndexChanged(int index)
{
    delete filterStrategy;
    switch (index)
    {
    case 0:
        filterStrategy = new YeFilterPointCloudStrategy();
        break;
    case 1:
        filterStrategy = new GuidedFilterPointCloudStrategy();
        break;
    }
}

void PCLViewer::on_filterButton_clicked()
{
    if (!filtratedCloud->empty())
    {
        if (filterStrategy->prepareForExecution(this))
        {
            statisticsManager->resetInformationRelevance();
            steady_clock::time_point begin = steady_clock::now();
            *filtratedCloud = filterStrategy->filterCloud(*filtratedCloud, this);
            steady_clock::time_point end = steady_clock::now();

            duration<long, ratio<1, 1000>> timeDuration = duration_cast<milliseconds>(end - begin);
            statisticsManager->writeTime(timeDuration.count() / 1000.0);
            isCloudFiltrated = true;
            ui->sceneWindow->showCloud(filtratedCloud);
            QMessageBox::information(this, "Успех", "Фильтрация облака точек успешно проведена!");
        }
    }
    else
    {
        QMessageBox::warning(this, "Ошибка", "Сначала загрузите облако точек для фильтрации.");
    }
}

void PCLViewer::on_openSaveFileButton_clicked()
{
    QString saveFileName = QFileDialog::getSaveFileName(this, "Сохранить файл", "../", "Point Clouds Files (*.ply *.pcd)");
    ui->saveFileEdit->setText(saveFileName);
}

void PCLViewer::on_saveFiltratedButton_clicked()
{
    QString saveFileName = ui->saveFileEdit->text();
    for (QString format : saveFormats)
    {
        if (saveFileName.endsWith(format))
        {
            fileSaver->handleFileSaving(saveFileName.toStdString(), filtratedCloud);
            QMessageBox::information(this, "Успех", "Облако точек успешно сохранено!");
            return;
        }
    }
    QMessageBox::warning(this, "Ошибка", "Введите правильный формат файла для сохранения (.pcd или .ply).");
}

void PCLViewer::on_statisticsParamsButton_clicked()
{
    if (!filtratedCloud->empty())
    {
        if (isCloudFiltrated)
        {
            QString filterTime = QString::number(statisticsManager->getFilteringTime(), 'g', 3);
            QMessageBox::information(this, "Статистика по фильтрации",
                                     QString("Время фильтрации облака точек: %1 секунд").arg(filterTime));
            return;
        }
        QMessageBox::warning(this, "Ошибка", "Необходимо сначала отфильтровать облако точек.");
        return;
    }
    QMessageBox::warning(this, "Ошибка", "Сначала загрузите облако точек для фильтрации.");
}

void PCLViewer::on_compareWithReferenceButton_clicked()
{
    if (!filtratedCloud->empty() && !referenceCloud->empty())
    {
        if (isCloudFiltrated)
        {
            double averageDistance = statisticsManager->getAverageDistanceBetweenPointsInPointClouds(*filtratedCloud,
                                                                                                     *referenceCloud);
            QString averageDistanceString = QString::number(averageDistance, 'g', 6);
            QMessageBox::information(this, "Сравнение облаков точек",
                                     QString("Среднее расстояние между точками: %1").arg(averageDistanceString));
            return;
        }
        QMessageBox::warning(this, "Ошибка", "Необходимо сначала отфильтровать облако точек.");
        return;
    }
    QMessageBox::warning(this, "Ошибка", "Для сравнения необходимы два облака точек (эталон и фильтруемое облако).");
}
