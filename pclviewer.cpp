#include "pclviewer.h"
#include "./ui_pclviewer.h"

PCLViewer::PCLViewer(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::PCLViewer),
      filtratedCloud(new pcl::PointCloud<pcl::PointXYZRGBA>),
      referenceCloud(new pcl::PointCloud<pcl::PointXYZRGBA>),
      filterStrategy(new YeFilterPointCloudStrategy())
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

    const QList<QString> algorythms = {"Алогритм фильтрации Ye et al.", "Алгоритм управляемой фильтрации облака точек"};
    const QStringList algorythmsList(algorythms);
    ui->filterAlgorythmsComboBox->addItems(algorythmsList);
    filterStrategy = new YeFilterPointCloudStrategy();
    this->setWindowTitle("PCD Viewer");
}

PCLViewer::~PCLViewer()
{
    delete ui;
    delete fileLoader;
    delete filterStrategy;
}

void PCLViewer::on_openFileButton_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, "Open File", "../", "Point Clouds Files (*.ply *.pcd)");
    ui->fileEdit->setText(fileName);
}

bool PCLViewer::checkFileExistance(QString fileName)
{
    QFile file(fileName);
    if (!file.exists())
    {
        QMessageBox::warning(this, "Error", "File with the given path can't be found.");
    }
    return file.exists();
}

void PCLViewer::on_loadReferenceButton_clicked()
{
    QString fileName = ui->fileEdit->text();
    if (checkFileExistance(fileName))
    {
        fileLoader->handleFileLoading(fileName.toStdString(), referenceCloud);
    }
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
    if (checkFileExistance(fileName))
    {
        fileLoader->handleFileLoading(fileName.toStdString(), filtratedCloud);
    }
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
    filterStrategy->prepareForExecution();
    *filtratedCloud = filterStrategy->filterCloud(*filtratedCloud, this);
    ui->sceneWindow->showCloud(filtratedCloud);
}

void PCLViewer::on_openSaveFileButton_clicked()
{
    QString saveFileName = QFileDialog::getSaveFileName(this, "Save File", "../", "Point Clouds Files (*.ply *.pcd)");
    ui->saveFileEdit->setText(saveFileName);
}

void PCLViewer::on_saveFiltratedButton_clicked()
{
    QString saveFileName = ui->saveFileEdit->text();
    fileSaver->handleFileSaving(saveFileName.toStdString(), filtratedCloud);
}
