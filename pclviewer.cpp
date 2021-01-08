#include "pclviewer.h"
#include "./ui_pclviewer.h"

PCLViewer::PCLViewer(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::PCLViewer),
      filtratedCloud(new pcl::PointCloud<pcl::PointXYZRGBA>),
      referenceCloud(new pcl::PointCloud<pcl::PointXYZRGBA>)
{
    ui->setupUi(this);

    this->setWindowTitle("PCD Viewer");
}

PCLViewer::~PCLViewer()
{
    delete ui;
}

void PCLViewer::on_openFileButton_clicked()
{

    QString fileName = QFileDialog::getOpenFileName(this, "Open File", "../", "PLY Files (*.ply)");
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
        pcl::io::loadPLYFile<pcl::PointXYZRGBA>(fileName.toStdString(), *referenceCloud);
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

void PCLViewer::on_loadFiltrateddButton_clicked()
{
    QString fileName = ui->fileEdit->text();
    if (checkFileExistance(fileName))
    {
        pcl::io::loadPLYFile<pcl::PointXYZRGBA>(fileName.toStdString(), *filtratedCloud);
    }
}
