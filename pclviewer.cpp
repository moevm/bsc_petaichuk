#include "pclviewer.h"
#include "./ui_pclviewer.h"

#include <QFile>
#include <QFileDialog>
#include <QMessageBox>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

PCLViewer::PCLViewer(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::PCLViewer)
{
    ui->setupUi(this);

    this->setWindowTitle("PCD Viewer");
}

PCLViewer::~PCLViewer()
{
    delete ui;
}

void PCLViewer::on_openButton_clicked()
{

    QString fileName = QFileDialog::getOpenFileName(this, "Open File", "../", "PCD Files (*.pcd)");
    ui->fileEdit->setText(fileName);
}

void PCLViewer::on_reviewButton_clicked()
{
    QString fileName = ui->fileEdit->text();
    QFile file(fileName);
    if (!file.exists())
    {
        QMessageBox::warning(this, "Error", "File with the given path can't be found.");
        return;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>(fileName.toStdString(), *cloud);
    ui->sceneWindow->addCloud(cloud);
}
