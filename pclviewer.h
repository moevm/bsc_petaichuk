#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <ctime>

#include <QWidget>
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>

#include "fileloader.h"
#include "filesaver.h"
#include "filterpointcloudstrategy.h"

QT_BEGIN_NAMESPACE
namespace Ui { class PCLViewer; }
QT_END_NAMESPACE

class PCLViewer : public QWidget
{
    Q_OBJECT

public:
    explicit PCLViewer(QWidget *parent = nullptr);
    ~PCLViewer();

private slots:
    void on_openFileButton_clicked();
    void on_loadFiltratedButton_clicked();
    void on_loadReferenceButton_clicked();
    void on_openFiltratedButton_clicked();
    void on_openReferenceButton_clicked();
    void on_filterButton_clicked();
    void on_openSaveFileButton_clicked();
    void on_saveFiltratedButton_clicked();

    void on_filterAlgorythmsComboBox_currentIndexChanged(int index);

private:
    bool checkFileExistance(QString fileName);

    Ui::PCLViewer *ui;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtratedCloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr referenceCloud;
    FilterPointCloudStrategy *filterStrategy;
    FileLoader *fileLoader;
    FileSaver *fileSaver;
};
#endif // PCLVIEWER_H
