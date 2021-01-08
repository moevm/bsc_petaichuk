#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <QWidget>
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

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
    void on_loadFiltrateddButton_clicked();
    void on_loadReferenceButton_clicked();
    void on_openFiltratedButton_clicked();
    void on_openReferenceButton_clicked();

private:
    bool checkFileExistance(QString fileName);

    Ui::PCLViewer *ui;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtratedCloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr referenceCloud;
};
#endif // PCLVIEWER_H
