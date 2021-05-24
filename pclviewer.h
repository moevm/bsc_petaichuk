#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <chrono>
#include <ctime>

#include <QWidget>
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>
#include <QDebug>
#include <QList>

#include "fileloader.h"
#include "filesaver.h"
#include "filterpointcloudstrategy.h"
#include "statisticsmanager.h"

using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::steady_clock;
using std::ratio;

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
    void on_statisticsParamsButton_clicked();
    void on_compareWithReferenceButton_clicked();

    void on_filterAlgorythmsComboBox_currentIndexChanged(int index);

private:
    bool checkFileExistance(QString fileName);

    bool isCloudFiltrated;
    const QList<QString> loadFormats = {".ply", ".pcd"};
    const QList<QString> saveFormats = {".ply", ".pcd"};

    Ui::PCLViewer *ui;
    PointCloud<PointXYZRGBA>::Ptr filtratedCloud;
    PointCloud<PointXYZRGBA>::Ptr referenceCloud;
    FilterPointCloudStrategy *filterStrategy;
    FileLoader *fileLoader;
    FileSaver *fileSaver;
    StatisticsManager *statisticsManager;
};
#endif // PCLVIEWER_H
