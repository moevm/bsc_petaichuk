#ifndef SCENEWINDOW_H
#define SCENEWINDOW_H

#include <QWidget>
#include <QVTKOpenGLWidget.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl::visualization;

class SceneWindow : public QVTKOpenGLWidget
{
    Q_OBJECT

public:
    explicit SceneWindow(QWidget *parent = nullptr);
    void addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

private:
    pcl::shared_ptr<PCLVisualizer> m_viewer;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renderWindow;
};

#endif // SCENEWINDOW_H
