#include "scenewindow.h"

#include <vtkPointPicker.h>
#include <vtkGenericOpenGLRenderWindow.h>

SceneWindow::SceneWindow(QWidget *parent) : QVTKOpenGLWidget(parent)
{
    auto renderer = vtkSmartPointer <vtkRenderer>::New();
    m_renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    m_renderWindow->AddRenderer(renderer);
    m_viewer.reset(new PCLVisualizer(renderer, m_renderWindow, "cloud", false));
    this->SetRenderWindow(m_renderWindow);
    this->update();
}

void SceneWindow::addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    if (!m_viewer->updatePointCloud(cloud, "cloud"))
    {
        m_viewer->addPointCloud(cloud, "cloud");
    }
    m_renderWindow->Render();
}
