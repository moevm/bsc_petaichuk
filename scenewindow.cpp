#include "scenewindow.h"

#include <vtkPointPicker.h>
#include <vtkGenericOpenGLRenderWindow.h>

SceneWindow::SceneWindow(QWidget *parent) : QVTKOpenGLWidget(parent)
{
    auto renderer = vtkSmartPointer <vtkRenderer>::New();
    m_renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    m_renderWindow->AddRenderer(renderer);
    m_viewer.reset(new PCLVisualizer(renderer, m_renderWindow, "cloud", false));
    m_viewer->setBackgroundColor(0.2, 0.2, 0.2, 0);
    this->SetRenderWindow(m_renderWindow);
    this->update();
}

void SceneWindow::showCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    if (!m_viewer->updatePointCloud(cloud, "cloud"))
    {
        m_viewer->addPointCloud(cloud, "cloud");
    }
    m_renderWindow->Render();
}
