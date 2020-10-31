#include "pclviewer.h"

#include <QApplication>

#include <QVTKOpenGLWidget.h>
#include <QSurfaceFormat>

int main(int argc, char *argv[])
{
    auto format = QVTKOpenGLWidget::defaultFormat();
    format.setProfile(QSurfaceFormat::CompatibilityProfile);
    QSurfaceFormat::setDefaultFormat(format);

    QApplication a(argc, argv);
    PCLViewer w;
    w.show();

    return a.exec();
}
