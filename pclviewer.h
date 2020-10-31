#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <QWidget>

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
    void on_openButton_clicked();
    void on_reviewButton_clicked();

private:
    Ui::PCLViewer *ui;
};
#endif // PCLVIEWER_H
