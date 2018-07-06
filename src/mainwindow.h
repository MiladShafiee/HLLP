#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "robot.h"
#include <QMainWindow>
#include <taskspace.h>
#include <qcustomplot.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void Plot(TaskSpace Pelvis);
private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H




