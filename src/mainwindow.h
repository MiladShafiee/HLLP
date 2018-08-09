#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "robot.h"
#include <QMainWindow>
#include <taskspace.h>
#include <qcustomplot.h>
#include"chartform.h"
#include <taskspaceoffline.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    ChartForm *Charts;
     ChartForm *Charts1;
ChartForm *Chartsfoot;
ChartForm *ChartsPelvisOffline;

ChartForm *ChartsfootTime;

ChartForm *ChartsfootVel;
ChartForm  *ChartsfootAccel;
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void Plot(TaskSpaceOffline Pelvis);
private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H




