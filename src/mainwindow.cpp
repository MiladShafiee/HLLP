#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);


//    Charts=new ChartForm();

//   Charts->show();
   //Charts.Plot( Pelvis);

    //setGeometry(400, 250, 542, 390);
}





void MainWindow::Plot(TaskSpaceOffline Pelvis){

//    Charts=new ChartForm();
//    Charts->show();
//    Charts->Plot( Pelvis);


    Chartsfoot=new ChartForm();
    Chartsfoot->show();
    Chartsfoot->PlotFootOffline(Pelvis);

    ChartsPelvisOffline=new ChartForm();
    ChartsPelvisOffline->show();
    ChartsPelvisOffline->PlotOfflinePelvis(Pelvis);

//    Chartsfoot=new ChartForm();
//    Chartsfoot->show();
//    Chartsfoot->PlotFoot(Pelvis);

//    ChartsfootTime=new ChartForm();
//    ChartsfootTime->show();
//    ChartsfootTime->PlotFootTime(Pelvis);


//    ChartsfootVel=new ChartForm();
//    ChartsfootVel->show();
//    ChartsfootVel->PlotFootVel(Pelvis);


//    ChartsfootAccel=new ChartForm();
//    ChartsfootAccel->show();
//    ChartsfootAccel->PlotFootAccel(Pelvis);


//    Charts1=new ChartForm();
//    Charts1->show();
//    Charts1->Plot( Pelvis);


}


MainWindow::~MainWindow()
{
    delete ui;
}
