#include "mainwindow.h"
#include <QApplication>
#include "Eigen/Dense"
#include "robot.h"
#include <Eigen/Geometry>
#include <iostream>
#include <cstdlib>
#include <qdebug.h>
#include <qmath.h>
#include <taskspace.h>
using namespace Eigen;
using namespace std;
int main(int argc, char *argv[])
{

    // SQDTaskSpace Forwardwalking();
    //    MatrixXd PelvisTrajectory;
    // PelvisTrajectory.setLinSpaced(81,0,0.4);
    // MatrixXd AnkleTrajectory;
    //AnkleTrajectory.setLinSpaced(81,0,0.4);
    //Robot Surena4(PelvisTrajectory,AnkleTrajectory);

    TaskSpace Pelvis;
    //QApplication a(argc, argv);
    // MainWindow w;
    // QList<Link> temp = Surena.GetLinks();
    //w.show();
    //MatrixXd temp(2,3);
    //temp<< 1,2,3,4,5,6;
    //return a.exec();
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    w.Plot(Pelvis);
    return a.exec();
}
