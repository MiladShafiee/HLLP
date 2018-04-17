#include "mainwindow.h"
#include <QApplication>
#include "Eigen/Dense"
#include "robot.h"
using namespace Eigen;
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
   // MainWindow w;
        Robot Surena;
 //   w.show();
 //MatrixXd temp(2,3);
 //temp<< 1,2,3,4,5,6;
    return a.exec();
}
