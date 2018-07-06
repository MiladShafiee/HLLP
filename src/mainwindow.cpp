#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}





void MainWindow::Plot(TaskSpace Pelvis){
//    QVector<double> tComYV(Pelvis.tComdY.rows());
//    QVector<double> tCopYV(Pelvis.tCopY.rows());

//    memcpy(tComYV.data(),Pelvis.tComY.data(),sizeof(double)*Pelvis.tComdY.rows());
//    memcpy(tCopYV.data(),Pelvis.tCopY.data(),sizeof(double)*Pelvis.tCopY.rows());

    QCPLayoutGrid *subLayout2 = new QCPLayoutGrid;

     QCPAxisRect *leftAxisRect2 = new QCPAxisRect(ui->widget);
    subLayout2->addElement(0, 0, leftAxisRect2);
    leftAxisRect2->axis(QCPAxis::atLeft, 0)->setTickLabels(true);
    leftAxisRect2->axis(QCPAxis::atBottom, 0)->setTickLabels(true);
    leftAxisRect2->axis(QCPAxis::atBottom)->grid()->setVisible(true);

    ui->widget->plotLayout()->addElement(0, 0, subLayout2);



ui->widget->legend->setVisible(true);
    QCPGraph *bottomLeft = ui->widget->addGraph();
    bottomLeft->setName("ComY");
    bottomLeft->setData(Pelvis.timeVector,Pelvis.DCMXVector);
    bottomLeft->setLineStyle(QCPGraph::lsLine);
    bottomLeft->setPen(QPen(QColor("#A1FF00"), 1.5));
    bottomLeft->rescaleAxes();


    QCPGraph *bottomright = ui->widget->addGraph();
    bottomright->setName("CoPY");
    bottomright->setData(Pelvis.timeVector,Pelvis.EndCoPYVector);
    bottomright->setLineStyle(QCPGraph::lsLine);
    bottomright->setPen(QPen(QColor("#FFA100"), 2));
    bottomright->rescaleAxes();


    QCPLegend *legend5 = new QCPLegend();
       legend5->setLayer("legend5");
       leftAxisRect2->insetLayout()->addElement(legend5,Qt::AlignTop|Qt::AlignTop);
       legend5->addItem(new QCPPlottableLegendItem(legend5, bottomLeft));
       legend5->addItem(new QCPPlottableLegendItem(legend5, bottomLeft));


}


MainWindow::~MainWindow()
{
    delete ui;
}
