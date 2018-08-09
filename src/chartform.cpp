#include "chartform.h"
#include "ui_chartform.h"

ChartForm::ChartForm(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ChartForm)
{
    ui->setupUi(this);
    setGeometry(400, 250, 542, 390);

}

ChartForm::~ChartForm()
{
    delete ui;
}

void ChartForm::PlotOfflinePelvis(TaskSpaceOffline Pelvis){
    // ui->widget1->plotLayout()->clear(); // clear default axis rect so we can start from scratch

    //    ui->widget1->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    //    ui->widget1->legend->setVisible(true);
    //    QFont legendFont = font();  // start out with MainWindow's font..
    //    legendFont.setPointSize(9); // and make a bit smaller for legend
    //    ui->widget1->legend->setFont(legendFont);
    //    ui->widget1->legend->setBrush(QBrush(QColor(255,255,255,230)));
    //    // by default, the legend is in the inset layout of the main axis rect. So this is how we access it to change legend placement:
    //    ui->widget1->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom|Qt::AlignRight);

    //    // setup for graph 0: key axis left, value axis bottom
    //    // will contain left maxwell-like function
    //    ui->widget1->addGraph(ui->widget1->yAxis, ui->widget1->xAxis);
    //    ui->widget1->graph(0)->setPen(QPen(QColor(255, 100, 0)));
    //    //ui->widget1->graph(0)->setBrush(QBrush(QPixmap("./balboa.jpg"))); // fill with texture of specified image
    //    ui->widget1->graph(0)->setLineStyle(QCPGraph::lsLine);
    //    ui->widget1->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 5));
    //    ui->widget1->graph(0)->setName("Left maxwell function");



    ui->widget1->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom));
    //ui->widget->legend->setVisible(true);
    ui->widget1->setAutoAddPlottableToLegend(false);

    ui->widget1->plotLayout()->clear();



    // ui->widget1->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom));
    //    ui->widget1->legend->setVisible(true);
    //    QFont legendFont = font();  // start out with MainWindow's font..
    //    legendFont.setPointSize(9); // and make a bit smaller for legend
    //    ui->widget1->legend->setFont(legendFont);
    //    ui->widget1->legend->setBrush(QBrush(QColor(255,255,255,230)));
    //ui->widget1->legend->setVisible(true);

    //ui->widget1->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom|Qt::AlignRight);
    QCPLayoutGrid *subLayout1 = new QCPLayoutGrid;
    QCPAxisRect *AxisRect1 = new QCPAxisRect(ui->widget1);
    AxisRect1->axis(QCPAxis::atLeft, 0)->setTickLabels(true);
    AxisRect1->axis(QCPAxis::atBottom, 0)->setTickLabels(true);
    AxisRect1->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    AxisRect1->setupFullAxesBox(true);
    //AxisRect1->addAxis(QCPAxis::atLeft)->setTickLabelColor(QColor("#6050F8")); // add an extra axis on the left and color its numbers
    AxisRect1->axis(QCPAxis::atRight, 0)->setTickLabels(true);
    QCPLayoutGrid *subLayout = new QCPLayoutGrid;
    subLayout1->addElement(0, 0, AxisRect1);
    ui->widget1->plotLayout()->addElement(0, 0, subLayout1);// sub layout in second row (grid layout will grow accordingly)



    QCPAxisRect *AxisRect2 = new QCPAxisRect(ui->widget1);
    AxisRect2->axis(QCPAxis::atLeft, 0)->setTickLabels(true);
    AxisRect2->axis(QCPAxis::atBottom, 0)->setTickLabels(true);
    AxisRect2->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    AxisRect2->setupFullAxesBox(true);
    AxisRect2->axis(QCPAxis::atRight, 0)->setTickLabels(true);

    ui->widget1->plotLayout()->addElement(1, 0, AxisRect2);// sub layout in second row (grid layout will grow accordingly)


    QCPAxisRect *AxisRect3 = new QCPAxisRect(ui->widget1);
    AxisRect3->axis(QCPAxis::atLeft, 0)->setTickLabels(true);
    AxisRect3->axis(QCPAxis::atBottom, 0)->setTickLabels(true);
    AxisRect3->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    AxisRect3->setupFullAxesBox(true);
    AxisRect3->axis(QCPAxis::atRight, 0)->setTickLabels(true);


    ui->widget1->plotLayout()->addElement(2, 0, AxisRect3);// sub layout in second row (grid layout will grow accordingly)


    foreach (QCPAxisRect *rect, ui->widget1->axisRects())
    {
        foreach (QCPAxis *axis, rect->axes())
        {
            axis->setLayer("axes");
            axis->grid()->setLayer("grid");
        }
    }



    QCPGraph *plot1 = ui->widget1->addGraph(AxisRect1->axis(QCPAxis::atBottom), AxisRect1->axis(QCPAxis::atLeft));
    plot1->keyAxis()->setLabel(" X Trajectory");
    plot1->valueAxis()->setLabel("Y Trajectory");
    plot1->setName("CoM");
    plot1->setData(Pelvis.timeVector,Pelvis.CoMXVector);
    // plot1->valueAxis()->setRange(-0.2, 0.2);
    // plot1->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, QPen(Qt::green), QBrush(Qt::green), 1.5));
    plot1->setName("CoM");
    plot1->setLineStyle(QCPGraph::lsLine);
    plot1->setPen(QPen(QColor("#00ff00"), 2.5));
    plot1->rescaleAxes();


    //    QCPGraph *plot11 = ui->widget1->addGraph(AxisRect1->axis(QCPAxis::atBottom), AxisRect1->axis(QCPAxis::atLeft));
    //    plot11->setData(Pelvis.DCMXVector,Pelvis.DCMYVector);

    //    plot11->setLineStyle(QCPGraph::lsLine);
    //    plot11->rescaleKeyAxis();
    //    //plot11->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, QPen(Qt::blue), QBrush(Qt::blue), 1.5));
    //    plot11->setName("DCM");
    //    plot11->setPen(QPen(QColor("blue"), 3));



    //    QCPGraph *plot111 = ui->widget1->addGraph(AxisRect1->axis(QCPAxis::atBottom), AxisRect1->axis(QCPAxis::atLeft));
    //    plot111->setData(Pelvis.EndCoPXVector,Pelvis.EndCoPYVector);
    //    plot111->valueAxis()->setRange(-0.2, 0.4);
    //    plot111->setLineStyle(QCPGraph::lsLine);
    //    plot111->rescaleKeyAxis();
    //    plot111->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::red), QBrush(Qt::red), 3));
    //    plot111->setName("CoP");
    //    // ui->widget1->plotLayout()->addElement(3, 0, new QCPTextElement(ui->widget1, "Way too many graphs in one plot", QFont("sans", 12, QFont::Bold)));
    //    plot111->setPen(QPen(QColor("magenta"), 2.5));



    QCPLegend *legend1 = new QCPLegend();
    legend1->setLayer("legend1");
    AxisRect1->insetLayout()->addElement(legend1,Qt::AlignTop|Qt::AlignRight);
    legend1->addItem(new QCPPlottableLegendItem(legend1,plot1));
    //    legend1->addItem(new QCPPlottableLegendItem(legend1,plot11));
    //    legend1->addItem(new QCPPlottableLegendItem(legend1,plot111));
    legend1->setBrush(QBrush(QColor(255,255,255,150)));
    //legend1->setVisible(true);


    QCPGraph *plot2 = ui->widget1->addGraph(AxisRect2->axis(QCPAxis::atBottom), AxisRect2->axis(QCPAxis::atLeft));
    plot2->setName("CoMX");
    plot2->setData(Pelvis.timeVector,Pelvis.CoMYVector);
    plot2->keyAxis()->setLabel("time(s)");
    plot2->valueAxis()->setLabel("X Trajectory");
    // plot1->valueAxis()->setRange(-0.2, 0.2);
    // plot2->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::green), QBrush(Qt::green), 2));
    plot2->keyAxis()->setLabel("time(s)");
    plot2->valueAxis()->setLabel("X trajectory");
    plot2->setLineStyle(QCPGraph::lsLine);
    //    QPen blueDotPen;
    //    blueDotPen.setColor(QColor(30, 40, 255, 150));
    //    blueDotPen.setStyle(Qt::DotLine);
    plot2->setPen(QPen(QColor("#00ff00"), 2));
    plot2->rescaleAxes();




    //    QCPGraph *plot22 = ui->widget1->addGraph(AxisRect2->axis(QCPAxis::atBottom), AxisRect2->axis(QCPAxis::atLeft));
    //    plot22->setData(Pelvis.timeVector,Pelvis.DCMXVector);
    //    //plot22->valueAxis()->setRange(-0.2, 0.2);
    //    plot22->setLineStyle(QCPGraph::lsLine);
    // plot22->setPen(QPen(QColor("blue"), 2));
    //    plot22->rescaleKeyAxis();
    //    plot22->setName("DCMX");
    //    // ui->widget1->plotLayout()->addElement(3, 0, new QCPTextElement(ui->widget1, "Way too many graphs in one plot", QFont("sans", 12, QFont::Bold)));



    //    QCPGraph *plot222 = ui->widget1->addGraph(AxisRect2->axis(QCPAxis::atBottom), AxisRect2->axis(QCPAxis::atLeft));
    //    plot222->setData(Pelvis.timeVector,Pelvis.EndCoPXVector);
    //    // plot222->valueAxis()->setRange(-0.2, 0.2);
    //    plot222->setLineStyle(QCPGraph::lsLine);
    //    plot222->rescaleKeyAxis();
    //    plot222->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, QPen(Qt::magenta), QBrush(Qt::red), 2));
    //     plot222->setPen(QPen(QColor("magenta"), 2.5));
    //    plot222->setName("CoPX");



    QCPLegend *legend2 = new QCPLegend();
    legend2->setLayer("legend1");
    AxisRect2->insetLayout()->addElement(legend2,Qt::AlignBottom|Qt::AlignRight);
    legend2->addItem(new QCPPlottableLegendItem(legend2,plot2));
    //    legend2->addItem(new QCPPlottableLegendItem(legend2,plot22));
    //    legend2->addItem(new QCPPlottableLegendItem(legend2,plot222));



    QCPGraph *plot3 = ui->widget1->addGraph(AxisRect3->axis(QCPAxis::atBottom), AxisRect3->axis(QCPAxis::atLeft));
    plot3->setName("CoMY");
    plot3->setData(Pelvis.timeVector,Pelvis.CoMZVector);
    plot3->keyAxis()->setLabel("time(s)");
    plot3->valueAxis()->setLabel("Y trajectory");
    // plot1->valueAxis()->setRange(-0.2, 0.2);
    //plot3->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::green), 2));
    plot3->keyAxis()->setLabel("time(s)");
    plot3->valueAxis()->setLabel("Y Trajectory");
    plot3->setLineStyle(QCPGraph::lsLine);
    plot3->setPen(QPen(QColor("#00ff00"), 2));
    plot3->rescaleAxes();




    //    QCPGraph *plot33 = ui->widget1->addGraph(AxisRect3->axis(QCPAxis::atBottom), AxisRect3->axis(QCPAxis::atLeft));
    //    plot33->setData(Pelvis.timeVector,Pelvis.DCMYVector);
    //    plot33->valueAxis()->setRange(-0.2, 0.5);
    //    plot33->setLineStyle(QCPGraph::lsLine);
    //    plot33->rescaleKeyAxis();
    //    //plot33->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::blue), 1.5));
    //    plot33->setName("DCMY");
    //    plot33->setPen(QPen(QColor("blue"), 2));
    //    // ui->widget1->plotLayout()->addElement(3, 0, new QCPTextElement(ui->widget1, "Way too many graphs in one plot", QFont("sans", 12, QFont::Bold)));


    //    QCPGraph *plot333 = ui->widget1->addGraph(AxisRect3->axis(QCPAxis::atBottom), AxisRect3->axis(QCPAxis::atLeft));
    //    plot333->setData(Pelvis.timeVector,Pelvis.EndCoPYVector);
    //    plot333->setLineStyle(QCPGraph::lsLine);
    //    plot333->rescaleKeyAxis();
    //    //plot333->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::magenta), 1.5));
    //    plot333->setName("CoPY");

    ////    plot333->setPen(QPen(QColor(120, 120, 120), 2));
    //    plot333->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, QPen(Qt::magenta), QBrush(Qt::red), 2));
    //     plot333->setPen(QPen(QColor("magenta"), 2.5));
    //    // ui->widget1->plotLayout()->addElement(3, 0, new QCPTextElement(ui->widget1, "Way too many graphs in one plot", QFont("sans", 12, QFont::Bold)));


    //    QCPLegend *legend3 = new QCPLegend();
    //    legend3->setLayer("legend3");
    //    AxisRect3->insetLayout()->addElement(legend3,Qt::AlignTop|Qt::AlignRight);
    //    legend3->addItem(new QCPPlottableLegendItem(legend3,plot3));
    //    legend3->addItem(new QCPPlottableLegendItem(legend3,plot33));
    //    legend3->addItem(new QCPPlottableLegendItem(legend3,plot333));
       ui->widget1->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
}


void ChartForm::PlotFootOffline(TaskSpaceOffline Pelvis){
    // ui->widget1->plotLayout()->clear(); // clear default axis rect so we can start from scratch

    //    ui->widget1->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    //    ui->widget1->legend->setVisible(true);
    //    QFont legendFont = font();  // start out with MainWindow's font..
    //    legendFont.setPointSize(9); // and make a bit smaller for legend
    //    ui->widget1->legend->setFont(legendFont);
    //    ui->widget1->legend->setBrush(QBrush(QColor(255,255,255,230)));
    //    // by default, the legend is in the inset layout of the main axis rect. So this is how we access it to change legend placement:
    //    ui->widget1->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom|Qt::AlignRight);

    //    // setup for graph 0: key axis left, value axis bottom
    //    // will contain left maxwell-like function
    //    ui->widget1->addGraph(ui->widget1->yAxis, ui->widget1->xAxis);
    //    ui->widget1->graph(0)->setPen(QPen(QColor(255, 100, 0)));
    //    //ui->widget1->graph(0)->setBrush(QBrush(QPixmap("./balboa.jpg"))); // fill with texture of specified image
    //    ui->widget1->graph(0)->setLineStyle(QCPGraph::lsLine);
    //    ui->widget1->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 5));
    //    ui->widget1->graph(0)->setName("Left maxwell function");

    ui->widget1->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom));
    //ui->widget->legend->setVisible(true);
    ui->widget1->setAutoAddPlottableToLegend(false);

    ui->widget1->plotLayout()->clear();



    // ui->widget1->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom));
    //    ui->widget1->legend->setVisible(true);
    //    QFont legendFont = font();  // start out with MainWindow's font..
    //    legendFont.setPointSize(9); // and make a bit smaller for legend
    //    ui->widget1->legend->setFont(legendFont);
    //    ui->widget1->legend->setBrush(QBrush(QColor(255,255,255,230)));
    //ui->widget1->legend->setVisible(true);

    //ui->widget1->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom|Qt::AlignRight);
    QCPLayoutGrid *subLayout1 = new QCPLayoutGrid;
    QCPAxisRect *AxisRect1 = new QCPAxisRect(ui->widget1);
    AxisRect1->axis(QCPAxis::atLeft, 0)->setTickLabels(true);
    AxisRect1->axis(QCPAxis::atBottom, 0)->setTickLabels(true);
    AxisRect1->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    AxisRect1->setupFullAxesBox(true);
    //AxisRect1->addAxis(QCPAxis::atLeft)->setTickLabelColor(QColor("#6050F8")); // add an extra axis on the left and color its numbers
    AxisRect1->axis(QCPAxis::atRight, 0)->setTickLabels(true);
    QCPLayoutGrid *subLayout = new QCPLayoutGrid;
    subLayout1->addElement(0, 0, AxisRect1);
    ui->widget1->plotLayout()->addElement(0, 0, subLayout1);// sub layout in second row (grid layout will grow accordingly)



    QCPAxisRect *AxisRect2 = new QCPAxisRect(ui->widget1);
    AxisRect2->axis(QCPAxis::atLeft, 0)->setTickLabels(true);
    AxisRect2->axis(QCPAxis::atBottom, 0)->setTickLabels(true);
    AxisRect2->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    AxisRect2->setupFullAxesBox(true);
    AxisRect2->axis(QCPAxis::atRight, 0)->setTickLabels(true);

    ui->widget1->plotLayout()->addElement(1, 0, AxisRect2);// sub layout in second row (grid layout will grow accordingly)


    QCPAxisRect *AxisRect3 = new QCPAxisRect(ui->widget1);
    AxisRect3->axis(QCPAxis::atLeft, 0)->setTickLabels(true);
    AxisRect3->axis(QCPAxis::atBottom, 0)->setTickLabels(true);
    AxisRect3->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    AxisRect3->setupFullAxesBox(true);
    AxisRect3->axis(QCPAxis::atRight, 0)->setTickLabels(true);


    ui->widget1->plotLayout()->addElement(2, 0, AxisRect3);// sub layout in second row (grid layout will grow accordingly)




    foreach (QCPAxisRect *rect, ui->widget1->axisRects())
    {
        foreach (QCPAxis *axis, rect->axes())
        {
            axis->setLayer("axes");
            axis->grid()->setLayer("grid");
        }
    }



    QCPGraph *plot1 = ui->widget1->addGraph(AxisRect1->axis(QCPAxis::atBottom), AxisRect1->axis(QCPAxis::atLeft));
    plot1->keyAxis()->setLabel(" X Foot Trajectory");
    plot1->valueAxis()->setLabel("Y Foot  Trajectory");
    plot1->setName("Right Foot");
    plot1->setData(Pelvis.timeVector,Pelvis.RightFootXTrajectory);

    // plot1->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, QPen(Qt::green), QBrush(Qt::green), 1.5));
    plot1->setLineStyle(QCPGraph::lsLine);
    plot1->setPen(QPen(QColor("#00ff00"), 2.5));
    plot1->rescaleAxes();


    QCPGraph *plot11 = ui->widget1->addGraph(AxisRect1->axis(QCPAxis::atBottom), AxisRect1->axis(QCPAxis::atLeft));
    plot11->setData(Pelvis.timeVector,Pelvis.LeftFootXTrajectory);
    //plot11->valueAxis()->setRange(-2, 2);
    plot11->setLineStyle(QCPGraph::lsLine);
    plot11->rescaleKeyAxis();
    //plot1->valueAxis()->setRange(0.7, 0.95);
    //plot11->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, QPen(Qt::blue), QBrush(Qt::blue), 1.5));
    plot11->setName("Left Foot");
    plot11->setPen(QPen(QColor("blue"), 3));



    ////    QCPGraph *plot111 = ui->widget1->addGraph(AxisRect1->axis(QCPAxis::atBottom), AxisRect1->axis(QCPAxis::atLeft));
    ////    plot111->setData(Pelvis.EndCoPXVector,Pelvis.EndCoPYVector);
    ////    plot111->valueAxis()->setRange(-0.2, 0.4);
    ////    plot111->setLineStyle(QCPGraph::lsLine);
    ////    plot111->rescaleKeyAxis();
    ////    plot111->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::red), QBrush(Qt::red), 3));
    ////    plot111->setName("CoP");
    ////    // ui->widget1->plotLayout()->addElement(3, 0, new QCPTextElement(ui->widget1, "Way too many graphs in one plot", QFont("sans", 12, QFont::Bold)));
    ////    plot111->setPen(QPen(QColor("magenta"), 2.5));



    QCPLegend *legend1 = new QCPLegend();
    legend1->setLayer("legend1");
    AxisRect1->insetLayout()->addElement(legend1,Qt::AlignTop|Qt::AlignRight);
    legend1->addItem(new QCPPlottableLegendItem(legend1,plot1));
    legend1->addItem(new QCPPlottableLegendItem(legend1,plot11));
    //legend1->addItem(new QCPPlottableLegendItem(legend1,plot111));
    legend1->setBrush(QBrush(QColor(255,255,255,150)));
    //legend1->setVisible(true);


    QCPGraph *plot2 = ui->widget1->addGraph(AxisRect2->axis(QCPAxis::atBottom), AxisRect2->axis(QCPAxis::atLeft));
    plot2->setName("Right Foot");
    plot2->setData(Pelvis.timeVector,Pelvis.RightFootYTrajectory);
    plot2->keyAxis()->setLabel("X Foot Trajectory");
    plot2->valueAxis()->setLabel("Z Foot Trajectory");
    // plot1->valueAxis()->setRange(-0.2, 0.2);
    // plot2->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::green), QBrush(Qt::green), 2));
    plot2->setLineStyle(QCPGraph::lsLine);
    //    QPen blueDotPen;
    //    blueDotPen.setColor(QColor(30, 40, 255, 150));
    //    blueDotPen.setStyle(Qt::DotLine);
    plot2->setPen(QPen(QColor("#00ff00"), 2));
    plot2->rescaleAxes();




    QCPGraph *plot22 = ui->widget1->addGraph(AxisRect2->axis(QCPAxis::atBottom), AxisRect2->axis(QCPAxis::atLeft));
    plot22->setData(Pelvis.timeVector,Pelvis.LeftFootYTrajectory);
    plot22->valueAxis()->setRange(-0.2, 0.2);
    plot22->setLineStyle(QCPGraph::lsLine);
    plot22->setPen(QPen(QColor("blue"), 2));
    plot22->rescaleKeyAxis();
    plot22->setName("Left Foot");
    // ui->widget1->plotLayout()->addElement(3, 0, new QCPTextElement(ui->widget1, "Way too many graphs in one plot", QFont("sans", 12, QFont::Bold)));







    QCPLegend *legend2 = new QCPLegend();
    legend2->setLayer("legend1");
    AxisRect2->insetLayout()->addElement(legend2,Qt::AlignBottom|Qt::AlignRight);
    legend2->addItem(new QCPPlottableLegendItem(legend2,plot2));
    legend2->addItem(new QCPPlottableLegendItem(legend2,plot22));
    //legend2->addItem(new QCPPlottableLegendItem(legend2,plot222));



    QCPGraph *plot3 = ui->widget1->addGraph(AxisRect3->axis(QCPAxis::atBottom), AxisRect3->axis(QCPAxis::atLeft));
    plot3->setName("Left Foot");
    plot3->setData(Pelvis.timeVector,Pelvis.LeftFootZTrajectory);
    plot3->keyAxis()->setLabel("time(s)");
    plot3->valueAxis()->setLabel("Z trajectory");
    // plot1->valueAxis()->setRange(-0.2, 0.2);
    //plot3->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::green), 2));
    plot3->keyAxis()->setLabel("Y Foot Trajectory");
    plot3->valueAxis()->setLabel("Z Foot Trajectory");
    plot3->setLineStyle(QCPGraph::lsLine);
    plot3->setPen(QPen(QColor("#00ff00"), 2));
    plot3->rescaleAxes();
    //    plot3->keyAxis()->setRange(-1, 1);
    //  plot3->valueAxis()->setRange(-0.2, 0.2);



    QCPGraph *plot33 = ui->widget1->addGraph(AxisRect3->axis(QCPAxis::atBottom), AxisRect3->axis(QCPAxis::atLeft));
    plot33->setData(Pelvis.timeVector,Pelvis.RightFootZTrajectory);
    //      plot33->keyAxis()->setRange(-1, 1);
    //    plot33->valueAxis()->setRange(-0.5, 0.5);

    plot33->setLineStyle(QCPGraph::lsLine);
    plot33->rescaleKeyAxis();
    //plot33->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::blue), 1.5));
    plot33->setName("Right Foot");
    plot33->setPen(QPen(QColor("blue"), 2));
    // ui->widget1->plotLayout()->addElement(3, 0, new QCPTextElement(ui->widget1, "Way too many graphs in one plot", QFont("sans", 12, QFont::Bold)));
    // plot33->keyAxis()->setRange(-0.15, 0.15);
    plot33->valueAxis()->setRange(0.1, 0.15);

    ////    QCPGraph *plot333 = ui->widget1->addGraph(AxisRect3->axis(QCPAxis::atBottom), AxisRect3->axis(QCPAxis::atLeft));
    ////    plot333->setData(Pelvis.timeVector,Pelvis.EndCoPYVector);
    ////    plot333->setLineStyle(QCPGraph::lsLine);
    ////    plot333->rescaleKeyAxis();
    ////    //plot333->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::magenta), 1.5));
    ////    plot333->setName("CoPY");

    ////    plot333->setPen(QPen(QColor(120, 120, 120), 2));
    ////    plot333->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, QPen(Qt::magenta), QBrush(Qt::red), 2));
    ////     plot333->setPen(QPen(QColor("magenta"), 2.5));
    ////    // ui->widget1->plotLayout()->addElement(3, 0, new QCPTextElement(ui->widget1, "Way too many graphs in one plot", QFont("sans", 12, QFont::Bold)));


    QCPLegend *legend3 = new QCPLegend();
    legend3->setLayer("legend3");
    AxisRect3->insetLayout()->addElement(legend3,Qt::AlignTop|Qt::AlignRight);
    legend3->addItem(new QCPPlottableLegendItem(legend3,plot3));
    legend3->addItem(new QCPPlottableLegendItem(legend3,plot33));
    //legend3->addItem(new QCPPlottableLegendItem(legend3,plot333));
    ui->widget1->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
}



void ChartForm::Plot(TaskSpace Pelvis){
    // ui->widget1->plotLayout()->clear(); // clear default axis rect so we can start from scratch

    //    ui->widget1->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    //    ui->widget1->legend->setVisible(true);
    //    QFont legendFont = font();  // start out with MainWindow's font..
    //    legendFont.setPointSize(9); // and make a bit smaller for legend
    //    ui->widget1->legend->setFont(legendFont);
    //    ui->widget1->legend->setBrush(QBrush(QColor(255,255,255,230)));
    //    // by default, the legend is in the inset layout of the main axis rect. So this is how we access it to change legend placement:
    //    ui->widget1->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom|Qt::AlignRight);

    //    // setup for graph 0: key axis left, value axis bottom
    //    // will contain left maxwell-like function
    //    ui->widget1->addGraph(ui->widget1->yAxis, ui->widget1->xAxis);
    //    ui->widget1->graph(0)->setPen(QPen(QColor(255, 100, 0)));
    //    //ui->widget1->graph(0)->setBrush(QBrush(QPixmap("./balboa.jpg"))); // fill with texture of specified image
    //    ui->widget1->graph(0)->setLineStyle(QCPGraph::lsLine);
    //    ui->widget1->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 5));
    //    ui->widget1->graph(0)->setName("Left maxwell function");



    ui->widget1->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom));
    //ui->widget->legend->setVisible(true);
    ui->widget1->setAutoAddPlottableToLegend(false);

    ui->widget1->plotLayout()->clear();



    // ui->widget1->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom));
    //    ui->widget1->legend->setVisible(true);
    //    QFont legendFont = font();  // start out with MainWindow's font..
    //    legendFont.setPointSize(9); // and make a bit smaller for legend
    //    ui->widget1->legend->setFont(legendFont);
    //    ui->widget1->legend->setBrush(QBrush(QColor(255,255,255,230)));
    //ui->widget1->legend->setVisible(true);

    //ui->widget1->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom|Qt::AlignRight);
    QCPLayoutGrid *subLayout1 = new QCPLayoutGrid;
    QCPAxisRect *AxisRect1 = new QCPAxisRect(ui->widget1);
    AxisRect1->axis(QCPAxis::atLeft, 0)->setTickLabels(true);
    AxisRect1->axis(QCPAxis::atBottom, 0)->setTickLabels(true);
    AxisRect1->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    AxisRect1->setupFullAxesBox(true);
    //AxisRect1->addAxis(QCPAxis::atLeft)->setTickLabelColor(QColor("#6050F8")); // add an extra axis on the left and color its numbers
    AxisRect1->axis(QCPAxis::atRight, 0)->setTickLabels(true);
    QCPLayoutGrid *subLayout = new QCPLayoutGrid;
    subLayout1->addElement(0, 0, AxisRect1);
    ui->widget1->plotLayout()->addElement(0, 0, subLayout1);// sub layout in second row (grid layout will grow accordingly)



    QCPAxisRect *AxisRect2 = new QCPAxisRect(ui->widget1);
    AxisRect2->axis(QCPAxis::atLeft, 0)->setTickLabels(true);
    AxisRect2->axis(QCPAxis::atBottom, 0)->setTickLabels(true);
    AxisRect2->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    AxisRect2->setupFullAxesBox(true);
    AxisRect2->axis(QCPAxis::atRight, 0)->setTickLabels(true);

    ui->widget1->plotLayout()->addElement(1, 0, AxisRect2);// sub layout in second row (grid layout will grow accordingly)


    QCPAxisRect *AxisRect3 = new QCPAxisRect(ui->widget1);
    AxisRect3->axis(QCPAxis::atLeft, 0)->setTickLabels(true);
    AxisRect3->axis(QCPAxis::atBottom, 0)->setTickLabels(true);
    AxisRect3->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    AxisRect3->setupFullAxesBox(true);
    AxisRect3->axis(QCPAxis::atRight, 0)->setTickLabels(true);


    ui->widget1->plotLayout()->addElement(2, 0, AxisRect3);// sub layout in second row (grid layout will grow accordingly)


    foreach (QCPAxisRect *rect, ui->widget1->axisRects())
    {
        foreach (QCPAxis *axis, rect->axes())
        {
            axis->setLayer("axes");
            axis->grid()->setLayer("grid");
        }
    }



    QCPGraph *plot1 = ui->widget1->addGraph(AxisRect1->axis(QCPAxis::atBottom), AxisRect1->axis(QCPAxis::atLeft));
    plot1->keyAxis()->setLabel(" X Trajectory");
    plot1->valueAxis()->setLabel("Y Trajectory");
    plot1->setName("CoM");
    plot1->setData(Pelvis.CoMXVector,Pelvis.CoMYVector);
    // plot1->valueAxis()->setRange(-0.2, 0.2);
    // plot1->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, QPen(Qt::green), QBrush(Qt::green), 1.5));
    plot1->setName("CoM");
    plot1->setLineStyle(QCPGraph::lsLine);
    plot1->setPen(QPen(QColor("#00ff00"), 2.5));
    plot1->rescaleAxes();


    QCPGraph *plot11 = ui->widget1->addGraph(AxisRect1->axis(QCPAxis::atBottom), AxisRect1->axis(QCPAxis::atLeft));
    plot11->setData(Pelvis.DCMXVector,Pelvis.DCMYVector);

    plot11->setLineStyle(QCPGraph::lsLine);
    plot11->rescaleKeyAxis();
    //plot11->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, QPen(Qt::blue), QBrush(Qt::blue), 1.5));
    plot11->setName("DCM");
    plot11->setPen(QPen(QColor("blue"), 3));



    QCPGraph *plot111 = ui->widget1->addGraph(AxisRect1->axis(QCPAxis::atBottom), AxisRect1->axis(QCPAxis::atLeft));
    plot111->setData(Pelvis.EndCoPXVector,Pelvis.EndCoPYVector);
    plot111->valueAxis()->setRange(-0.2, 0.4);
    plot111->setLineStyle(QCPGraph::lsLine);
    plot111->rescaleKeyAxis();
    plot111->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::red), QBrush(Qt::red), 3));
    plot111->setName("CoP");
    // ui->widget1->plotLayout()->addElement(3, 0, new QCPTextElement(ui->widget1, "Way too many graphs in one plot", QFont("sans", 12, QFont::Bold)));
    plot111->setPen(QPen(QColor("magenta"), 2.5));



    QCPLegend *legend1 = new QCPLegend();
    legend1->setLayer("legend1");
    AxisRect1->insetLayout()->addElement(legend1,Qt::AlignTop|Qt::AlignRight);
    legend1->addItem(new QCPPlottableLegendItem(legend1,plot1));
    legend1->addItem(new QCPPlottableLegendItem(legend1,plot11));
    legend1->addItem(new QCPPlottableLegendItem(legend1,plot111));
    legend1->setBrush(QBrush(QColor(255,255,255,150)));
    //legend1->setVisible(true);


    QCPGraph *plot2 = ui->widget1->addGraph(AxisRect2->axis(QCPAxis::atBottom), AxisRect2->axis(QCPAxis::atLeft));
    plot2->setName("CoMX");
    plot2->setData(Pelvis.timeVector,Pelvis.CoMXVector);
    plot2->keyAxis()->setLabel("time(s)");
    plot2->valueAxis()->setLabel("X Trajectory");
    // plot1->valueAxis()->setRange(-0.2, 0.2);
    // plot2->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::green), QBrush(Qt::green), 2));
    plot2->keyAxis()->setLabel("time(s)");
    plot2->valueAxis()->setLabel("X trajectory");
    plot2->setLineStyle(QCPGraph::lsLine);
    //    QPen blueDotPen;
    //    blueDotPen.setColor(QColor(30, 40, 255, 150));
    //    blueDotPen.setStyle(Qt::DotLine);
    plot2->setPen(QPen(QColor("#00ff00"), 2));
    plot2->rescaleAxes();




    QCPGraph *plot22 = ui->widget1->addGraph(AxisRect2->axis(QCPAxis::atBottom), AxisRect2->axis(QCPAxis::atLeft));
    plot22->setData(Pelvis.timeVector,Pelvis.DCMXVector);
    //plot22->valueAxis()->setRange(-0.2, 0.2);
    plot22->setLineStyle(QCPGraph::lsLine);
    plot22->setPen(QPen(QColor("blue"), 2));
    plot22->rescaleKeyAxis();
    plot22->setName("DCMX");
    // ui->widget1->plotLayout()->addElement(3, 0, new QCPTextElement(ui->widget1, "Way too many graphs in one plot", QFont("sans", 12, QFont::Bold)));



    QCPGraph *plot222 = ui->widget1->addGraph(AxisRect2->axis(QCPAxis::atBottom), AxisRect2->axis(QCPAxis::atLeft));
    plot222->setData(Pelvis.timeVector,Pelvis.EndCoPXVector);
    // plot222->valueAxis()->setRange(-0.2, 0.2);
    plot222->setLineStyle(QCPGraph::lsLine);
    plot222->rescaleKeyAxis();
    plot222->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, QPen(Qt::magenta), QBrush(Qt::red), 2));
    plot222->setPen(QPen(QColor("magenta"), 2.5));
    plot222->setName("CoPX");



    QCPLegend *legend2 = new QCPLegend();
    legend2->setLayer("legend1");
    AxisRect2->insetLayout()->addElement(legend2,Qt::AlignBottom|Qt::AlignRight);
    legend2->addItem(new QCPPlottableLegendItem(legend2,plot2));
    legend2->addItem(new QCPPlottableLegendItem(legend2,plot22));
    legend2->addItem(new QCPPlottableLegendItem(legend2,plot222));



    QCPGraph *plot3 = ui->widget1->addGraph(AxisRect3->axis(QCPAxis::atBottom), AxisRect3->axis(QCPAxis::atLeft));
    plot3->setName("CoMY");
    plot3->setData(Pelvis.timeVector,Pelvis.CoMYVector);
    plot3->keyAxis()->setLabel("time(s)");
    plot3->valueAxis()->setLabel("Y trajectory");
    // plot1->valueAxis()->setRange(-0.2, 0.2);
    //plot3->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::green), 2));
    plot3->keyAxis()->setLabel("time(s)");
    plot3->valueAxis()->setLabel("Y Trajectory");
    plot3->setLineStyle(QCPGraph::lsLine);
    plot3->setPen(QPen(QColor("#00ff00"), 2));
    plot3->rescaleAxes();




    QCPGraph *plot33 = ui->widget1->addGraph(AxisRect3->axis(QCPAxis::atBottom), AxisRect3->axis(QCPAxis::atLeft));
    plot33->setData(Pelvis.timeVector,Pelvis.DCMYVector);
    plot33->valueAxis()->setRange(-0.2, 0.5);
    plot33->setLineStyle(QCPGraph::lsLine);
    plot33->rescaleKeyAxis();
    //plot33->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::blue), 1.5));
    plot33->setName("DCMY");
    plot33->setPen(QPen(QColor("blue"), 2));
    // ui->widget1->plotLayout()->addElement(3, 0, new QCPTextElement(ui->widget1, "Way too many graphs in one plot", QFont("sans", 12, QFont::Bold)));


    QCPGraph *plot333 = ui->widget1->addGraph(AxisRect3->axis(QCPAxis::atBottom), AxisRect3->axis(QCPAxis::atLeft));
    plot333->setData(Pelvis.timeVector,Pelvis.EndCoPYVector);
    plot333->setLineStyle(QCPGraph::lsLine);
    plot333->rescaleKeyAxis();
    //plot333->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::magenta), 1.5));
    plot333->setName("CoPY");

    //    plot333->setPen(QPen(QColor(120, 120, 120), 2));
    plot333->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, QPen(Qt::magenta), QBrush(Qt::red), 2));
    plot333->setPen(QPen(QColor("magenta"), 2.5));
    // ui->widget1->plotLayout()->addElement(3, 0, new QCPTextElement(ui->widget1, "Way too many graphs in one plot", QFont("sans", 12, QFont::Bold)));


    QCPLegend *legend3 = new QCPLegend();
    legend3->setLayer("legend3");
    AxisRect3->insetLayout()->addElement(legend3,Qt::AlignTop|Qt::AlignRight);
    legend3->addItem(new QCPPlottableLegendItem(legend3,plot3));
    legend3->addItem(new QCPPlottableLegendItem(legend3,plot33));
    legend3->addItem(new QCPPlottableLegendItem(legend3,plot333));
    ui->widget1->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
}







void ChartForm::PlotFoot(TaskSpace Pelvis){
    // ui->widget1->plotLayout()->clear(); // clear default axis rect so we can start from scratch

    //    ui->widget1->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    //    ui->widget1->legend->setVisible(true);
    //    QFont legendFont = font();  // start out with MainWindow's font..
    //    legendFont.setPointSize(9); // and make a bit smaller for legend
    //    ui->widget1->legend->setFont(legendFont);
    //    ui->widget1->legend->setBrush(QBrush(QColor(255,255,255,230)));
    //    // by default, the legend is in the inset layout of the main axis rect. So this is how we access it to change legend placement:
    //    ui->widget1->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom|Qt::AlignRight);

    //    // setup for graph 0: key axis left, value axis bottom
    //    // will contain left maxwell-like function
    //    ui->widget1->addGraph(ui->widget1->yAxis, ui->widget1->xAxis);
    //    ui->widget1->graph(0)->setPen(QPen(QColor(255, 100, 0)));
    //    //ui->widget1->graph(0)->setBrush(QBrush(QPixmap("./balboa.jpg"))); // fill with texture of specified image
    //    ui->widget1->graph(0)->setLineStyle(QCPGraph::lsLine);
    //    ui->widget1->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 5));
    //    ui->widget1->graph(0)->setName("Left maxwell function");



    ui->widget1->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom));
    //ui->widget->legend->setVisible(true);
    ui->widget1->setAutoAddPlottableToLegend(false);

    ui->widget1->plotLayout()->clear();



    // ui->widget1->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom));
    //    ui->widget1->legend->setVisible(true);
    //    QFont legendFont = font();  // start out with MainWindow's font..
    //    legendFont.setPointSize(9); // and make a bit smaller for legend
    //    ui->widget1->legend->setFont(legendFont);
    //    ui->widget1->legend->setBrush(QBrush(QColor(255,255,255,230)));
    //ui->widget1->legend->setVisible(true);

    //ui->widget1->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom|Qt::AlignRight);
    QCPLayoutGrid *subLayout1 = new QCPLayoutGrid;
    QCPAxisRect *AxisRect1 = new QCPAxisRect(ui->widget1);
    AxisRect1->axis(QCPAxis::atLeft, 0)->setTickLabels(true);
    AxisRect1->axis(QCPAxis::atBottom, 0)->setTickLabels(true);
    AxisRect1->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    AxisRect1->setupFullAxesBox(true);
    //AxisRect1->addAxis(QCPAxis::atLeft)->setTickLabelColor(QColor("#6050F8")); // add an extra axis on the left and color its numbers
    AxisRect1->axis(QCPAxis::atRight, 0)->setTickLabels(true);
    QCPLayoutGrid *subLayout = new QCPLayoutGrid;
    subLayout1->addElement(0, 0, AxisRect1);
    ui->widget1->plotLayout()->addElement(0, 0, subLayout1);// sub layout in second row (grid layout will grow accordingly)



    QCPAxisRect *AxisRect2 = new QCPAxisRect(ui->widget1);
    AxisRect2->axis(QCPAxis::atLeft, 0)->setTickLabels(true);
    AxisRect2->axis(QCPAxis::atBottom, 0)->setTickLabels(true);
    AxisRect2->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    AxisRect2->setupFullAxesBox(true);
    AxisRect2->axis(QCPAxis::atRight, 0)->setTickLabels(true);

    ui->widget1->plotLayout()->addElement(1, 0, AxisRect2);// sub layout in second row (grid layout will grow accordingly)


    QCPAxisRect *AxisRect3 = new QCPAxisRect(ui->widget1);
    AxisRect3->axis(QCPAxis::atLeft, 0)->setTickLabels(true);
    AxisRect3->axis(QCPAxis::atBottom, 0)->setTickLabels(true);
    AxisRect3->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    AxisRect3->setupFullAxesBox(true);
    AxisRect3->axis(QCPAxis::atRight, 0)->setTickLabels(true);


    ui->widget1->plotLayout()->addElement(2, 0, AxisRect3);// sub layout in second row (grid layout will grow accordingly)




    foreach (QCPAxisRect *rect, ui->widget1->axisRects())
    {
        foreach (QCPAxis *axis, rect->axes())
        {
            axis->setLayer("axes");
            axis->grid()->setLayer("grid");
        }
    }



    QCPGraph *plot1 = ui->widget1->addGraph(AxisRect1->axis(QCPAxis::atBottom), AxisRect1->axis(QCPAxis::atLeft));
    plot1->keyAxis()->setLabel(" X Foot Trajectory");
    plot1->valueAxis()->setLabel("Y Foot  Trajectory");
    plot1->setName("Right Foot");
    plot1->setData(Pelvis.RightFootXTrajectory,Pelvis.RightFootYTrajectory);

    // plot1->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, QPen(Qt::green), QBrush(Qt::green), 1.5));
    plot1->setLineStyle(QCPGraph::lsLine);
    plot1->setPen(QPen(QColor("#00ff00"), 2.5));
    plot1->rescaleAxes();


    QCPGraph *plot11 = ui->widget1->addGraph(AxisRect1->axis(QCPAxis::atBottom), AxisRect1->axis(QCPAxis::atLeft));
    plot11->setData(Pelvis.LeftFootXTrajectory,Pelvis.LeftFootYTrajectory);
    //plot11->valueAxis()->setRange(-2, 2);
    plot11->setLineStyle(QCPGraph::lsLine);
    plot11->rescaleKeyAxis();
    plot1->valueAxis()->setRange(-0.2, 0.2);
    //plot11->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, QPen(Qt::blue), QBrush(Qt::blue), 1.5));
    plot11->setName("Left Foot");
    plot11->setPen(QPen(QColor("blue"), 3));



    //    QCPGraph *plot111 = ui->widget1->addGraph(AxisRect1->axis(QCPAxis::atBottom), AxisRect1->axis(QCPAxis::atLeft));
    //    plot111->setData(Pelvis.EndCoPXVector,Pelvis.EndCoPYVector);
    //    plot111->valueAxis()->setRange(-0.2, 0.4);
    //    plot111->setLineStyle(QCPGraph::lsLine);
    //    plot111->rescaleKeyAxis();
    //    plot111->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::red), QBrush(Qt::red), 3));
    //    plot111->setName("CoP");
    //    // ui->widget1->plotLayout()->addElement(3, 0, new QCPTextElement(ui->widget1, "Way too many graphs in one plot", QFont("sans", 12, QFont::Bold)));
    //    plot111->setPen(QPen(QColor("magenta"), 2.5));



    QCPLegend *legend1 = new QCPLegend();
    legend1->setLayer("legend1");
    AxisRect1->insetLayout()->addElement(legend1,Qt::AlignTop|Qt::AlignRight);
    legend1->addItem(new QCPPlottableLegendItem(legend1,plot1));
    legend1->addItem(new QCPPlottableLegendItem(legend1,plot11));
    //legend1->addItem(new QCPPlottableLegendItem(legend1,plot111));
    legend1->setBrush(QBrush(QColor(255,255,255,150)));
    //legend1->setVisible(true);


    QCPGraph *plot2 = ui->widget1->addGraph(AxisRect2->axis(QCPAxis::atBottom), AxisRect2->axis(QCPAxis::atLeft));
    plot2->setName("Right Foot");
    plot2->setData(Pelvis.RightFootXTrajectory,Pelvis.RightFootZTrajectory);
    plot2->keyAxis()->setLabel("X Foot Trajectory");
    plot2->valueAxis()->setLabel("Z Foot Trajectory");
    // plot1->valueAxis()->setRange(-0.2, 0.2);
    // plot2->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::green), QBrush(Qt::green), 2));
    plot2->setLineStyle(QCPGraph::lsLine);
    //    QPen blueDotPen;
    //    blueDotPen.setColor(QColor(30, 40, 255, 150));
    //    blueDotPen.setStyle(Qt::DotLine);
    plot2->setPen(QPen(QColor("#00ff00"), 2));
    plot2->rescaleAxes();




    QCPGraph *plot22 = ui->widget1->addGraph(AxisRect2->axis(QCPAxis::atBottom), AxisRect2->axis(QCPAxis::atLeft));
    plot22->setData(Pelvis.LeftFootXTrajectory,Pelvis.LeftFootZTrajectory);
    plot22->valueAxis()->setRange(-0.2, 0.4);
    plot22->setLineStyle(QCPGraph::lsLine);
    plot22->setPen(QPen(QColor("blue"), 2));
    plot22->rescaleKeyAxis();
    plot22->setName("Left Foot");
    // ui->widget1->plotLayout()->addElement(3, 0, new QCPTextElement(ui->widget1, "Way too many graphs in one plot", QFont("sans", 12, QFont::Bold)));







    QCPLegend *legend2 = new QCPLegend();
    legend2->setLayer("legend1");
    AxisRect2->insetLayout()->addElement(legend2,Qt::AlignBottom|Qt::AlignRight);
    legend2->addItem(new QCPPlottableLegendItem(legend2,plot2));
    legend2->addItem(new QCPPlottableLegendItem(legend2,plot22));
    //legend2->addItem(new QCPPlottableLegendItem(legend2,plot222));



    QCPGraph *plot3 = ui->widget1->addGraph(AxisRect3->axis(QCPAxis::atBottom), AxisRect3->axis(QCPAxis::atLeft));
    plot3->setName("Right Foot");
    plot3->setData(Pelvis.RightFootYTrajectory,Pelvis.RightFootZTrajectory);
    plot3->keyAxis()->setLabel("time(s)");
    plot3->valueAxis()->setLabel("Y trajectory");
    // plot1->valueAxis()->setRange(-0.2, 0.2);
    //plot3->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::green), 2));
    plot3->keyAxis()->setLabel("Y Foot Trajectory");
    plot3->valueAxis()->setLabel("Z Foot Trajectory");
    plot3->setLineStyle(QCPGraph::lsLine);
    plot3->setPen(QPen(QColor("#00ff00"), 2));
    plot3->rescaleAxes();
    //    plot3->keyAxis()->setRange(-1, 1);
    //  plot3->valueAxis()->setRange(-0.5, 0.5);



    QCPGraph *plot33 = ui->widget1->addGraph(AxisRect3->axis(QCPAxis::atBottom), AxisRect3->axis(QCPAxis::atLeft));
    plot33->setData(Pelvis.LeftFootYTrajectory,Pelvis.LeftFootZTrajectory);
    //      plot33->keyAxis()->setRange(-1, 1);
    //    plot33->valueAxis()->setRange(-0.5, 0.5);

    plot33->setLineStyle(QCPGraph::lsLine);
    plot33->rescaleKeyAxis();
    //plot33->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::blue), 1.5));
    plot33->setName("Left Foot");
    plot33->setPen(QPen(QColor("blue"), 2));
    // ui->widget1->plotLayout()->addElement(3, 0, new QCPTextElement(ui->widget1, "Way too many graphs in one plot", QFont("sans", 12, QFont::Bold)));
    plot33->keyAxis()->setRange(-0.15, 0.15);
    plot33->valueAxis()->setRange(0, 0.3);

    //    QCPGraph *plot333 = ui->widget1->addGraph(AxisRect3->axis(QCPAxis::atBottom), AxisRect3->axis(QCPAxis::atLeft));
    //    plot333->setData(Pelvis.timeVector,Pelvis.EndCoPYVector);
    //    plot333->setLineStyle(QCPGraph::lsLine);
    //    plot333->rescaleKeyAxis();
    //    //plot333->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::magenta), 1.5));
    //    plot333->setName("CoPY");

    //    plot333->setPen(QPen(QColor(120, 120, 120), 2));
    //    plot333->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, QPen(Qt::magenta), QBrush(Qt::red), 2));
    //     plot333->setPen(QPen(QColor("magenta"), 2.5));
    //    // ui->widget1->plotLayout()->addElement(3, 0, new QCPTextElement(ui->widget1, "Way too many graphs in one plot", QFont("sans", 12, QFont::Bold)));


    QCPLegend *legend3 = new QCPLegend();
    legend3->setLayer("legend3");
    AxisRect3->insetLayout()->addElement(legend3,Qt::AlignTop|Qt::AlignRight);
    legend3->addItem(new QCPPlottableLegendItem(legend3,plot3));
    legend3->addItem(new QCPPlottableLegendItem(legend3,plot33));
    //legend3->addItem(new QCPPlottableLegendItem(legend3,plot333));
    ui->widget1->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
}






void ChartForm::PlotFootTime(TaskSpace Pelvis){

    ui->widget1->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom));
    //ui->widget->legend->setVisible(true);
    ui->widget1->setAutoAddPlottableToLegend(false);

    ui->widget1->plotLayout()->clear();

    QCPLayoutGrid *subLayout1 = new QCPLayoutGrid;
    QCPAxisRect *AxisRect1 = new QCPAxisRect(ui->widget1);
    AxisRect1->axis(QCPAxis::atLeft, 0)->setTickLabels(true);
    AxisRect1->axis(QCPAxis::atBottom, 0)->setTickLabels(true);
    AxisRect1->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    AxisRect1->setupFullAxesBox(true);
    //AxisRect1->addAxis(QCPAxis::atLeft)->setTickLabelColor(QColor("#6050F8")); // add an extra axis on the left and color its numbers
    AxisRect1->axis(QCPAxis::atRight, 0)->setTickLabels(true);
    QCPLayoutGrid *subLayout = new QCPLayoutGrid;
    subLayout1->addElement(0, 0, AxisRect1);
    ui->widget1->plotLayout()->addElement(0, 0, subLayout1);// sub layout in second row (grid layout will grow accordingly)



    QCPAxisRect *AxisRect2 = new QCPAxisRect(ui->widget1);
    AxisRect2->axis(QCPAxis::atLeft, 0)->setTickLabels(true);
    AxisRect2->axis(QCPAxis::atBottom, 0)->setTickLabels(true);
    AxisRect2->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    AxisRect2->setupFullAxesBox(true);
    AxisRect2->axis(QCPAxis::atRight, 0)->setTickLabels(true);

    ui->widget1->plotLayout()->addElement(1, 0, AxisRect2);// sub layout in second row (grid layout will grow accordingly)


    QCPAxisRect *AxisRect3 = new QCPAxisRect(ui->widget1);
    AxisRect3->axis(QCPAxis::atLeft, 0)->setTickLabels(true);
    AxisRect3->axis(QCPAxis::atBottom, 0)->setTickLabels(true);
    AxisRect3->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    AxisRect3->setupFullAxesBox(true);
    AxisRect3->axis(QCPAxis::atRight, 0)->setTickLabels(true);


    ui->widget1->plotLayout()->addElement(2, 0, AxisRect3);// sub layout in second row (grid layout will grow accordingly)




    foreach (QCPAxisRect *rect, ui->widget1->axisRects())
    {
        foreach (QCPAxis *axis, rect->axes())
        {
            axis->setLayer("axes");
            axis->grid()->setLayer("grid");
        }
    }



    QCPGraph *plot1 = ui->widget1->addGraph(AxisRect1->axis(QCPAxis::atBottom), AxisRect1->axis(QCPAxis::atLeft));
    plot1->keyAxis()->setLabel(" time(s)");
    plot1->valueAxis()->setLabel("X Foot  Trajectory");
    plot1->setName("Right Foot");
    plot1->setData(Pelvis.timeVector,Pelvis.RightFootXTrajectory);

    // plot1->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, QPen(Qt::green), QBrush(Qt::green), 1.5));
    plot1->setLineStyle(QCPGraph::lsLine);
    plot1->setPen(QPen(QColor("#00ff00"), 2.5));
    plot1->rescaleAxes();


    QCPGraph *plot11 = ui->widget1->addGraph(AxisRect1->axis(QCPAxis::atBottom), AxisRect1->axis(QCPAxis::atLeft));
    plot11->setData(Pelvis.timeVector,Pelvis.LeftFootXTrajectory);
    //plot11->valueAxis()->setRange(-2, 2);
    plot11->setLineStyle(QCPGraph::lsLine);
    plot11->rescaleKeyAxis();
    //plot1->valueAxis()->setRange(-0.2, 0.2);
    //plot11->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, QPen(Qt::blue), QBrush(Qt::blue), 1.5));
    plot11->setName("Left Foot");
    plot11->setPen(QPen(QColor("blue"), 3));





    QCPLegend *legend1 = new QCPLegend();
    legend1->setLayer("legend1");
    AxisRect1->insetLayout()->addElement(legend1,Qt::AlignTop|Qt::AlignRight);
    legend1->addItem(new QCPPlottableLegendItem(legend1,plot1));
    legend1->addItem(new QCPPlottableLegendItem(legend1,plot11));
    //legend1->addItem(new QCPPlottableLegendItem(legend1,plot111));
    legend1->setBrush(QBrush(QColor(255,255,255,150)));
    //legend1->setVisible(true);


    QCPGraph *plot2 = ui->widget1->addGraph(AxisRect2->axis(QCPAxis::atBottom), AxisRect2->axis(QCPAxis::atLeft));
    plot2->setName("Right Foot");
    plot2->setData(Pelvis.timeVector,Pelvis.RightFootYTrajectory);
    plot2->keyAxis()->setLabel("time(s)");
    plot2->valueAxis()->setLabel("Y Foot Trajectory");
    // plot1->valueAxis()->setRange(-0.2, 0.2);
    // plot2->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::green), QBrush(Qt::green), 2));
    plot2->setLineStyle(QCPGraph::lsLine);
    //    QPen blueDotPen;
    //    blueDotPen.setColor(QColor(30, 40, 255, 150));
    //    blueDotPen.setStyle(Qt::DotLine);
    plot2->setPen(QPen(QColor("#00ff00"), 2));
    plot2->rescaleAxes();




    QCPGraph *plot22 = ui->widget1->addGraph(AxisRect2->axis(QCPAxis::atBottom), AxisRect2->axis(QCPAxis::atLeft));
    plot22->setData(Pelvis.timeVector,Pelvis.LeftFootYTrajectory);
    plot22->valueAxis()->setRange(-0.2, 0.4);
    //plot22->setLineStyle(QCPGraph::lsLine);
    plot22->setPen(QPen(QColor("blue"), 2));
    plot22->rescaleKeyAxis();
    plot22->setName("Left Foot");
    // ui->widget1->plotLayout()->addElement(3, 0, new QCPTextElement(ui->widget1, "Way too many graphs in one plot", QFont("sans", 12, QFont::Bold)));


    QCPLegend *legend2 = new QCPLegend();
    legend2->setLayer("legend1");
    AxisRect2->insetLayout()->addElement(legend2,Qt::AlignBottom|Qt::AlignRight);
    legend2->addItem(new QCPPlottableLegendItem(legend2,plot2));
    legend2->addItem(new QCPPlottableLegendItem(legend2,plot22));
    //legend2->addItem(new QCPPlottableLegendItem(legend2,plot222));



    QCPGraph *plot3 = ui->widget1->addGraph(AxisRect3->axis(QCPAxis::atBottom), AxisRect3->axis(QCPAxis::atLeft));
    plot3->setName("Right Foot");
    plot3->setData(Pelvis.timeVector,Pelvis.RightFootZTrajectory);
    plot3->keyAxis()->setLabel("time(s)");
    plot3->valueAxis()->setLabel("Z Foot trajectory");
    // plot1->valueAxis()->setRange(-0.2, 0.2);
    //plot3->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::green), 2));
    plot3->setLineStyle(QCPGraph::lsLine);
    plot3->setPen(QPen(QColor("#00ff00"), 2));
    plot3->rescaleAxes();
    //    plot3->keyAxis()->setRange(-1, 1);
    //  plot3->valueAxis()->setRange(-0.5, 0.5);



    QCPGraph *plot33 = ui->widget1->addGraph(AxisRect3->axis(QCPAxis::atBottom), AxisRect3->axis(QCPAxis::atLeft));
    plot33->setData(Pelvis.timeVector,Pelvis.LeftFootZTrajectory);
    //      plot33->keyAxis()->setRange(-1, 1);
    //    plot33->valueAxis()->setRange(-0.5, 0.5);

    plot33->setLineStyle(QCPGraph::lsLine);
    plot33->rescaleKeyAxis();
    //plot33->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::blue), 1.5));
    plot33->setName("Left Foot");
    plot33->setPen(QPen(QColor("blue"), 2));
    // ui->widget1->plotLayout()->addElement(3, 0, new QCPTextElement(ui->widget1, "Way too many graphs in one plot", QFont("sans", 12, QFont::Bold)));
    //plot33->keyAxis()->setRange(-0.15, 0.15);
    // plot33->valueAxis()->setRange(0, 0.3);

    QCPLegend *legend3 = new QCPLegend();
    legend3->setLayer("legend3");
    AxisRect3->insetLayout()->addElement(legend3,Qt::AlignTop|Qt::AlignRight);
    legend3->addItem(new QCPPlottableLegendItem(legend3,plot3));
    legend3->addItem(new QCPPlottableLegendItem(legend3,plot33));
    //legend3->addItem(new QCPPlottableLegendItem(legend3,plot333));
    ui->widget1->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
}



//-----------------------------------------------------------------------------------------------------

void ChartForm::PlotFootVel(TaskSpace Pelvis){

    ui->widget1->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom));
    //ui->widget->legend->setVisible(true);
    ui->widget1->setAutoAddPlottableToLegend(false);

    ui->widget1->plotLayout()->clear();

    QCPLayoutGrid *subLayout1 = new QCPLayoutGrid;
    QCPAxisRect *AxisRect1 = new QCPAxisRect(ui->widget1);
    AxisRect1->axis(QCPAxis::atLeft, 0)->setTickLabels(true);
    AxisRect1->axis(QCPAxis::atBottom, 0)->setTickLabels(true);
    AxisRect1->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    AxisRect1->setupFullAxesBox(true);
    //AxisRect1->addAxis(QCPAxis::atLeft)->setTickLabelColor(QColor("#6050F8")); // add an extra axis on the left and color its numbers
    AxisRect1->axis(QCPAxis::atRight, 0)->setTickLabels(true);
    QCPLayoutGrid *subLayout = new QCPLayoutGrid;
    subLayout1->addElement(0, 0, AxisRect1);
    ui->widget1->plotLayout()->addElement(0, 0, subLayout1);// sub layout in second row (grid layout will grow accordingly)



    QCPAxisRect *AxisRect2 = new QCPAxisRect(ui->widget1);
    AxisRect2->axis(QCPAxis::atLeft, 0)->setTickLabels(true);
    AxisRect2->axis(QCPAxis::atBottom, 0)->setTickLabels(true);
    AxisRect2->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    AxisRect2->setupFullAxesBox(true);
    AxisRect2->axis(QCPAxis::atRight, 0)->setTickLabels(true);

    ui->widget1->plotLayout()->addElement(1, 0, AxisRect2);// sub layout in second row (grid layout will grow accordingly)


    QCPAxisRect *AxisRect3 = new QCPAxisRect(ui->widget1);
    AxisRect3->axis(QCPAxis::atLeft, 0)->setTickLabels(true);
    AxisRect3->axis(QCPAxis::atBottom, 0)->setTickLabels(true);
    AxisRect3->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    AxisRect3->setupFullAxesBox(true);
    AxisRect3->axis(QCPAxis::atRight, 0)->setTickLabels(true);


    ui->widget1->plotLayout()->addElement(2, 0, AxisRect3);// sub layout in second row (grid layout will grow accordingly)




    foreach (QCPAxisRect *rect, ui->widget1->axisRects())
    {
        foreach (QCPAxis *axis, rect->axes())
        {
            axis->setLayer("axes");
            axis->grid()->setLayer("grid");
        }
    }



    QCPGraph *plot1 = ui->widget1->addGraph(AxisRect1->axis(QCPAxis::atBottom), AxisRect1->axis(QCPAxis::atLeft));
    plot1->keyAxis()->setLabel(" time(s)");
    plot1->valueAxis()->setLabel("X Foot  Velocity");
    plot1->setName("Right Foot");
    plot1->setData(Pelvis.timeVector,Pelvis.RightFootXVelocity);

    // plot1->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, QPen(Qt::green), QBrush(Qt::green), 1.5));
    plot1->setLineStyle(QCPGraph::lsLine);
    plot1->setPen(QPen(QColor("#00ff00"), 2.5));
    plot1->rescaleAxes();


    QCPGraph *plot11 = ui->widget1->addGraph(AxisRect1->axis(QCPAxis::atBottom), AxisRect1->axis(QCPAxis::atLeft));
    plot11->setData(Pelvis.timeVector,Pelvis.LeftFootXVelocity);
    //plot11->valueAxis()->setRange(-2, 2);
    plot11->setLineStyle(QCPGraph::lsLine);
    plot11->rescaleKeyAxis();
    //plot1->valueAxis()->setRange(-0.2, 0.2);
    //plot11->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, QPen(Qt::blue), QBrush(Qt::blue), 1.5));
    plot11->setName("Left Foot");
    plot11->setPen(QPen(QColor("blue"), 3));





    QCPLegend *legend1 = new QCPLegend();
    legend1->setLayer("legend1");
    AxisRect1->insetLayout()->addElement(legend1,Qt::AlignTop|Qt::AlignRight);
    legend1->addItem(new QCPPlottableLegendItem(legend1,plot1));
    legend1->addItem(new QCPPlottableLegendItem(legend1,plot11));
    //legend1->addItem(new QCPPlottableLegendItem(legend1,plot111));
    legend1->setBrush(QBrush(QColor(255,255,255,150)));
    //legend1->setVisible(true);


    QCPGraph *plot2 = ui->widget1->addGraph(AxisRect2->axis(QCPAxis::atBottom), AxisRect2->axis(QCPAxis::atLeft));
    plot2->setName("Right Foot");
    plot2->setData(Pelvis.timeVector,Pelvis.RightFootYVelocity);
    plot2->keyAxis()->setLabel("time(s)");
    plot2->valueAxis()->setLabel("Y Foot Velocity");
    // plot1->valueAxis()->setRange(-0.2, 0.2);
    // plot2->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::green), QBrush(Qt::green), 2));
    plot2->setLineStyle(QCPGraph::lsLine);
    //    QPen blueDotPen;
    //    blueDotPen.setColor(QColor(30, 40, 255, 150));
    //    blueDotPen.setStyle(Qt::DotLine);
    plot2->setPen(QPen(QColor("#00ff00"), 2));
    plot2->rescaleAxes();




    QCPGraph *plot22 = ui->widget1->addGraph(AxisRect2->axis(QCPAxis::atBottom), AxisRect2->axis(QCPAxis::atLeft));
    plot22->setData(Pelvis.timeVector,Pelvis.LeftFootYVelocity);
    //plot22->valueAxis()->setRange(-0.2, 0.4);
    //plot22->setLineStyle(QCPGraph::lsLine);
    plot22->setPen(QPen(QColor("blue"), 2));
    plot22->rescaleKeyAxis();
    plot22->setName("Left Foot");
    // ui->widget1->plotLayout()->addElement(3, 0, new QCPTextElement(ui->widget1, "Way too many graphs in one plot", QFont("sans", 12, QFont::Bold)));


    QCPLegend *legend2 = new QCPLegend();
    legend2->setLayer("legend1");
    AxisRect2->insetLayout()->addElement(legend2,Qt::AlignBottom|Qt::AlignRight);
    legend2->addItem(new QCPPlottableLegendItem(legend2,plot2));
    legend2->addItem(new QCPPlottableLegendItem(legend2,plot22));
    //legend2->addItem(new QCPPlottableLegendItem(legend2,plot222));



    QCPGraph *plot3 = ui->widget1->addGraph(AxisRect3->axis(QCPAxis::atBottom), AxisRect3->axis(QCPAxis::atLeft));
    plot3->setName("Right Foot");
    plot3->setData(Pelvis.timeVector,Pelvis.RightFootZVelocity);
    plot3->keyAxis()->setLabel("time(s)");
    plot3->valueAxis()->setLabel("Z Foot Velocity");
    // plot1->valueAxis()->setRange(-0.2, 0.2);
    //plot3->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::green), 2));
    plot3->setLineStyle(QCPGraph::lsLine);
    plot3->setPen(QPen(QColor("#00ff00"), 2));
    plot3->rescaleAxes();
    //    plot3->keyAxis()->setRange(-1, 1);
    //  plot3->valueAxis()->setRange(-0.5, 0.5);



    QCPGraph *plot33 = ui->widget1->addGraph(AxisRect3->axis(QCPAxis::atBottom), AxisRect3->axis(QCPAxis::atLeft));
    plot33->setData(Pelvis.timeVector,Pelvis.LeftFootZVelocity);
    //      plot33->keyAxis()->setRange(-1, 1);
    //    plot33->valueAxis()->setRange(-0.5, 0.5);

    plot33->setLineStyle(QCPGraph::lsLine);
    plot33->rescaleKeyAxis();
    //plot33->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::blue), 1.5));
    plot33->setName("Left Foot");
    plot33->setPen(QPen(QColor("blue"), 2));
    // ui->widget1->plotLayout()->addElement(3, 0, new QCPTextElement(ui->widget1, "Way too many graphs in one plot", QFont("sans", 12, QFont::Bold)));
    //plot33->keyAxis()->setRange(-0.15, 0.15);
    // plot33->valueAxis()->setRange(0, 0.3);

    QCPLegend *legend3 = new QCPLegend();
    legend3->setLayer("legend3");
    AxisRect3->insetLayout()->addElement(legend3,Qt::AlignTop|Qt::AlignRight);
    legend3->addItem(new QCPPlottableLegendItem(legend3,plot3));
    legend3->addItem(new QCPPlottableLegendItem(legend3,plot33));
    //legend3->addItem(new QCPPlottableLegendItem(legend3,plot333));
    ui->widget1->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
}






void ChartForm::PlotFootAccel(TaskSpace Pelvis){

    ui->widget1->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom));
    //ui->widget->legend->setVisible(true);
    ui->widget1->setAutoAddPlottableToLegend(false);

    ui->widget1->plotLayout()->clear();

    QCPLayoutGrid *subLayout1 = new QCPLayoutGrid;
    QCPAxisRect *AxisRect1 = new QCPAxisRect(ui->widget1);
    AxisRect1->axis(QCPAxis::atLeft, 0)->setTickLabels(true);
    AxisRect1->axis(QCPAxis::atBottom, 0)->setTickLabels(true);
    AxisRect1->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    AxisRect1->setupFullAxesBox(true);
    //AxisRect1->addAxis(QCPAxis::atLeft)->setTickLabelColor(QColor("#6050F8")); // add an extra axis on the left and color its numbers
    AxisRect1->axis(QCPAxis::atRight, 0)->setTickLabels(true);
    QCPLayoutGrid *subLayout = new QCPLayoutGrid;
    subLayout1->addElement(0, 0, AxisRect1);
    ui->widget1->plotLayout()->addElement(0, 0, subLayout1);// sub layout in second row (grid layout will grow accordingly)



    QCPAxisRect *AxisRect2 = new QCPAxisRect(ui->widget1);
    AxisRect2->axis(QCPAxis::atLeft, 0)->setTickLabels(true);
    AxisRect2->axis(QCPAxis::atBottom, 0)->setTickLabels(true);
    AxisRect2->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    AxisRect2->setupFullAxesBox(true);
    AxisRect2->axis(QCPAxis::atRight, 0)->setTickLabels(true);

    ui->widget1->plotLayout()->addElement(1, 0, AxisRect2);// sub layout in second row (grid layout will grow accordingly)


    QCPAxisRect *AxisRect3 = new QCPAxisRect(ui->widget1);
    AxisRect3->axis(QCPAxis::atLeft, 0)->setTickLabels(true);
    AxisRect3->axis(QCPAxis::atBottom, 0)->setTickLabels(true);
    AxisRect3->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    AxisRect3->setupFullAxesBox(true);
    AxisRect3->axis(QCPAxis::atRight, 0)->setTickLabels(true);


    ui->widget1->plotLayout()->addElement(2, 0, AxisRect3);// sub layout in second row (grid layout will grow accordingly)




    foreach (QCPAxisRect *rect, ui->widget1->axisRects())
    {
        foreach (QCPAxis *axis, rect->axes())
        {
            axis->setLayer("axes");
            axis->grid()->setLayer("grid");
        }
    }



    QCPGraph *plot1 = ui->widget1->addGraph(AxisRect1->axis(QCPAxis::atBottom), AxisRect1->axis(QCPAxis::atLeft));
    plot1->keyAxis()->setLabel(" time(s)");
    plot1->valueAxis()->setLabel("X Foot  Acceleration");
    plot1->setName("Right Foot");
    plot1->setData(Pelvis.timeVector,Pelvis.RightFootXAcceleration);

    // plot1->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, QPen(Qt::green), QBrush(Qt::green), 1.5));
    plot1->setLineStyle(QCPGraph::lsLine);
    plot1->setPen(QPen(QColor("#00ff00"), 2.5));
    plot1->rescaleAxes();


    QCPGraph *plot11 = ui->widget1->addGraph(AxisRect1->axis(QCPAxis::atBottom), AxisRect1->axis(QCPAxis::atLeft));
    plot11->setData(Pelvis.timeVector,Pelvis.LeftFootXAcceleration);
    //plot11->valueAxis()->setRange(-2, 2);
    plot11->setLineStyle(QCPGraph::lsLine);
    plot11->rescaleKeyAxis();
    //plot1->valueAxis()->setRange(-0.2, 0.2);
    //plot11->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, QPen(Qt::blue), QBrush(Qt::blue), 1.5));
    plot11->setName("Left Foot");
    plot11->setPen(QPen(QColor("blue"), 3));





    QCPLegend *legend1 = new QCPLegend();
    legend1->setLayer("legend1");
    AxisRect1->insetLayout()->addElement(legend1,Qt::AlignTop|Qt::AlignRight);
    legend1->addItem(new QCPPlottableLegendItem(legend1,plot1));
    legend1->addItem(new QCPPlottableLegendItem(legend1,plot11));
    //legend1->addItem(new QCPPlottableLegendItem(legend1,plot111));
    legend1->setBrush(QBrush(QColor(255,255,255,150)));
    //legend1->setVisible(true);


    QCPGraph *plot2 = ui->widget1->addGraph(AxisRect2->axis(QCPAxis::atBottom), AxisRect2->axis(QCPAxis::atLeft));
    plot2->setName("Right Foot");
    plot2->setData(Pelvis.timeVector,Pelvis.RightFootYAcceleration);
    plot2->keyAxis()->setLabel("time(s)");
    plot2->valueAxis()->setLabel("Y Foot Acceleration");
    // plot1->valueAxis()->setRange(-0.2, 0.2);
    // plot2->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::green), QBrush(Qt::green), 2));
    plot2->setLineStyle(QCPGraph::lsLine);
    //    QPen blueDotPen;
    //    blueDotPen.setColor(QColor(30, 40, 255, 150));
    //    blueDotPen.setStyle(Qt::DotLine);
    plot2->setPen(QPen(QColor("#00ff00"), 2));
    plot2->rescaleAxes();




    QCPGraph *plot22 = ui->widget1->addGraph(AxisRect2->axis(QCPAxis::atBottom), AxisRect2->axis(QCPAxis::atLeft));
    plot22->setData(Pelvis.timeVector,Pelvis.LeftFootYAcceleration);
    plot22->valueAxis()->setRange(-0.2, 0.4);
    //plot22->setLineStyle(QCPGraph::lsLine);
    plot22->setPen(QPen(QColor("blue"), 2));
    plot22->rescaleKeyAxis();
    plot22->setName("Left Foot");
    // ui->widget1->plotLayout()->addElement(3, 0, new QCPTextElement(ui->widget1, "Way too many graphs in one plot", QFont("sans", 12, QFont::Bold)));


    QCPLegend *legend2 = new QCPLegend();
    legend2->setLayer("legend1");
    AxisRect2->insetLayout()->addElement(legend2,Qt::AlignBottom|Qt::AlignRight);
    legend2->addItem(new QCPPlottableLegendItem(legend2,plot2));
    legend2->addItem(new QCPPlottableLegendItem(legend2,plot22));
    //legend2->addItem(new QCPPlottableLegendItem(legend2,plot222));



    QCPGraph *plot3 = ui->widget1->addGraph(AxisRect3->axis(QCPAxis::atBottom), AxisRect3->axis(QCPAxis::atLeft));
    plot3->setName("Right Foot");
    plot3->setData(Pelvis.timeVector,Pelvis.RightFootZacceleration);
    plot3->keyAxis()->setLabel("time(s)");
    plot3->valueAxis()->setLabel("Z Foot Acceleration");
    // plot1->valueAxis()->setRange(-0.2, 0.2);
    //plot3->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::green), 2));
    plot3->setLineStyle(QCPGraph::lsLine);
    plot3->setPen(QPen(QColor("#00ff00"), 2));
    plot3->rescaleAxes();
    //    plot3->keyAxis()->setRange(-1, 1);
    //  plot3->valueAxis()->setRange(-0.5, 0.5);



    QCPGraph *plot33 = ui->widget1->addGraph(AxisRect3->axis(QCPAxis::atBottom), AxisRect3->axis(QCPAxis::atLeft));
    plot33->setData(Pelvis.timeVector,Pelvis.LeftFootZAcceleration);
    //      plot33->keyAxis()->setRange(-1, 1);
    //    plot33->valueAxis()->setRange(-0.5, 0.5);

    plot33->setLineStyle(QCPGraph::lsLine);
    plot33->rescaleKeyAxis();
    //plot33->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCrossSquare, QPen(Qt::blue), 1.5));
    plot33->setName("Left Foot");
    plot33->setPen(QPen(QColor("blue"), 2));
    // ui->widget1->plotLayout()->addElement(3, 0, new QCPTextElement(ui->widget1, "Way too many graphs in one plot", QFont("sans", 12, QFont::Bold)));
    //plot33->keyAxis()->setRange(-0.15, 0.15);
    // plot33->valueAxis()->setRange(0, 0.3);

    QCPLegend *legend3 = new QCPLegend();
    legend3->setLayer("legend3");
    AxisRect3->insetLayout()->addElement(legend3,Qt::AlignTop|Qt::AlignRight);
    legend3->addItem(new QCPPlottableLegendItem(legend3,plot3));
    legend3->addItem(new QCPPlottableLegendItem(legend3,plot33));
    //legend3->addItem(new QCPPlottableLegendItem(legend3,plot333));
    ui->widget1->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
}
