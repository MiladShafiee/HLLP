#include "robot.h"

Robot::Robot(QObject *parent) : QObject(parent)
{
    QString address="..//HLLP//RobotData//test.txt";
    if (!FileExists(address))
    {
        qWarning()<<"Invalid Robot data Path:"<<address;
        return;
    }
    QFile file(address);
    file.open(QFile::ReadWrite);
    QByteArray content = file.readAll();

    QList<QByteArray> values= GetContentOfRobot("AngularVelocityInWorldCoordinate",content);

    foreach (QByteArray v, values) {
        qDebug()<<"input method="<<v;
        MatrixXd mat= ExtractionOfMatrix(v);
        //qDebug()<<Matrix2;
        MatrixXd gg=Rodrigues(mat,20);
        //qDebug()<<gg;
    }

    file.close();

}

QList <QByteArray> Robot::GetContentOfRobot(QString name,QByteArray content)
{
    QList <QByteArray> result;
    int index=0;
    while (index<content.length()) {

        index=content.indexOf(name,index);
        if (index<0)
        {
            break;
        }

        QByteArray temp=content.mid(index,content.length());
        QByteArray line=temp.split('\n')[0];
        QList <QByteArray> values=line.split('=');
        if (values.length()>1){
            result.append(values[1]);
        }
        else
        {
            qDebug()<<"warning:expected '=' after value"<<name;
        }



        index++;
    }
    return result;

}


bool Robot::FileExists(QString path) {
    QFileInfo check_file(path);
    // check if file exists and if yes: Is it really a file and no directory?
    if (check_file.exists() && check_file.isFile()) {
        return true;
    } else {
        return false;
    }
}

MatrixXd Robot::ExtractionOfMatrix(QByteArray data)
{
    MatrixXd mat;
    QByteArray insideBrackets=data.split(']')[0];
    insideBrackets= insideBrackets.split('[')[1];
    QList <QByteArray> rows=insideBrackets.split(';');
    QList <QByteArray> columns=rows[0].split(',');

    mat.resize(rows.length(),columns.length());

    //////initial mat values
    for (int i = 0; i < rows.length(); i++) {
        QList <QByteArray> currentCols=rows[i].split(',');
        for (int j = 0; j < currentCols.length(); j++) {
            mat(i,j) = currentCols[j].toDouble();

        }
    }


    //    qDebug()<<insideBrackets<<"bale";

    //mat(0,0) = 0;

    return mat;
}

MatrixXd Robot::Rodrigues(MatrixXd omega,double angle)
{
    MatrixXd rotation;
    double normOfOmega=omega.norm();

    if (!normOfOmega<std::numeric_limits<double>::epsilon()) {

        MatrixXd rotation= MatrixXd::Identity(3,3);
        //      for (int var = 0; var < rotation.rows(); ++var) {
        //          for (int var2 = 0; var2 < rotation.cols(); ++var2) {
        //              qDebug()<<rotation(var,var2);
        //          }
        //      }

    } else {
        MatrixXd NormalizedAxisRotation=omega/normOfOmega;
        double AmountOfRotation=normOfOmega*angle;
        MatrixXd SkewSymMatrixOfVector;
        SkewSymMatrixOfVector<<0, -NormalizedAxisRotation(3), NormalizedAxisRotation(2),
                NormalizedAxisRotation(3), 0, -NormalizedAxisRotation(1)
                -NormalizedAxisRotation(2),  NormalizedAxisRotation(1), 0;
        rotation=MatrixXd::Identity(3,3)+SkewSymMatrixOfVector*qSin(AmountOfRotation)+(SkewSymMatrixOfVector*SkewSymMatrixOfVector)*(1-qCos(AmountOfRotation));
    }
       return rotation;
}
