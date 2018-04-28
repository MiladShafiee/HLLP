#ifndef ROBOT_H
#define ROBOT_H
#include "Eigen/Dense"
#include <QObject>
#include <qfile.h>
#include <qfileinfo.h>
#include <qdebug.h>
#include <qmath.h>
#include  <link.h>

using namespace Eigen;
class Robot : public QObject
{
    Q_OBJECT
    bool FileExists(QString path);
    QList<QByteArray> GetContentOfRobot(QString name, QByteArray content);
    MatrixXd ExtractionOfMatrix(QByteArray data);
    MatrixXd Rodrigues(MatrixXd omega, double angle);
    QList<Link> CreateRobotLinks(QByteArray content);
    void ForwardKinematic(int input,QByteArray content);

public:
    explicit Robot(QObject *parent = nullptr);
    QList<Link> Links;
    QList<Link> GetLinks();
    QHash <QString,int> MappingName2ID;
signals:

public slots:
};

#endif // ROBOT_H
