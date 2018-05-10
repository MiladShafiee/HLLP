#ifndef ROBOT_H
#define ROBOT_H
#include "Eigen/Dense"
#include <QObject>
#include <qfile.h>
#include <qfileinfo.h>
#include <qdebug.h>
#include <qmath.h>
#include  <link.h>
# include <taskspace.h>>
#include <Eigen/Geometry>
#include <iostream>
#include <cstdlib>
using namespace std;
using namespace Eigen;
class Robot : public QObject
{
    Q_OBJECT
    bool FileExists(QString path);
    MatrixXd _leftLegRoute;
    MatrixXd _rightLegRoute;
    QList<QByteArray> GetContentOfRobot(QString name, QByteArray content);
    MatrixXd ExtractionOfMatrix(QByteArray data);
    MatrixXd Rodrigues(MatrixXd omega, double angle);
    QList<Link> CreateRobotLinks(QByteArray content);
    void ForwardKinematic(int input);
    MatrixXi _route;
    void SetJointAngle(MatrixXd JointAngle, MatrixXi Route);
    MatrixXi FindRoutfromRoot2Destination(QString Destination);
    void MoveJoints(MatrixXi route, MatrixXd deltaJointAngle);
    Vector3d Rot2omega(Matrix3d rotation);
    MatrixXd CalcTaskSpaceError(Link Target, QString current);
    MatrixXd CalcJacobian(MatrixXi route);
    double IKLevenbergMarquardt(Link Target, QString current);
public:
    explicit Robot(QObject *parent = nullptr);
    QList<Link> Links;
    TaskSpace StraightLEgWalk;
    QList<Link> GetLinks();
    MatrixXi Getroute();
    QHash <QString,int> MapingName2ID;
    QHash <int,QString> MapingID2Name;
    MatrixXi GetLeftLegRoute();
    void SetLeftLegRoute(MatrixXd leftLegRoute );
    MatrixXi GetRightLegRoute();
    void SetRightLegRoute(MatrixXd rightLegRoute);

 QByteArray content;
signals:

public slots:
};

#endif // ROBOT_H
