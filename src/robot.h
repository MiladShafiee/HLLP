#ifndef ROBOT_H
#define ROBOT_H
#include "Eigen/Dense"
#include <qfile.h>
#include <qfileinfo.h>
#include <qdebug.h>
#include <qmath.h>
#include  <link.h>
//# include <taskspace.h>>
#include <Eigen/Geometry>
#include <iostream>
#include <cstdlib>
using namespace std;
using namespace Eigen;
class Robot
{

    bool FileExists(QString path);
    QList<QByteArray> GetContentOfRobot(QString name, QByteArray content);
    MatrixXd ExtractionOfMatrix(QByteArray data);
    MatrixXd Rodrigues(MatrixXd omega, double angle);
    QList<Link> CreateRobotLinks(QByteArray content);
    void ForwardKinematic(int input);
    void SetJointAngle(MatrixXd JointAngle, MatrixXi Route);
    MatrixXi FindRoutfromRoot2Destination(QString Destination);
    void MoveJoints(MatrixXi route, MatrixXd deltaJointAngle);
    Vector3d Rot2omega(Matrix3d rotation);
    MatrixXd CalcTaskSpaceError(Link Target, QString current);
    MatrixXd CalcJacobian(MatrixXi route);
    double IKLevenbergMarquardt(Link Target, QString current);

    MatrixXd _joinAngles;
    MatrixXi _route;
    MatrixXd _leftLegRoute;
    MatrixXd _rightLegRoute;
public:
    explicit Robot(MatrixXd PelvisTrajectory,MatrixXd AnkleTrajectory);
    //TaskSpace StraightLEgWalk;
    QList<Link> GetLinks();
    MatrixXi Getroute();
    MatrixXi GetLeftLegRoute();
    void SetLeftLegRoute(MatrixXd leftLegRoute );
    MatrixXi GetRightLegRoute();
    void SetRightLegRoute(MatrixXd rightLegRoute);

    QList<Link> Links;
    QByteArray content;
    QHash <QString,int> MapingName2ID;
    QHash <int,QString> MapingID2Name;
signals:

public slots:
};

#endif // ROBOT_H
