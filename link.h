#ifndef LINK_H
#define LINK_H
#include "Eigen/Dense"
using namespace Eigen;
#include <QObject>

class Link : public QObject
{
    Q_OBJECT

public:
    explicit Link(QObject *parent = nullptr);
    QString GetName();
    void SetName(QString name);
    Link(QString name, int IDofLink,int IDofSyster,int IDofChild, int IDofMother,double AngleOfJoint,MatrixXd LocalAxisVectorOfJoint, MatrixXd PositionRelative2ParentOfJoint);

    double JointAngle;
    double JointVelocity;
    double JointAcceleration;
    MatrixXd PositionInWorldCoordinate;
    MatrixXd AttitudeInWorldCoordinate;
    MatrixXd LinearVelocityInWorldCoordinate;
    MatrixXd AngularVelocityInWorldCoordinate;


signals:

public slots:


private:
    QString _name;
    int _ID ;
    int _sisterID;
    int _childID;
    int _motherID;
    double _mass;
    MatrixXd _momentumOfInertia;
    MatrixXd _jointAxisVectorLocal;
    MatrixXd _jointPositionRelative2Parent;
    MatrixXd _centerOfMassLocal;
};

#endif // LINK_H
