#ifndef LINK_H
#define LINK_H
#include "Eigen/Dense"
using namespace Eigen;
#include <QObject>

class Link
{


public:

    QString GetName();
    void SetName(QString name);
    int GetID();
    void SetID(int ID);
    int GetSisterID();
    void SetSisterID(int SisterID);
    int GetMotherID();
    void SetMother(int MotherID);
    int GetChildID();
    void SetChildID(int childID);
    double GetMass();
    void SetMass(double Mass);
    MatrixXd GetMomentumOfInertia();
    void SetMomentumOfInertia(MatrixXd momentumOfInertia);
    MatrixXd GetJointAxisVectorLocal();
    void SetJointAxisVectorLocal(MatrixXd jointAxisVectorLocal);
    MatrixXd GetJointPositionRelative2Parent();
    void SetJointPositionRelative2Parent(MatrixXd jointPositionRelative2Parent);
    MatrixXd GetCenterOfMassLocal();
    void SetCenterOfMassLocal(MatrixXd centerOfMassLocal);

    Link(QString name, int IDofLink, int IDofSyster, int IDofChild, int IDofMother, double AngleOfJoint, MatrixXd LocalAxisVectorOfJoint, MatrixXd PositionRelative2ParentOfJoint);

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
