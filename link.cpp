#include "link.h"

QString Link::GetName()
{
    return _name;
}

void Link::SetName(QString name)
{
    _name=name;

}

int Link::GetID()
{
    return _ID;
}

void Link::SetID(int ID)
{
    _ID=ID;
}

int Link::GetSisterID()
{
    return _sisterID;
}

void Link::SetSisterID(int SisterID)
{
    _sisterID=SisterID;
}

int Link::GetMotherID()
{
    return _motherID;
}

void Link::SetMother(int MotherID)
{
    _motherID=MotherID;
}

int Link::GetChildID()
{
    return _childID;
}

void Link::SetChildID(int childID)
{
    _childID=childID;
}

double Link::GetMass()
{
    return _mass;
}

void Link::SetMass(double Mass)
{
    _mass=Mass;
}

MatrixXd Link::GetMomentumOfInertia()
{
    return _momentumOfInertia;
}

void Link::SetMomentumOfInertia(MatrixXd momentumOfInertia)
{
    _momentumOfInertia=momentumOfInertia;
}

MatrixXd Link::GetJointAxisVectorLocal()
{
    return _jointAxisVectorLocal;
}

void Link::SetJointAxisVectorLocal(MatrixXd jointAxisVectorLocal)
{
    _jointAxisVectorLocal=jointAxisVectorLocal;
}

MatrixXd Link::GetJointPositionRelative2Parent()
{
    return _jointPositionRelative2Parent;
}

void Link::SetJointPositionRelative2Parent(MatrixXd jointPositionRelative2Parent)
{
    _jointPositionRelative2Parent=jointPositionRelative2Parent;
}

MatrixXd Link::GetCenterOfMassLocal()
{
    return _centerOfMassLocal;
}

void Link::SetCenterOfMassLocal(MatrixXd centerOfMassLocal)
{
    _centerOfMassLocal=centerOfMassLocal;
}





Link::Link(QString name, int IDofLink, int IDofSyster, int IDofChild, int IDofMother, double AngleOfJoint, MatrixXd LocalAxisVectorOfJoint, MatrixXd PositionRelative2ParentOfJoint)
{
     JointAngle= AngleOfJoint;
    _name=name;
    _ID=IDofLink;
    _sisterID=IDofSyster;
    _childID=IDofChild;
    _motherID=IDofMother;
    _jointAxisVectorLocal=LocalAxisVectorOfJoint ;
    _jointPositionRelative2Parent=PositionRelative2ParentOfJoint;
}


