#include "link.h"

Link::Link(QObject *parent) : QObject(parent)
{

}

QString Link::GetName()
{
    return _name;
}

void Link::SetName(QString name)
{
    _name=name;

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


