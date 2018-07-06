#include "sqdtaskspace.h"

SQDTaskSpace::SQDTaskSpace(QList<double> WalkParameter, QList<Link> Links, QHash <QString,int> MapingName2ID, QHash <int,QString> MapingID2Name)
{
    _RightStart=false ;
    _LeftStart=true;
    _Xoffset=0.05;
    _SpatialStepSize=0.01;
    _nSteps=7;
    //_stepLength=stepLength;
    _ankleLength=0.05;
    _tempTigh=Links[MapingName2ID.value("RLeg_KneeP_J4")].GetJointPositionRelative2Parent();
    _thighLength=_tempTigh(2);
    _tempShank=Links[MapingName2ID.value("RLeg_AnkleP_J5")].GetJointPositionRelative2Parent();
    _shankLength=_tempShank(2);
    _XMiddleOfStep=_stepLength/2;
    _legLength=_shankLength+_thighLength;

    MatrixXd _xHip;
    MatrixXd _zHip;
    //X=0;
}
