#ifndef SQDTASKSPACE_H
#define SQDTASKSPACE_H

#include "Eigen/Dense"
#include <qdebug.h>
#include <qmath.h>
#include <Eigen/Geometry>
#include <iostream>
#include <cstdlib>
#include <link.h>
using namespace std;
using namespace Eigen;

class SQDTaskSpace
{
    bool _RightStart;
    bool _LeftStart;
    double _Xoffset;
    double _SpatialStepSize;
    int _nSteps;
    double _stepLength;
    double _ankleLength;
    double _thighLength;
    double _shankLength;
    double _XMiddleOfStep;
    double _legLength;
    MatrixXd _xHip;
    MatrixXd _zHip;
    double _X;
    MatrixXd _tempTigh;
    MatrixXd _tempShank;
public:
    SQDTaskSpace(QList<double> WalkParameter, QList<Link> Links, QHash<QString, int> MapingName2ID, QHash<int, QString> MapingID2Name);
};

#endif // SQDTASKSPACE_H
