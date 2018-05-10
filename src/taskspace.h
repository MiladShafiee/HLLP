#ifndef TASKSPACE_H
#define TASKSPACE_H
#include <qdebug.h>
#include <qmath.h>
#include "Eigen/Dense"
using namespace Eigen;

class TaskSpace
{
    Vector3d _bodyPosition;
    Matrix3d _bodyAttitude;

    Vector3d _rightFootPosition;
    Vector3d _leftFootPosition;
    Matrix3d _rightFootAttitude;
    Matrix3d _lEftFootAttitude;
public:
    TaskSpace();
    Vector3d GetBodyPosition();
    Matrix3d GetBodyAttitude();

    Vector3d GetRightFootPosition();
    Matrix3d GetRightFootAttitude();

    Matrix3d GetLeftFootAttitude();
    Vector3d GetLeftFootPosition();

    void SetBodyPosition(Vector3d BodyPosition);
    void SetBodyAttitude(Matrix3d BodyAttitude);

    void SetRightFootPosition(Vector3d RightFootPosition);
    void SetRightFootAttitude(Matrix3d RightFootAttitude);

    void SetLeftFootPosition(Vector3d LeftFootPosition);
    void SetLeftFootAttitude(Matrix3d LeftFootAttitude );


};

#endif // TASKSPACE_H
