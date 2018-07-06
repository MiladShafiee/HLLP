#ifndef MinimumJerkInterpolation_H
#define POLYNOMIALCOEF_H
#include <iostream>
#include <fstream>
#include "Eigen/Dense"
#include <qdebug.h>
#include <qmath.h>

using namespace Eigen;
using namespace std;


class PolynomialCoef
{
public:
    PolynomialCoef();
    MatrixXd Coefficient(MatrixXd time, MatrixXd p, MatrixXd dp, MatrixXd dpp);


    void PrintArray(MatrixXd array, QString filename);
    MatrixXd diff(MatrixXd E);
    int length(MatrixXd in);
    QVector<int> findIsinf(MatrixXd in,bool inf);
    MatrixXd MatrixIndex(MatrixXd in, QVector<int> indexes);
    MatrixXd AddtoIndexes(MatrixXd in, MatrixXd val, QVector<int> indexes);
};

#endif // POLYNOMIALCOEF_H
