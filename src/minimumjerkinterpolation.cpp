#include "minimumjerkinterpolation.h"

MinimumJerkInterpolation::MinimumJerkInterpolation()
{

}

MatrixXd MinimumJerkInterpolation::Coefficient(MatrixXd time, MatrixXd p, MatrixXd dp, MatrixXd ddp)
{
    int n = length(time) - 1;
    if (n != (length(p)-1))
    {
        qDebug("Size of Position Error!");
        MatrixXd temp;
        return temp;
    }
    if (n != (length(dp)-1))
    {
        qDebug("Size of Velocity Error!");
        MatrixXd temp;
        return temp;
    }
    if (n != (length(ddp)-1))
    {
        qDebug("Size of Acceleration Error!");
        MatrixXd temp;
        return temp;
    }

    if(diff(time).minCoeff() <= 0)
    {
        qDebug("Time Error!");
        MatrixXd temp;
        return temp;
    }
    MatrixXd A = MatrixXd::Zero(6*n,6*n);
    MatrixXd H = MatrixXd::Zero(6*n,6*n);
    MatrixXd M = MatrixXd::Zero(6*n,3*(n+1));

    for(int i = 0; i < n;i++){
        double ti = time(0,i);
        double tf = time(0,i+1);
        MatrixXd Ai(6,6);
        Ai << (6/(-1*pow(tf,5) + 5*pow(tf,4)*ti - 10*pow(tf,3)*pow(ti,2) + 10*pow(tf,2)*pow(ti,3) - 5*tf*pow(ti,4) + pow(ti,5))), -3/(pow(tf,4) - 4*pow(tf,3)*ti + 6*pow(tf,2)*pow(ti,2) - 4*tf*pow(ti,3) + pow(ti,4)), -1/(2*(pow(tf,3) - 3*pow(tf,2)*ti + 3*tf*pow(ti,2) - pow(ti,3))), -6/(-1*pow(tf,5) + 5*pow(tf,4)*ti - 10*pow(tf,3)*pow(ti,2)+ 10*pow(tf,2)*pow(ti,3) - 5*tf*pow(ti,4) + pow(ti,5)), -3/(pow(tf,4) - 4*pow(tf,3)*ti + 6*pow(tf,2)*pow(ti,2) - 4*tf*pow(ti,3) + pow(ti,4)), 1/(2*(pow(tf,3) - 3*pow(tf,2)*ti + 3*tf*pow(ti,2) - pow(ti,3))),
            15/(pow(tf,4) - 4*pow(tf,3)*ti + 6*pow(tf,2)*pow(ti,2) - 4*tf*pow(ti,3) + pow(ti,4)), (8/(pow(tf,3) - 3*pow(tf,2)*ti + 3*tf*pow(ti,2) - pow(ti,3))), 3/(2*(pow(tf,2) - 2*tf*ti + pow(ti,2))), -15/(pow(tf,4) - 4*pow(tf,3)*ti + 6*pow(tf,2)*pow(ti,2) - 4*tf*pow(ti,3) + pow(ti,4)), 7/(pow(tf,3) - 3*pow(tf,2)*ti + 3*tf*pow(ti,2) - pow(ti,3)), -1/(pow(tf,2) - 2*tf*ti + pow(ti,2)),
            -10/(pow(tf,3) - 3*pow(tf,2)*ti + 3*tf*pow(ti,2) - pow(ti,3)),    -6/(pow(tf,2) - 2*tf*ti + pow(ti,2)), -3/(2*(tf - ti)), 10/(pow(tf,3) - 3*pow(tf,2)*ti + 3*tf*pow(ti,2) - pow(ti,3)),       -4/(pow(tf,2) - 2*tf*ti + pow(ti,2)),                             1/(2*(tf - ti)),
            0,                                                      0,                                          0.5,                                                                        0,                                                      0,                                           0,
            0,                                                      1,                                            0,                                                                        0,                                                      0,                                           0,
            1,                                                      0,                                            0,                                                                        0,                                                      0,                                           0;

        MatrixXd Hi(6,6);
        Hi << (400*pow((tf - ti),7))/7,      40*pow((tf - ti),6), 24*pow((tf - ti),5), 10*pow((tf - ti),4), 0, 0,
                40*pow((tf - ti),6), (144*pow((tf - ti),5))/5, 18*pow((tf - ti),4),  8*pow((tf - ti),3), 0, 0,
                24*pow((tf - ti),5),      18*pow((tf - ti),4), 12*pow((tf - ti),3),  6*pow((tf - ti),2), 0, 0,
                10*pow((tf - ti),4),       8*pow((tf - ti),3),  6*pow((tf - ti),2),    4*tf - 4*ti, 0, 0,
                0,                   0,              0,              0, 0, 0,
                0,                   0,              0,              0, 0, 0;

        A.block(6*i,6*i,6,6) = Ai;
        H.block(6*i,6*i,6,6) = Hi;

        M.block(6*i,3*i,6,6) = MatrixXd::Identity(6,6);
    }
    MatrixXd F;
    F = (A*M).transpose()*H*(A*M);
    PrintArray(F,"F");

    MatrixXd tempx(p.rows()+dp.rows()+ddp.rows(),p.cols());
    tempx.block(0,0,p.rows(),p.cols()) = p;
    tempx.block(p.rows(),0,dp.rows(),dp.cols()) = dp;
    tempx.block(p.rows() + dp.rows(),0,ddp.rows(),ddp.cols()) = ddp;

    //For reshaping the matrix
    Map<MatrixXd> x_temp(tempx.data(),3*(n+1),1);
    MatrixXd x = x_temp;

    QVector<int> u = findIsinf(x,true);
    QVector<int> v = findIsinf(x,false);

    if(u.length() != 0){
        MatrixXd Fuu(u.length(),u.length());
        MatrixXd Fvv(v.length(),v.length());
        MatrixXd Fuv(u.length(),v.length());
        MatrixXd Fvu(v.length(),u.length());

        for(int i = 0; i < u.length();i++){
            for(int j = 0 ; j < u.length();j++)
                Fuu(i,j) = F(u.at(i),u.at(j));
        }

        for(int i = 0; i < v.length();i++){
            for(int j = 0 ; j < v.length();j++)
                Fvv(i,j) = F(v.at(i),v.at(j));
        }

        for(int i = 0; i < u.length();i++){
            for(int j = 0 ; j < v.length();j++)
                Fuv(i,j) = F(u.at(i),v.at(j));
        }

        for(int i = 0; i < v.length();i++){
            for(int j = 0 ; j < u.length();j++)
                Fvu(i,j) = F(v.at(i),u.at(j));
        }

        MatrixXd G(Fuu.rows()+Fvu.rows(),Fuu.cols()+Fuv.cols());
        G.block(0,0,Fuu.rows(),Fuu.cols()) = Fuu;
        G.block(0,Fuu.rows(),Fuv.rows(),Fuv.cols()) = Fuv;
        G.block(Fuu.rows(),0,Fvu.rows(),Fvu.cols()) = Fvu;
        G.block(Fuu.rows(),Fvu.cols(),Fvv.rows(),Fvv.cols()) = Fvv;

        MatrixXd xv = MatrixIndex(x,v);

        MatrixXd b = 0.5*(xv.transpose()*Fvu+xv.transpose()*Fuv.transpose());
        MatrixXd C = 0.5*(Fuu.transpose()+Fuu);

        MatrixXd x_inf = -1 * C.inverse()*b.transpose();
        x = AddtoIndexes(x,x_inf,u);
    }
    MatrixXd c = A*M*x;
    PrintArray(A,"A");
    PrintArray(M,"M");
    PrintArray(x,"x");
    Map<MatrixXd> coef_temp(c.data(),6,4);
    MatrixXd coef = coef_temp;
    return coef.transpose();

}

MatrixXd MinimumJerkInterpolation::AddtoIndexes(MatrixXd in,MatrixXd val,QVector<int> indexes){
    for(int i = 0; i < indexes.length(); i++)
        in(indexes.at(i),0) = val(i);
    return in;
}

MatrixXd MinimumJerkInterpolation::MatrixIndex(MatrixXd in,QVector<int> indexes){
    MatrixXd result(indexes.length(),1);
    for(int i = 0; i < indexes.length();i++){
        result(i,0) = in(indexes[i],0);
    }
    return result;
}

int MinimumJerkInterpolation::length(MatrixXd in){
    return max(in.rows(),in.cols());
}


QVector<int> MinimumJerkInterpolation::findIsinf(MatrixXd in,bool inf){
    QVector<int> resultInf;
    QVector<int> resultNotInf;
    for(int i = 0; i < in.rows();i++)
    {
        if(in(i,0) == INFINITY)
            resultInf.append(i);
        else
            resultNotInf.append(i);
    }
    if(inf)
        return resultInf;
    else
        return resultNotInf;
}

void MinimumJerkInterpolation::PrintArray(MatrixXd array,QString filename){
    ofstream file(filename.toLatin1() + ".csv");
    if(file.is_open()){
        for(int i = 0;i < array.rows();i++){
            for(int j = 0;j < array.cols();j++)
                file << array(i,j) << ",";
            file << "\r" << endl;
        }
    }
}

MatrixXd MinimumJerkInterpolation::diff(MatrixXd E){
    MatrixXd E1 = E.block(0,0,E.rows(),E.cols()-1);
    MatrixXd E2 = E.block(0,1,E.rows(),E.cols()-1);
    return (E2 - E1);
}
