#include "taskspace.h"

void TaskSpace::SetWalkState(bool walkState)
{
    _walkstate=walkState;
}

void TaskSpace::SetStepTimingGain(double alphaT4)
{
    _alphaT=alphaT4;
}

void TaskSpace::SetStepPositionXGain(double alpha1)
{
    _alpha1x=alpha1;
}

void TaskSpace::SetStepPositionYGain(double alpha1)
{
    _alpha1y=alpha1;
}

void TaskSpace::SetDeltaXGain(double alpha3)
{
    _alpha2x=alpha3;
}

void TaskSpace::SetDeltaYGain(double alpha3)
{
    _alpha2y=alpha3;
}


void TaskSpace::SetDCMOffsetXGain(double alpha4)
{
    _alpha3x=alpha4;
}

void TaskSpace::SetDCMOffsetYGain(double alpha4)
{
    _alpha3y=alpha4;
}
void TaskSpace::SetDesiredMAxFootHeight(double desiredMaxFootHeight){

    _desiredMAxFootHeight=desiredMaxFootHeight;
}

TaskSpace::TaskSpace()
{
    //    _RightStart=false ;
    //    _LeftStart=true;
    //    _Xoffset=0.05;
    //    _SpatialStepSize=0.01;
    //    _nSteps=7;
    //    //_stepLength=stepLength;
    //    _ankleLength=0.05;
    //    _tempTigh=Links[MapingName2ID.value("RLeg_KneeP_J4")].GetJointPositionRelative2Parent();
    //    _thighLength=_tempTigh(2);
    //    _tempShank=Links[MapingName2ID.value("RLeg_AnkleP_J5")].GetJointPositionRelative2Parent();
    //    _shankLength=_tempShank(2);
    //    //_XMiddleOfStep=_stepLength/2;
    //    _legLength=_shankLength+_thighLength;

    //    MatrixXd _xHip;
    //    MatrixXd _zHip;
    _XDesiredVelocity=1;
    _YDesiredVelocity=0;
    SetStepTimingGain(1);
    SetStepPositionXGain(1);
    SetDCMOffsetXGain(10000);
    SetDeltaXGain(1);
    _n;

    SetStepPositionYGain(1);
    SetDCMOffsetYGain(10000);
    SetDeltaYGain(1);
    time=0;
    globalTime=0;
    _MinStepLength = -0.500;
    _MinStepDuration= 0.2000;
    _MaxStepLength =  0.500;
    _MaxStepDuration = 0.8000;
    _MinStepWidth=0.1;
    _MaxStepWidth=0.4;
    _xdeltaMax=1;
    _xdeltaMin=1;
    _xdeltaNom=1;
    _ydeltaMax=1;
    _ydeltaMin=1;
    _ydeltaNom=1;
    _mass=60;
    _gravity=9.80000;
    _PelvisHeight=0.8;
    _pelvisLength=0.2;
    _omega=sqrt(_gravity/_PelvisHeight);
    GetDesiredParameter();

_desiredMAxFootHeight=0.2;

    _DCM.resize(1,2);
    _DCM.fill(0);
    _timeStep=0.001;
    InitialCoP.resize(1,2);
    InitialCoP<<0,0.1;
    _CoPHeel.resize(1,2);
    _CoPHeel<<0,0;

    _CoPToe.resize(1,2);
    _CoPToe<<0,0;

    _nSteps=8;
    _stepNumber=1;
    _stepDuration=0.35;
    _xDCMOffsetNom=0.1456;
    double mmmm;
    mmmm=InitialCoP(0,1)+-1*(_pelvisLength/(1+exp(_omega*_desiredStepDuration)));
    _yDCMOffsetNom=InitialCoP(0,1)+(-1)*(_pelvisLength/(1+exp(_omega*_desiredStepDuration)))-_desiredStepWidth/(1-exp(_omega*_desiredStepDuration));


    _XCoPDisplacementSS=0.00000;
    _YCoPDisplacementSS=0.00000;

    _XError=0;
    _YError=0;

    _error.resize(2,1);
    _error<<_XError,_YError;

    _gama.resize(1,2);
    _gama<<(exp(1*_omega*_desiredStepDuration)),0;

    _yDCMOffset=_yDCMOffsetNom;
    _xDCMOffset=_xDCMOffsetNom;

    InitialDCM.resize(1,2);
    InitialDCM<<_xDCMOffsetNom,_yDCMOffsetNom;

    _CoPDisplacement.resize(2,1);
    _CoPDisplacement<<_XCoPDisplacementSS,_YCoPDisplacementSS;

    RightFootXVelocity.append(0);
    RightFootYVelocity.append(0);

    RightFootXTrajectory.append(0);
    RightFootYTrajectory.append(0);

    RightFootXAcceleration.append(0);
    RightFootYAcceleration.append(0);

    while (_walkstate==true) {

        //        QElapsedTimer timer;
        //        qint64 nanoSec;
        //        timer.start();//----------------------------------------------------------------

        MatrixXd DCM=CoMDynamics(_stepNumber,_gama(_gama.rows()-1,0),_CoPDisplacement);

        _n=_stepNumber-1;

        if ((_n%2)==0){
            _MinStepWidth = InitialCoP(InitialCoP.rows()-1,1)-0.40;
            _MaxStepWidth = InitialCoP(InitialCoP.rows()-1,1)-.10;

        }
        else {
            _MinStepWidth = InitialCoP(InitialCoP.rows()-1,1)+0.10;
            _MaxStepWidth = InitialCoP(InitialCoP.rows()-1,1)+0.40;

        }
        _yDCMOffsetNom=(pow(-1,_n))*(_pelvisLength/(1+exp(_omega*_desiredStepDuration)))-_desiredStepWidth/(1-exp(_omega*_desiredStepDuration));


        _MinStepLength=InitialCoP(_stepNumber-1,0)-0.5000;
        _MaxStepLength=InitialCoP(_stepNumber-1,0)+0.50000;
        Input=QPController(_stepNumber,_CoPDisplacement);
        _gama.conservativeResize(_gama.rows()+1,NoChange);
        _gama(_gama.rows()-1,0)=Input(3,0);
        _stepDuration=(1/_omega)*log(Input(3,0));
        timeVector.append(globalTime);
        DCMXVector.append(DCM(0,1));
        EndCoPYVector.append( InitialCoP( InitialCoP.rows()-1,1));
        if (globalTime==0) {
            RightFootXTrajectory.append(InitialCoP(0,0));
            RightFootXVelocity.append(0);
            RightFootXAcceleration.append(0);


            RightFootYTrajectory.append(-1*InitialCoP(0,1));
            RightFootYVelocity.append(0);
            RightFootYAcceleration.append(0);


            RightFootZTrajectory.append(0);
            RightFootZVelocity.append(0);
            RightFootZacceleration.append(0);
        }
        else {
            MatrixXd tempRightFoot= RightFoot();

            RightFootXTrajectory.append(tempRightFoot(0,0));
            RightFootXVelocity.append(tempRightFoot(0,1));
            RightFootXAcceleration.append(tempRightFoot(0,2));

            RightFootYTrajectory.append(tempRightFoot(0,3));
            RightFootYVelocity.append(tempRightFoot(0,4));
            RightFootYAcceleration.append(tempRightFoot(0,5));


            RightFootZTrajectory.append(tempRightFoot(0,6));
            RightFootZVelocity.append(tempRightFoot(0,7));
            RightFootZacceleration.append(tempRightFoot(0,8));
        }



        time=time+_timeStep;
        globalTime=globalTime+_timeStep;


        //        nanoSec = timer.nsecsElapsed();
        //        qDebug()<< nanoSec*pow(10,-6);


        if (time>_stepDuration) {
            //if stepduration is nan add please
            time=0;
            InitialDCM.conservativeResize(InitialDCM.rows()+1,NoChange);
            InitialDCM(InitialDCM.rows()-1,0)=DCM(0,0);
            InitialDCM(InitialDCM.rows()-1,1)=DCM(0,1);


            InitialCoP.conservativeResize(InitialCoP.rows()+1,NoChange);
            InitialCoP(InitialCoP.rows()-1,0)=Input(0,0);
            InitialCoP(InitialCoP.rows()-1,1)=Input(4,0);

            _stepNumber=_stepNumber+1;
            _xdeltaMax =((2*_XCoPDisplacementSS)/InitialCoP(InitialCoP.rows()-1,0))+1;
            _xdeltaMin =((2*_XCoPDisplacementSS)/InitialCoP(InitialCoP.rows()-1,0))+1;
            _xdeltaNom =((2*_XCoPDisplacementSS)/InitialCoP(InitialCoP.rows()-1,0))+1;


            _ydeltaMax =((2*_YCoPDisplacementSS)/InitialCoP(InitialCoP.rows()-1,1))+1;
            _ydeltaMin =((2*_YCoPDisplacementSS)/InitialCoP(InitialCoP.rows()-1,1))+1;
            _ydeltaNom =((2*_YCoPDisplacementSS)/InitialCoP(InitialCoP.rows()-1,1))+1;
        }

        if(_stepNumber==_nSteps+1){
            break;
        }

    }

}

MatrixXd TaskSpace::QPController(int StepNumber,MatrixXd CoPDisplacementSS){
    MatrixXd H(7,7);
    MatrixXd F(1,7);
    MatrixXd Aiq(10,7);
    MatrixXd biq(10,1);
    MatrixXd Aeq(2,7);
    MatrixXd beq(1,2);
    double alphaa=0.000000;
    double XCoPDisplacementSS;
    double YCoPDisplacementSS;
    XCoPDisplacementSS=CoPDisplacementSS(0,0);
    YCoPDisplacementSS=CoPDisplacementSS(1,0);

    H<<_alpha1x,0,0,0,0,0,0,
            0,_alpha2x,0,0,0,0,0,
            0,0,_alpha3x,0,0,0,0,
            0,0,0,_alphaT,0,0,0,
            0,0,0,0,_alpha1y,0,0,
            0,0,0,0,0,_alpha2y,0,
            0,0,0,0,0,0,_alpha3y;

    F<<-1*_alpha1x*_desiredStepLength,
            -1*_alpha2x*_xdeltaNom,
            -1*_alpha3x*_xDCMOffsetNom,
            -1*_alphaT*_sigmaDesired,
            -1*_alpha1y*_desiredStepWidth,
            -1*_alpha2y*_ydeltaNom,
            -1*_alpha3y*_yDCMOffsetNom;

    Aiq<<1,0,0,0,0,0,0,
            -1,0,0,0,0,0,0,
            0,+1,0,0,0,0,0,
            0,-1,0,0,0,0,0,
            0,0,0,+1,0,0,0,
            0,0,0,-1,0,0,0,
            0,0,0,0,+1,0,0,
            0,0,0,0,-1,0,0,
            0,0,0,0,0,+1,0,
            0,0,0,0,0,-1,0;

    biq<<_MaxStepLength,
            -1*_MinStepLength,
            _xdeltaMax,
            -1*_xdeltaMin,
            exp(_omega*_MaxStepDuration),
            -1*exp(_omega*_MinStepDuration),
            1*_MaxStepWidth,
            -1*_MinStepWidth,
            _ydeltaMax,
            -1*_ydeltaMin;

    if (StepNumber==1){
        Aeq<<1,-(InitialCoP(StepNumber-1,0)-XCoPDisplacementSS),1,((InitialCoP(StepNumber-1,0)-XCoPDisplacementSS)-_XError*exp(-_omega*time)-InitialDCM(StepNumber-1,0)),0,0,0,
                0,0,0,((InitialCoP(StepNumber-1,1)-YCoPDisplacementSS)-_YError*exp(-_omega*time)-InitialDCM(StepNumber-1,1)),1,-(InitialCoP(StepNumber-1,1)-YCoPDisplacementSS),1;
        beq<<0,0;
    }
    else{
        Aeq<<1,-InitialCoP(StepNumber-1,0),1,((InitialCoP(StepNumber-1,0))-_XError*exp(-_omega*time)-InitialDCM(StepNumber-1,0)),0,0,0,
                0,0,0,(InitialCoP(StepNumber-1,1)-_YError*exp(-_omega*time)-InitialDCM(StepNumber-1,1)),1,-InitialCoP(StepNumber-1,1),1;
        beq<<0,0;
    }

    VectorXd sol(H.rows());
    MatrixXd HTemp2 = H;

    MatrixXd CE(7,2);
    VectorXd ce0(2);
    CE<<Aeq.transpose();
    ce0<<beq(0,0),beq(0,1);

    VectorXd g(Map<VectorXd>(F.data(),F.cols()*F.rows()));

    double optCosts;
    VectorXd  ci0=biq;
    MatrixXd CIT=Aiq.transpose();


    QElapsedTimer timer;
    qint64 nanoSec;
    timer.start();//----------------------------------------------------------------


    optCosts = solve_quadprog(HTemp2, g, -1*CE,ce0 ,-1*CIT , ci0, sol);
    if(optCosts==std::numeric_limits<double>::infinity()){
              qDebug("Quadratic Programming for DCM Planning failed!");
   }
    MatrixXd ux = sol;
    nanoSec = timer.nsecsElapsed();
   // qDebug()<< nanoSec*pow(10,-6);
    return ux;

    //                //min  0.5 * x H x + g x
    //                //s.t. CE^T x + ce0 = 0
    //                //     CI^T x + ci0 >= 0
    //                // Since you cannot explicitly express bilateral constraints
    //                // consider only lower bounds
}

MatrixXd TaskSpace::CoMDynamics(int stepNumber, double GamaX, MatrixXd CoPDisplacementSS){
    double XCoPDisplacementSS;
    double YCoPDisplacementSS;
    MatrixXd TempDCM;
    XCoPDisplacementSS=CoPDisplacementSS(0,0);
    YCoPDisplacementSS=CoPDisplacementSS(1,0);

    _DCM.conservativeResize(_DCM.rows()+1,NoChange);
    _DCM(_DCM.rows()-1,0)=((InitialCoP(stepNumber-1,0)- XCoPDisplacementSS)*(1-_xdeltaNom)/(1-(1/GamaX)))*(0.500*exp(-_omega*time)-0.500*exp(_omega*time))+(InitialCoP(stepNumber-1,0)- XCoPDisplacementSS)*((_xdeltaNom-(1/GamaX))/(1-(1/GamaX)))*(1-exp(_omega*time))+InitialDCM(stepNumber-1,0)*exp(_omega*time);
    _DCM(_DCM.rows()-1,1)=((InitialCoP(stepNumber-1,1)- YCoPDisplacementSS)*(1-_ydeltaNom)/(1-(1/GamaX)))*(0.500*exp(-_omega*time)-0.500*exp(_omega*time))+(InitialCoP(stepNumber-1,1)- YCoPDisplacementSS)*((_ydeltaNom-(1/GamaX))/(1-(1/GamaX)))*(1-exp(_omega*time))+InitialDCM(stepNumber-1,1)*exp(_omega*time);
    double _DCMx=_DCM(_DCM.rows()-1,0);
    double _DCMy=_DCM(_DCM.rows()-1,1);
    TempDCM.resize(1,2);
    TempDCM<<_DCMx,_DCMy;
    return TempDCM;

}



void TaskSpace::GetDesiredParameter(){
    double LowerBound1;
    double UpperBound1;

    double LowerBound2;
    double UpperBound2;

    double LowerBound3;
    double UpperBound3;

    double LowerBound;
    double UpperBound;
    LowerBound1=_MinStepLength/(std::abs(_XDesiredVelocity));
    UpperBound1=_MaxStepLength/(std::abs(_XDesiredVelocity));

    LowerBound2=_MinStepWidth/(std::abs(_YDesiredVelocity));
    UpperBound2=_MaxStepWidth/(std::abs(_YDesiredVelocity));

    LowerBound3=_MinStepDuration;
    UpperBound3=_MaxStepDuration;
    if (_XDesiredVelocity==0){
        LowerBound=max(LowerBound2,LowerBound3);
        UpperBound=min(UpperBound2,UpperBound3);
    }
    else if (_YDesiredVelocity==0) {


        LowerBound=max(LowerBound1,LowerBound3);
        UpperBound=min(UpperBound1,UpperBound3);


    }
    else {
        double tempL1=max(LowerBound1,LowerBound3);
        double tempL2=max(LowerBound1,LowerBound2);

        double tempU1=min(LowerBound1,LowerBound3);
        double tempU2=min(LowerBound1,LowerBound2);


        LowerBound=max(tempL1,tempL2);
        UpperBound=min(tempU1,tempU2);

    }
    _desiredStepDuration=(LowerBound+UpperBound)/2;
    _desiredStepLength=_XDesiredVelocity*(LowerBound+UpperBound)/2;
    _desiredStepWidth=_YDesiredVelocity*(LowerBound+UpperBound)/2;
    _sigmaDesired= exp(_omega*_desiredStepDuration);

}

MatrixXd TaskSpace::RightFoot(){
    double x;
    double xf;
    double dx;
    double ddx;
    double y;
    double dy;
    double ddy;
    double z;
    double dz;
    double ddz;
    double Yaw;
    double Roll;
    double Pitch;
    MatrixXd Time;
    MatrixXd xposition;
    MatrixXd xvelocity;
    MatrixXd xacceleration;
double t0=0;
    MatrixXd yposition;
    MatrixXd yvelocity;
    MatrixXd yacceleration;
    Time.resize(1,2);
    Time<<time-_timeStep,(1/_omega)*log(Input(3,0));


    if (_n%2==0){
        //left foot is support foot
        //        if (time==_timeStep){
        //            position.resize(1,2);
        //            velocity.resize(1,2);
        //            acceleration.resize(1,2);
        //            position(0,0)=RightFootXTrajectory[RightFootXTrajectory.size()-1];
        //            velocity(0,0)=0;
        //            acceleration(0,0)=0;
        //            position(0,1)=Input(0,0);
        //            velocity(0,1)=0;
        //            acceleration(0,1)=0;
        //        }


        //   else{

        xposition.resize(1,2);
        xvelocity.resize(1,2);
        xacceleration.resize(1,2);
        xposition(0,0)=RightFootXTrajectory[RightFootXTrajectory.size()-1];
        xvelocity(0,0)=RightFootXVelocity[RightFootXVelocity.size()-1];
        xacceleration(0,0)=RightFootXAcceleration[RightFootXAcceleration.size()-1];
        xposition(0,1)=Input(0,0);
        xvelocity(0,1)=0;
        xacceleration(0,1)=0;
        //  }


        yposition.resize(1,2);
        yvelocity.resize(1,2);
        yacceleration.resize(1,2);
        yposition(0,0)=RightFootYTrajectory[RightFootYTrajectory.size()-1];
        yvelocity(0,0)=RightFootYVelocity[RightFootYVelocity.size()-1];
        yacceleration(0,0)=RightFootYAcceleration[RightFootYAcceleration.size()-1];
        yposition(0,1)=Input(4,0);
        yvelocity(0,1)=0;
        yacceleration(0,1)=0;

        MatrixXd CoefX =Coef.Coefficient(Time,xposition,xvelocity,xacceleration);
        MatrixXd outputx= GetAccVelPos(CoefX.topRows(1),time,time-_timeStep,5);

        x=outputx(0,0);
        dx=outputx(0,1);
        ddx=outputx(0,2);

        MatrixXd CoefY =Coef.Coefficient(Time,yposition,yvelocity,yacceleration);
        MatrixXd outputy= GetAccVelPos(CoefY.topRows(1),time,time-_timeStep,5);

        y=outputy(0,0);
        dy=outputy(0,1);
        ddy=outputy(0,2);

    }


    else {
        x=RightFootXTrajectory[RightFootXTrajectory.size()-1];
        dx=0;
        ddx=0;

        y=RightFootYTrajectory[RightFootYTrajectory.size()-1];
        dy=0;
        ddy=0;
    }

    MatrixXd H(10,10);
    MatrixXd F(1,10);
    MatrixXd Aiq(2,10);
    MatrixXd biq(2,1);
    MatrixXd Aeq;
    MatrixXd beq;

if(time!=_timeStep){
     Aeq.resize(9,10);
     beq.resize(9,1);

}

else{

     Aeq.resize(6,10);
     beq.resize(6,1);


}

    H<<2*pow((Time(0,1)/2-t0),18),2*pow((Time(0,1)/2-t0),17),2*pow((Time(0,1)/2-t0),16),2*pow((Time(0,1)/2-t0),15),2*pow((Time(0,1)/2-t0),14),2*pow((Time(0,1)/2-t0),13),2*pow((Time(0,1)/2-t0),12),2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),
       2*pow((Time(0,1)/2-t0),17),2*pow((Time(0,1)/2-t0),16),2*pow((Time(0,1)/2-t0),15),2*pow((Time(0,1)/2-t0),14),2*pow((Time(0,1)/2-t0),13),2*pow((Time(0,1)/2-t0),12),2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),
       2*pow((Time(0,1)/2-t0),16),2*pow((Time(0,1)/2-t0),15),2*pow((Time(0,1)/2-t0),14),2*pow((Time(0,1)/2-t0),13),2*pow((Time(0,1)/2-t0),12),2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),
       2*pow((Time(0,1)/2-t0),15),2*pow((Time(0,1)/2-t0),14),2*pow((Time(0,1)/2-t0),13),2*pow((Time(0,1)/2-t0),12),2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),2*pow((Time(0,1)/2-t0),6),
       2*pow((Time(0,1)/2-t0),14),2*pow((Time(0,1)/2-t0),13),2*pow((Time(0,1)/2-t0),12),2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),2*pow((Time(0,1)/2-t0),6),2*pow((Time(0,1)/2-t0),5),
       2*pow((Time(0,1)/2-t0),13),2*pow((Time(0,1)/2-t0),12),2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),2*pow((Time(0,1)/2-t0),6),2*pow((Time(0,1)/2-t0),5),2*pow((Time(0,1)/2-t0),4),
       2*pow((Time(0,1)/2-t0),12),2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),2*pow((Time(0,1)/2-t0),6),2*pow((Time(0,1)/2-t0),5),2*pow((Time(0,1)/2-t0),4),2*pow((Time(0,1)/2-t0),3),
       2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),2*pow((Time(0,1)/2-t0),6),2*pow((Time(0,1)/2-t0),5),2*pow((Time(0,1)/2-t0),4),2*pow((Time(0,1)/2-t0),3),2*pow((Time(0,1)/2-t0),2),
       2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),2*pow((Time(0,1)/2-t0),6),2*pow((Time(0,1)/2-t0),5),2*pow((Time(0,1)/2-t0),4),2*pow((Time(0,1)/2-t0),3),2*pow((Time(0,1)/2-t0),2),2*pow((Time(0,1)/2-t0),1),
       2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),2*pow((Time(0,1)/2-t0),6),2*pow((Time(0,1)/2-t0),5),2*pow((Time(0,1)/2-t0),4),2*pow((Time(0,1)/2-t0),3),2*pow((Time(0,1)/2-t0),2),2*pow((Time(0,1)/2-t0),1),2*pow((Time(0,1)/2-t0),0);

    F<<-2*pow((Time(0,1)/2-t0),9)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),8)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),7)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),6)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),5)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),4)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),3)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),2)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),1)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),0)*_desiredMAxFootHeight;


    Aiq<<pow((time-t0),9),pow((time-t0),8),pow((time-t0),7),pow((time-t0),6),pow((time-t0),5),pow((time-t0),4),pow((time-t0),3),pow((time-t0),2),pow((time-t0),1),pow((time-t0),0),
         pow(-1*(time-t0),9),-1*pow((time-t0),8),-1*pow((time-t0),7),-1*pow((time-t0),6),-1*pow((time-t0),5),-1*pow((time-t0),4),-1*pow((time-t0),3),-1*pow((time-t0),2),-1*pow((time-t0),1),-1*pow((time-t0),0);
    biq<<_desiredMAxFootHeight+0.01,0;
if(time!=_timeStep){
    Aeq<<pow((0-t0),9),pow((0-t0),8),pow((0-t0),7),pow((0-t0),6),pow((0-t0),5),pow((0-t0),4),pow((0-t0),3),pow((0-t0),2),pow((0-t0),1),1,
            pow((time-_timeStep-t0),9),pow((time-_timeStep-t0),8),pow((time-_timeStep-t0),7),pow((time-_timeStep-t0),6),pow((time-_timeStep-t0),5),pow((time-_timeStep-t0),4),pow((time-_timeStep-t0),3),pow((time-_timeStep-t0),2),pow((time-_timeStep-t0),1),1,
            pow((Time(0,1)-t0),9),pow((Time(0,1)-t0),8),pow((Time(0,1)-t0),7),pow((Time(0,1)-t0),6),pow((Time(0,1)-t0),5),pow((Time(0,1)-t0),4),pow((Time(0,1)-t0),3),pow((Time(0,1)-t0),2),pow((Time(0,1)-t0),1),1,
            9*pow((0-t0),8),8*pow((0-t0),7),7*pow((0-t0),6),6*pow((0-t0),5),5*pow((0-t0),4),4*pow((0-t0),3),3*pow((0-t0),2),2*pow((0-t0),1),1*pow((0-t0),0),0*pow((0-t0),0),
            9*pow((time-_timeStep-t0),8),8*pow((time-_timeStep-t0),7),7*pow((time-_timeStep-t0),6),6*pow((time-_timeStep-t0),5),5*pow((time-_timeStep-t0),4),4*pow((time-_timeStep-t0),3),3*pow((time-_timeStep-t0),2),2*pow((time-_timeStep-t0),1),1,0,
            9*pow((Time(0,1)-t0),8),8*pow((Time(0,1)-t0),7),7*pow((Time(0,1)-t0),6),6*pow((Time(0,1)-t0),5),5*pow((Time(0,1)-t0),4),4*pow((Time(0,1)-t0),3),3*pow((Time(0,1)-t0),2),2*pow((Time(0,1)-t0),1),1*pow((Time(0,1)-t0),0),0*pow((Time(0,1)-t0),0),
            9*8*pow((0-t0),7),8*7*pow((0-t0),6),7*6*pow((0-t0),5),6*5*pow((0-t0),4),5*4*pow((0-t0),3),4*3*pow((0-t0),2),3*2*pow((0-t0),1),2*1*pow((0-t0),0),0*pow((0-t0),0),0*pow((0-t0),0),
            9*8*pow((time-_timeStep-t0),7),8*7*pow((time-_timeStep-t0),6),7*6*pow((time-_timeStep-t0),5),6*5*pow((time-_timeStep-t0),4),5*4*pow((time-_timeStep-t0),3),4*3*pow((time-_timeStep-t0),2),3*2*pow((time-_timeStep-t0),1),2,0,0,
            9*8*pow((Time(0,1)-t0),7),8*7*pow((Time(0,1)-t0),6),7*6*pow((Time(0,1)-t0),5),6*5*pow((Time(0,1)-t0),4),5*4*pow((Time(0,1)-t0),3),4*3*pow((Time(0,1)-t0),2),3*2*pow((Time(0,1)-t0),1),2*1*pow((Time(0,1)-t0),0),0,0;

     beq<<0,RightFootZTrajectory[RightFootZTrajectory.size()-1],0,0,RightFootZVelocity[RightFootZVelocity.size()-1],0,0,RightFootZacceleration[RightFootZacceleration.size()-1],0;
   }
else{
    Aeq<<pow((0-t0),9),pow((0-t0),8),pow((0-t0),7),pow((0-t0),6),pow((0-t0),5),pow((0-t0),4),pow((0-t0),3),pow((0-t0),2),pow((0-t0),1),1,
            pow((Time(0,1)-t0),9),pow((Time(0,1)-t0),8),pow((Time(0,1)-t0),7),pow((Time(0,1)-t0),6),pow((Time(0,1)-t0),5),pow((Time(0,1)-t0),4),pow((Time(0,1)-t0),3),pow((Time(0,1)-t0),2),pow((Time(0,1)-t0),1),1,
            9*pow((0-t0),8),8*pow((0-t0),7),7*pow((0-t0),6),6*pow((0-t0),5),5*pow((0-t0),4),4*pow((0-t0),3),3*pow((0-t0),2),2*pow((0-t0),1),1*pow((0-t0),0),0*pow((0-t0),0),
            9*pow((Time(0,1)-t0),8),8*pow((Time(0,1)-t0),7),7*pow((Time(0,1)-t0),6),6*pow((Time(0,1)-t0),5),5*pow((Time(0,1)-t0),4),4*pow((Time(0,1)-t0),3),3*pow((Time(0,1)-t0),2),2*pow((Time(0,1)-t0),1),1*pow((Time(0,1)-t0),0),0*pow((Time(0,1)-t0),0),
            9*8*pow((0-t0),7),8*7*pow((0-t0),6),7*6*pow((0-t0),5),6*5*pow((0-t0),4),5*4*pow((0-t0),3),4*3*pow((0-t0),2),3*2*pow((0-t0),1),2*1*pow((0-t0),0),0*pow((0-t0),0),0*pow((0-t0),0),
            9*8*pow((Time(0,1)-t0),7),8*7*pow((Time(0,1)-t0),6),7*6*pow((Time(0,1)-t0),5),6*5*pow((Time(0,1)-t0),4),5*4*pow((Time(0,1)-t0),3),4*3*pow((Time(0,1)-t0),2),3*2*pow((Time(0,1)-t0),1),2*1*pow((Time(0,1)-t0),0),0,0;



    beq<<0,0,0,0,0,0;

}

    VectorXd sol(H.rows());
    MatrixXd HTemp2 = H;

    VectorXd  ci0=biq;
    MatrixXd CIT=-1*Aiq.transpose();


    MatrixXd CE=-1*Aeq.transpose();
    VectorXd ce0=beq;
   // CE<<
   // ce0<<beq(0,0),beq(0,1),beq(0,2),beq(0,3),beq(0,4),beq(0,5),beq(0,6),beq(0,7),beq(0,8);

    VectorXd g(Map<VectorXd>(F.data(),F.cols()*F.rows()));

    double optCost;



    optCost = solve_quadprog(HTemp2, g, CE,ce0 ,CIT , ci0, sol);
    if(optCost==std::numeric_limits<double>::infinity()){
              qDebug("Quadratic Programming for Foot Z Trajectory failed!");
   }

    MatrixXd CoefZ = sol;
    MatrixXd outputz= GetAccVelPos(CoefZ.transpose(),time,0,9);
    MatrixXd Milad=CoefZ.transpose();
    MatrixXd Ali=Milad.rowwise().reverse();
    MatrixXd outputz2= GetAccVelPos(CoefZ.transpose(),Time(0,1),0,9);


    MatrixXd milll(1,10);
    milll<<pow((time-t0),9),pow((time-t0),8),pow((time-t0),7),pow((time-t0),6),pow((time-t0),5),pow((time-t0),4),pow((time-t0),3),pow((time-t0),2),pow((time-t0),1),1;
    MatrixXd jack=milll*CoefZ;
    z=outputz(0,0);
    dz=outputz(0,1);
    ddz=outputz(0,2);
if(time==0.18){
    qDebug()<<"1";
}

    MatrixXd FootTrajectories(1,9);
    FootTrajectories<<x,dx,ddx,y,dy,ddy,z,dz,ddz;
    return FootTrajectories;
}



MatrixXd TaskSpace::GetAccVelPos(MatrixXd Coef,double time,double ti,int PolynomialOrder)
{
    int PolyNomialDegree=PolynomialOrder;
    MatrixXd T(PolyNomialDegree+1,1);
    T.fill(0);
    MatrixXd Diag(PolyNomialDegree+1,PolyNomialDegree+1);
    Diag.fill(0);
    for (int var = 0; var < PolyNomialDegree+1; var++) {
        T(var,0)=pow((time-ti),PolyNomialDegree-var);
        if (var!=0) {
            Diag.diagonal(1)(var-1,0)=PolyNomialDegree-var+1;

        }
    }

    MatrixXd x=Coef*T;
    double X=x(0,0);

    MatrixXd v=Coef*Diag*T;
    double V=v(0,0);

    MatrixXd a=Coef*Diag*Diag*T;
    double A=a(0,0);

    MatrixXd Output(1,3);
    Output<<X,V,A;
    return Output;
}
