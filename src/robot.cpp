#include "robot.h"
//note taht ID is one unit more than Links index
Robot::Robot(QObject *parent) : QObject(parent)
{
    QString address="..//src//RobotData//test.txt";
    if (!FileExists(address))
    {
        qWarning()<<"Invalid Robot data Path:"<<address;
        return;
    }
    QFile file(address);
    TaskSpace StraightLEgWalk;
    file.open(QFile::ReadWrite);
    content = file.readAll();


    Links= CreateRobotLinks( content );
    MatrixXi leftRoute= GetLeftLegRoute();
    MatrixXi rightRoute= GetRightLegRoute();

    MatrixXd rightLegAngle(1,6);
    rightLegAngle<<0 ,0 ,-25 ,50, -25, 0;
    VectorXd x;
    SetJointAngle(rightLegAngle*(M_PI/180),rightRoute);
//    Link Rfoot=Links[MapingName2ID.value("RLEG_J6")-1];
//    Rfoot.PositionInWorldCoordinate<<1,2,3;
//    Rfoot.AttitudeInWorldCoordinate<<8,6,5,3,1,7,6,5,4;
   /* MatrixXd temp(6,1);
            temp<<1,2,3,4,5,6;
    MoveJoints(rightRoute,temp);
   MatrixXd milad3= CalcJacobian(rightRoute);
  MatrixXd milad2=  CalcTaskSpaceError(Rfoot,"RLEG_J6");
    cout<<milad2*/;
    x.setLinSpaced(81,0,0.4);


    MatrixXd q_m;
    MatrixXd x_m;

    int Nstep = x.rows();
    q_m = MatrixXd::Zero(Nstep,6);
    x_m =  MatrixXd::Zero(Nstep,1);
    Link Rfoot=Links[MapingName2ID.value("RLEG_J6")-1];
    double errorm;
    for (int var = 0; var < Nstep; var++) {
        Rfoot.PositionInWorldCoordinate(0)=x(var);
        errorm=IKLevenbergMarquardt(Rfoot,"RLEG_J6");
qDebug()<<errorm;
    }


    file.close();

}

QList<Link> Robot::GetLinks()
{
    return Links;
}

MatrixXi Robot::Getroute()
{
    return _route;
}

MatrixXi Robot::GetLeftLegRoute()
{
    MatrixXi _leftLegRoute= FindRoutfromRoot2Destination("LLEG_J6");
    return _leftLegRoute;
}

void Robot::SetLeftLegRoute(MatrixXd leftLegRoute)
{
    _leftLegRoute=leftLegRoute;
}

MatrixXi Robot::GetRightLegRoute()
{
    MatrixXi _rightLegRoute= FindRoutfromRoot2Destination("RLEG_J6");
    return _rightLegRoute;
}

void Robot::SetRightLegRoute(MatrixXd rightLegRoute)
{
    _rightLegRoute=rightLegRoute;
}

QList <QByteArray> Robot::GetContentOfRobot(QString name,QByteArray content)
{
    QList <QByteArray> result;
    int index=0;
    while (index<content.length()) {

        index=content.indexOf(name,index);
        if (index<0)
        {
            break;
        }

        QByteArray temp=content.mid(index,content.length());
        QByteArray line=temp.split('\n')[0];
        QList <QByteArray> values=line.split('=');
        if (values.length()>1){
            result.append(values[1]);
        }
        else
        {
            qDebug()<<"warning:expected '=' after value"<<name;
        }
        index++;
    }
    return result;
}


bool Robot::FileExists(QString path) {
    QFileInfo check_file(path);
    // check if file exists and if yes: Is it really a file and no directory?
    if (check_file.exists() && check_file.isFile()) {
        return true;
    } else {
        return false;
    }
}



MatrixXd Robot::ExtractionOfMatrix(QByteArray data)
{
    MatrixXd mat;
    QByteArray insideBrackets=data.split(']')[0];
    insideBrackets= insideBrackets.split('[')[1];
    QList <QByteArray> rows=insideBrackets.split(';');
    QList <QByteArray> columns=rows[0].split(',');

    mat.resize(rows.length(),columns.length());

    //////initial mat values
    for (int i = 0; i < rows.length(); i++) {
        QList <QByteArray> currentCols=rows[i].split(',');
        for (int j = 0; j < currentCols.length(); j++) {
            mat(i,j) = currentCols[j].toDouble();

        }
    }


    return mat;
}



QList <Link> Robot::CreateRobotLinks(QByteArray content )
{
    QList<QByteArray> NameOfLinks= GetContentOfRobot("Name",content);
    QList<QByteArray> IDofLink= GetContentOfRobot("LinkID",content);
    QList<QByteArray> IDofSister= GetContentOfRobot("SisterID",content);
    QList<QByteArray> IDofChild= GetContentOfRobot("ChildID",content);
    QList<QByteArray> IDofMother= GetContentOfRobot("MotherID",content);
    QList<QByteArray> AngleOfJoint= GetContentOfRobot("JointAngle",content);
    QList<QByteArray> LocalAxisVectorOfJoint= GetContentOfRobot("JointAxisVectorLocal",content);
    QList<QByteArray> PositionRelative2ParentOfJoint= GetContentOfRobot("JointPositionRelative2Parent",content);



    for (int index = 0; index < NameOfLinks.count(); index++) {

        QString linkName=QString::fromLatin1(NameOfLinks[index].data());
        int iDofSister=QString::fromLatin1(IDofSister[index].data()).toInt();
        int iDofChild=QString::fromLatin1(IDofChild[index].data()).toInt();
        int iDofLink=QString::fromLatin1(IDofLink[index].data()).toInt();
        double angleOfJoint=QString::fromLatin1(AngleOfJoint[index].data()).toDouble();
        int iDofMother=QString::fromLatin1(IDofMother[index].data()).toInt();
        MatrixXd localAxisVectorOfJoint= ExtractionOfMatrix(LocalAxisVectorOfJoint[index]);
        MatrixXd positionRelative2ParentOfJoint= ExtractionOfMatrix(PositionRelative2ParentOfJoint[index]);
        Link templlink(linkName,iDofLink,iDofSister,iDofChild,iDofMother,angleOfJoint,localAxisVectorOfJoint,positionRelative2ParentOfJoint);
        Links.append(templlink);
        MapingName2ID.insert(Links[index].GetName(),Links[index].GetID());
        MapingID2Name.insert(Links[index].GetID(),Links[index].GetName());
        //qDebug()<<Links[index].GetName();
    }

    return Links;
}



MatrixXd Robot::Rodrigues(MatrixXd omega,double angle)
{
    MatrixXd rotation;
    double normOfOmega=omega.norm();

    if (normOfOmega<std::numeric_limits<double>::epsilon()) {

        rotation= MatrixXd::Identity(3,3);
    } else {
        MatrixXd NormalizedAxisRotation=omega/normOfOmega;;
        double AmountOfRotation=normOfOmega*angle;
        Matrix3d SkewSymMatrixOfVector;
        SkewSymMatrixOfVector<<0,
                -NormalizedAxisRotation(2),
                NormalizedAxisRotation(1),
                NormalizedAxisRotation(2),
                0,
                -NormalizedAxisRotation(0),
                -NormalizedAxisRotation(1),
                NormalizedAxisRotation(0),
                0;
        rotation=MatrixXd::Identity(3,3)+SkewSymMatrixOfVector*qSin(AmountOfRotation)+(SkewSymMatrixOfVector*SkewSymMatrixOfVector)*(1-qCos(AmountOfRotation));
    }
    return rotation;
}



void Robot::ForwardKinematic(int input)
{
    if (input==0) {
        return;
    }

    if (input!=1) {
        MatrixXd positionInWorldCoordinate=Links[Links[input-1].GetMotherID()-1].AttitudeInWorldCoordinate*Links[input-1].GetJointPositionRelative2Parent()+Links[Links[input-1].GetMotherID()-1].PositionInWorldCoordinate;
        MatrixXd attitudeInWorldCoordinate=Links[Links[input-1].GetMotherID()-1].AttitudeInWorldCoordinate*Rodrigues(Links[input-1].GetJointAxisVectorLocal(),Links[input-1].JointAngle);

        Links[input-1].AttitudeInWorldCoordinate=attitudeInWorldCoordinate;
        Links[input-1].PositionInWorldCoordinate=positionInWorldCoordinate;

    }  else {
        QList <QByteArray> PositionInWorldCoordinate= GetContentOfRobot("PositionInWorldCoordinate",content);

        MatrixXd positionInWorldCoordinate= ExtractionOfMatrix( PositionInWorldCoordinate[input-1]);
        QList <QByteArray> AttitudeInWorldCoordinate= GetContentOfRobot("AttitudeInWorldCoordinate",content);
        MatrixXd attitudeInWorldCoordinate= ExtractionOfMatrix( AttitudeInWorldCoordinate[input-1]);
        Links[input-1].AttitudeInWorldCoordinate=attitudeInWorldCoordinate;
        Links[input-1].PositionInWorldCoordinate=positionInWorldCoordinate;
    }
    ForwardKinematic( Links[input-1].GetChildID());
    ForwardKinematic( Links[input-1].GetSisterID());

}





void Robot::SetJointAngle(MatrixXd jointAngle, MatrixXi Route){

    for (int var = 0; var < Route.cols(); var++) {
        int jointNumber;
        jointNumber=Route(0,var);
        Links[jointNumber-1].JointAngle=jointAngle(0,var);
    }
    ForwardKinematic(1);
}


void Robot::MoveJoints(MatrixXi route, MatrixXd deltaJointAngle)
{int index;
    for (int var = 0; var < route.cols(); var++) {

        index=route(var)-1;
        Links[index].JointAngle=Links[index].JointAngle+deltaJointAngle(index-1);

    }
   //  qDebug()<<"milad";
}



MatrixXi Robot::FindRoutfromRoot2Destination(QString Destination)
{
    int to= MapingName2ID.value(Destination);
    MatrixXi toMatrix(1,1);
    toMatrix.fill(0);
    toMatrix<<to;
    int mother=Links[to-1].GetMotherID();
    if (mother==1)
        return toMatrix;

    else
    {
        MatrixXi _route=FindRoutfromRoot2Destination(MapingID2Name.value(mother));
        _route.conservativeResize(NoChange,_route.cols()+1);
        _route.block(0,_route.cols()-1,1,1) = toMatrix;
        return _route;

    }

}




Vector3d Robot::Rot2omega(Matrix3d rotation)
{
    Vector3d omega;
    Vector3d el;
    Vector3d temp;
    omega.fill(0);
    el.fill(0);
    temp.fill(0);
    //    omega.resize(3,1);
    //    el.resize(3,1);
    //    temp.resize(3,1);
    el<< rotation(2,1)-rotation(1,2),rotation(0,2)-rotation(2,0),rotation(1,0)-rotation(0,1);
    double norm_el= el.norm();
    if (norm_el>std::numeric_limits<double>::epsilon()) {
        omega<<(qAtan2(norm_el,rotation.trace()-1)/norm_el *el);
    }

    else if (rotation(0,0)>0 && rotation(1,1)>0 && rotation(2,2)>0) {
        omega<<0,0,0;
    }

    else {
        temp<<rotation(0,0)+1,rotation(1,1)+1,rotation(2,2)+1;
        omega<<(M_PI/2)*temp;
    }


    return omega;


}


MatrixXd Robot::CalcTaskSpaceError(Link Target,QString current)
{
    Vector3d PositionError;
    PositionError.fill(0);
    Matrix3d RotationError;
    RotationError.fill(0);
    Vector3d AttitudeError;
    AttitudeError.fill(0);
    MatrixXd TaskSpaceError;
    TaskSpaceError.resize(6,1);
    TaskSpaceError.fill(0);
    PositionError=Target.PositionInWorldCoordinate-Links[MapingName2ID.value(current)-1].PositionInWorldCoordinate;
    RotationError=Links[MapingName2ID.value(current)-1].AttitudeInWorldCoordinate.transpose()*Target.AttitudeInWorldCoordinate;
    AttitudeError=Links[MapingName2ID.value(current)-1].AttitudeInWorldCoordinate*Rot2omega(RotationError);


    TaskSpaceError<<PositionError,AttitudeError;

    return TaskSpaceError;
}

MatrixXd Robot::CalcJacobian(MatrixXi route )
{
    MatrixXd Jacobian;
    Vector3d target;
        Jacobian.fill(0);
        target.fill(0);
    Vector3d JointAxisInWorldFrame;
    JointAxisInWorldFrame.fill(0);
    int JacobianColumnSize;
    JacobianColumnSize=0;
    target=Links[route(route.cols()-1)-1].PositionInWorldCoordinate;
    JacobianColumnSize=route.cols();
    Jacobian= MatrixXd::Zero(6,JacobianColumnSize);
    for (int var = 0; var < JacobianColumnSize; var++) {
        //note var starts from 0
        int mother=Links[route(var)-1].GetMotherID();
        JointAxisInWorldFrame=Links[mother-1].AttitudeInWorldCoordinate*Links[route(var)-1].GetJointAxisVectorLocal();
        Jacobian.col(var)<<JointAxisInWorldFrame.cross(target-Links[route(var)-1].PositionInWorldCoordinate),JointAxisInWorldFrame;
    }

    return Jacobian;
}




double Robot::IKLevenbergMarquardt(Link Target,QString current)
{
    MatrixXd error;
    error.fill(0);
    MatrixXd gerr;
    gerr.fill(0);
    MatrixXd dq;
    dq.fill(0);
    MatrixXd Jh;
    Jh.fill(0);
    MatrixXd J;
    J.fill(0);
    MatrixXd EK2;
    EK2.fill(0);
    MatrixXd EK;
    double Ek=0;
    double Ek2=0;
    MatrixXi route=FindRoutfromRoot2Destination(current);
    double wn_position =1/0.3;
    double wn_angle = 1/(2*M_PI);
    MatrixXd we(6,6);
    we.fill(0);
    MatrixXd wn;
    we.diagonal()<<wn_position,wn_position,wn_position,wn_angle,wn_angle,wn_angle;
    wn=MatrixXd::Identity(route.cols(),route.cols());
    ForwardKinematic(1);
    error=CalcTaskSpaceError(Target,current);
    EK=error.transpose()*we*error;
    Ek=EK(0);
    for (int var = 0; var < 10; ++var) {
        J=CalcJacobian(route);

        Jh=J.transpose()*we*J+wn*(Ek+0.02);
        gerr=J.transpose()*we*error;
        dq=Jh.lu().solve(gerr);
        MoveJoints(route,dq);
        ForwardKinematic(1);
        error=CalcTaskSpaceError(Target,current);
        EK2=error.transpose()*we*error;
        Ek2=EK2(0);
        if (Ek2<(1e-12)) {
            return error.norm();
        } else if (Ek2 < Ek) {
            Ek = Ek2;
        }

        else {
            MoveJoints(route,-dq);
            ForwardKinematic(1);
            return error.norm();
        }

    }

    return error.norm();

}
