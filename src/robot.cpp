#include "robot.h"

Robot::Robot(QObject *parent) : QObject(parent)
{
    QString address="..//src//RobotData//test.txt";
    if (!FileExists(address))
    {
        qWarning()<<"Invalid Robot data Path:"<<address;
        return;
    }
    QFile file(address);
    file.open(QFile::ReadWrite);
    QByteArray content = file.readAll();
    QList<QByteArray> values= GetContentOfRobot("AngularVelocityInWorldCoordinate",content);
    QList <Link> milad=CreateRobotLinks( content );
    //    foreach (QByteArray v, values) {
    //        qDebug()<<"input method="<<v;
    //        MatrixXd mat= ExtractionOfMatrix(v);
    //        //qDebug()<<Matrix2;
    //        MatrixXd gg=Rodrigues(mat,20);
    //        //qDebug()<<gg;
    //    }
    ForwardKinematic(1,content);

    file.close();

}

QList<Link> Robot::GetLinks()
{
    return Links;
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

MatrixXd Robot::Rodrigues(MatrixXd omega,double angle)
{
    MatrixXd rotation;
    double normOfOmega=omega.norm();

    if (!normOfOmega<std::numeric_limits<double>::epsilon()) {

        rotation= MatrixXd::Identity(3,3);
    } else {
        MatrixXd NormalizedAxisRotation=omega/normOfOmega;
        double AmountOfRotation=normOfOmega*angle;
        MatrixXd SkewSymMatrixOfVector;
        SkewSymMatrixOfVector<<0, -NormalizedAxisRotation(3), NormalizedAxisRotation(2),
                NormalizedAxisRotation(3), 0, -NormalizedAxisRotation(1)
                -NormalizedAxisRotation(2),  NormalizedAxisRotation(1), 0;
        rotation=MatrixXd::Identity(3,3)+SkewSymMatrixOfVector*qSin(AmountOfRotation)+(SkewSymMatrixOfVector*SkewSymMatrixOfVector)*(1-qCos(AmountOfRotation));
    }
    return rotation;
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
        MappingName2ID.insert(Links[index].GetName(),Links[index].GetID());
        qDebug()<<Links[index].GetName();
    }

    return Links;
}


void Robot::ForwardKinematic(int input,QByteArray content)
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
    // qDebug()<<input-1;
    //qDebug()<<Links[input-1].GetID();
    //    qDebug()<<Links[input-1].GetChildID();
    //    qDebug()<<Links[input-1].GetSisterID();
    ForwardKinematic( Links[input-1].GetChildID(), content);
    ForwardKinematic( Links[input-1].GetSisterID(), content);

}
