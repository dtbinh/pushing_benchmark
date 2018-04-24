
#include "ABBRobot.h"

using namespace abb::egm;
using namespace tf;
using namespace std;
using Eigen::MatrixXd;

uint32_t GetTickCount(void) 
{
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now))
    return 0;
  return now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

// *********************************
// Create a simple robot message
void CreateSensorMessage(EgmSensor* pSensorMessage,  Vector3d q_pusher)
{
    Eigen::Quaternion<float> q;
    double x = q_pusher(0);
    double y = q_pusher(1);
    double theta = q_pusher(2);
    q = toABBQuaternion(theta);
    // cout<< "*******************"<<endl;
    static unsigned int sequenceNumber = 0;
    EgmHeader* header = new EgmHeader();
    header->set_mtype(EgmHeader_MessageType_MSGTYPE_CORRECTION);
    header->set_seqno(sequenceNumber++);
    header->set_tm(GetTickCount());

    pSensorMessage->set_allocated_header(header);

    EgmCartesian *pc = new EgmCartesian();
    double z = 0.17820;  //point_pusher
//    double z = 0.26172;  //line pusher
    if(x > 0.55) x = 0.55;
    if(x < 0.10) x = 0.10;
    if(y > 0.4) y = 0.4;
    if(y < -0.4) y = -0.4;
    pc->set_x(x*1000);    // convert to robot representation mm
    pc->set_y(y*1000);          
    pc->set_z(z*1000);
    
     EgmQuaternion *pq = new EgmQuaternion();
     pq->set_u0(q.w());   // need to fill in
     pq->set_u1(q.x());
     pq->set_u2(q.y());
     pq->set_u3(q.z());

    EgmPose *pcartesian = new EgmPose();
     pcartesian->set_allocated_orient(pq);
//    pcartesian->set_allocated_euler(pe);
    pcartesian->set_allocated_pos(pc);
    
    EgmPlanned *planned = new EgmPlanned();
    planned->set_allocated_cartesian(pcartesian);

    pSensorMessage->set_allocated_planned(planned);

}

//Create a sensor joints message
void CreateSensorJointsMessage(EgmSensor* pSensorMessage, VectorXd joints)
{
    int size = 6;
    size=size>6? 6:size;

    static unsigned int sequenceNumber = 0;
    EgmHeader* header = new EgmHeader();
    header->set_mtype(EgmHeader_MessageType_MSGTYPE_CORRECTION);
    header->set_seqno(sequenceNumber++);
    header->set_tm(GetTickCount());

    pSensorMessage->set_allocated_header(header);

    EgmJoints* pj = new EgmJoints();
    for(int i =0; i<size; i++)
    {
        pj->add_joints(joints(i));
    }
    EgmPlanned *planned = new EgmPlanned();
    planned->set_allocated_joints(pj);

    pSensorMessage->set_allocated_planned(planned);
}

void CreateSensorMessageEmpty(EgmSensor* pSensorMessage)
{ 
    static unsigned int sequenceNumber = 0;
    EgmHeader* header = new EgmHeader();
    header->set_mtype(EgmHeader_MessageType_MSGTYPE_CORRECTION);
    header->set_seqno(sequenceNumber++);
    header->set_tm(GetTickCount());

    pSensorMessage->set_allocated_header(header);

}

// ************************
void DisplayRobotMessage(EgmRobot *pRobotMessage, Vector3d& q_pusher, VectorXd& joint_states)
{
    double x_robot, y_robot, z_robot;
    double euler_x, euler_y, euler_z;
    double u0,u1, u2, u3;
    if (pRobotMessage->has_header() && pRobotMessage->header().has_seqno() && pRobotMessage->header().has_tm() && pRobotMessage->header().has_mtype()  )
    {
        //printf("SeqNo=%d Tm=%u Type=%d\n", pRobotMessage->header().seqno(), pRobotMessage->header().tm(), pRobotMessage->header().mtype());
        x_robot =  pRobotMessage->feedback().cartesian().pos().x();
        y_robot =  pRobotMessage->feedback().cartesian().pos().y();
        z_robot =  pRobotMessage->feedback().cartesian().pos().z();

        u0 =  pRobotMessage->feedback().cartesian().orient().u0();
        u1 =  pRobotMessage->feedback().cartesian().orient().u1();
        u2 =  pRobotMessage->feedback().cartesian().orient().u2();
        u3 =  pRobotMessage->feedback().cartesian().orient().u3();

        for (int i=0;i<6;i++) {
            joint_states(i) = pRobotMessage->feedback().joints().joints(i);
        }


        Eigen::Quaternion<float> q;
        q.w() = u0;
        q.x() = u1;
        q.y() = u2;
        q.z() = u3;


        q_pusher(0) = x_robot / 1000;
        q_pusher(1) = y_robot / 1000;
        q_pusher(2) = toPlanarAngle(q);

    }
    else
    {
        printf("No header\n");
    }
}

// ****************************
bool getRobotPose(UDPSocket* EGMsock, string& sourceAddress, unsigned short& sourcePort, EgmRobot* pRobotMessage, Vector3d& q_pusher, VectorXd& joint_states)
{
    int recvMsgSize;
    const int MAX_BUFFER = 1400;
    char buffer[MAX_BUFFER];
    try{
        recvMsgSize = EGMsock->recvFrom(buffer, MAX_BUFFER-1, sourceAddress, sourcePort);
        if (recvMsgSize < 0)
        {
            printf("Error receive message\n");
        }
        else {
            //printf("Received %d\n", recvMsgSize);
        }
        // deserialize inbound message
        pRobotMessage->ParseFromArray(buffer, recvMsgSize);
        DisplayRobotMessage(pRobotMessage, q_pusher, joint_states); //Assign tcp position of robot to robot_x, robot_y, robot_z
        return true;
    } catch (SocketException &e) {}
    
    return false;
}
// *****************************
bool getViconPose(Eigen::Vector3d& q_slider, TransformListener& listener){
    tf::StampedTransform obj_pose;
    try{
        listener.lookupTransform("map", "vicon/StainlessSteel/StainlessSteel_rect", ros::Time(0), obj_pose);
        tf::Quaternion q = obj_pose.getRotation();
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        q_slider << obj_pose.getOrigin().getX()- 0.0, obj_pose.getOrigin().getY()+ 0.0, yaw - 0*0.205681;
        return true;
    }
    catch (tf::TransformException ex){
       //ROS_ERROR("%s",ex.what());
    }
    return false;
}

// ****************************8
bool getViconVel(Eigen::Vector3d& dq_slider, TransformListener& listener){
    geometry_msgs::Twist obj_twist;
    try{
        listener.lookupTwist("vicon/StainlessSteel/StainlessSteel", "map", ros::Time(0), ros::Duration(0.5), obj_twist);
        dq_slider << obj_twist.linear.x, obj_twist.linear.y, obj_twist.angular.z;
        return true;
    }
    catch (tf::TransformException ex){
        //ROS_ERROR("%s",ex.what());
    }
    return false;
}

// ****************************8
void initializeVicon(Vector3d &q_slider, tf::TransformListener& listener){
    int tmp = 0;
    //Booleans
    bool has_vicon_pos = false;

    while(!has_vicon_pos && tmp<500)
    {
        tmp++;

        if(getViconPose(q_slider, listener)){
            if (has_vicon_pos){
                cout << " has_vicon_pos " << has_vicon_pos <<  endl;
                cout << " q_slider " << q_slider <<  endl;
            }
            has_vicon_pos = true;
            ros::spinOnce();}
         if (tmp>10000){break;}
         usleep(4e3);
    }
}

// ****************************8
void pauseEGM(robotStruct robot_struct, double _time, VectorXd& _joint_states){
    //(Give time to thread to initialize)-------------------------------------------------------------------
    int upTimeCounter = (int)(_time*1000);
    Vector3d _q_pusher; //dummy variables
    for(int i=0;i<upTimeCounter;i++){
        cout<<"Countdown "<< upTimeCounter - i << endl;
        if(getRobotPose(robot_struct.EGMsock, robot_struct.sourceAddress, robot_struct.sourcePort, robot_struct.pRobotMessage, _q_pusher, _joint_states)){
            CreateSensorMessageEmpty(robot_struct.pSensorMessage);
            robot_struct.pSensorMessage->SerializeToString(&robot_struct.messageBuffer);
            robot_struct.EGMsock->sendTo(robot_struct.messageBuffer.c_str(), robot_struct.messageBuffer.length(), robot_struct.sourceAddress, robot_struct.sourcePort);
        }
        ros::spinOnce();
        usleep(4e3);
    }
}
// ****************************8
 void initializeEGM(robotStruct robot_struct, Vector3d &q_pusher, VectorXd& joint_states){
     int tmp = 0;
     //Booleans
     bool has_robot = false;
     //Loop
     while(!has_robot)
     {
         ++tmp;
         if(getRobotPose(robot_struct.EGMsock, robot_struct.sourceAddress, robot_struct.sourcePort, robot_struct.pRobotMessage, q_pusher, joint_states)){
            has_robot = true;
            CreateSensorMessageEmpty(robot_struct.pSensorMessage);
            robot_struct.pSensorMessage->SerializeToString(&robot_struct.messageBuffer);
            robot_struct.EGMsock->sendTo(robot_struct.messageBuffer.c_str(), robot_struct.messageBuffer.length(), robot_struct.sourceAddress, robot_struct.sourcePort);
         }
         usleep(4e3);
         if (tmp>10000){break;}
     }
    //
 }

// ****************************8
void velocityControlABB(robotStruct robot_struct, Vector3d &q_pusher, Vector3d &twist_pusher, double h){
    q_pusher = q_pusher + h*twist_pusher;
//    cout<<"q_pusher"<<q_pusher<<endl;
    CreateSensorMessage(robot_struct.pSensorMessage, q_pusher);
    robot_struct.pSensorMessage->SerializeToString(&robot_struct.messageBuffer);
    robot_struct.EGMsock->sendTo(robot_struct.messageBuffer.c_str(), robot_struct.messageBuffer.length(), robot_struct.sourceAddress, robot_struct.sourcePort);
}

// ****************************8
void jointControlABB(robotStruct robot_struct, VectorXd &joints){

    CreateSensorJointsMessage(robot_struct.pSensorMessage, joints);
    robot_struct.pSensorMessage->SerializeToString(&robot_struct.messageBuffer);
    robot_struct.EGMsock->sendTo(robot_struct.messageBuffer.c_str(), robot_struct.messageBuffer.length(), robot_struct.sourceAddress, robot_struct.sourcePort);
}

void velocityOffsetABB(Vector3d &q_pusher, Vector3d &twist_pusher, double v_eq, double radius, double d){
    double dtheta;
    double theta;
    Vector2d vici;
    Vector2d vipi;
    Vector2d ripb;
    Vector2d rbpb;
    Matrix2d Cbi;

    theta = q_pusher(2);
    dtheta = v_eq/radius;
    Cbi = Helper::C3_2d(theta);
    rbpb << d,0;
    ripb = Cbi.transpose()*rbpb;
    vipi = twist_pusher.head(2);
    vici = vipi - Helper::cross3d(dtheta, ripb);

    twist_pusher << vici, dtheta;

}

//***************************************
Eigen::Quaternion<float> toABBQuaternion(double _theta)
{
    Matrix3f R0, R1, R2;
    Eigen::Quaternion<float> q;
    //Convert to tool frame
    R0 = AngleAxisf(M_PI, Vector3f::UnitX())
         * AngleAxisf(0,  Vector3f::UnitY())
         * AngleAxisf(M_PI, Vector3f::UnitZ());
    //Rotate about z tool axis by desired theta (what we want to control)
    R1 = AngleAxisf(-_theta, Vector3f::UnitZ());
    R2 = R0*R1;
    //Convert to quaternion
    q = R2;

    return q;
}
// *****************************//***************************************
double  toPlanarAngle(Eigen::Quaternion<float> q)
{
    Matrix3f R0, R1, R2;
    AngleAxisf aa;
    //Convert to tool frame
    R2 = q;
    R0 = AngleAxisf(M_PI, Vector3f::UnitX())
         * AngleAxisf(0,  Vector3f::UnitY())
         * AngleAxisf(M_PI, Vector3f::UnitZ());
    //Rotate about z tool axis by desired theta (what we want to control)
    R1 = R0.transpose()*R2;
    aa = R1;
    double theta;
    double theta_tmp = aa.angle();
    if (aa.axis().z()>0){
        theta = aa.angle()*-1;
    }
    else{
        theta = aa.angle()*1;
    }

    return theta;
}

void publish_joints(VectorXd joint_states, ros::Publisher exec_joint_pub){
    sensor_msgs::JointState msg;
    msg.name.push_back("joint1");
    msg.name.push_back("joint2");
    msg.name.push_back("joint3");
    msg.name.push_back("joint4");
    msg.name.push_back("joint5");
    msg.name.push_back("joint6");

    msg.position.push_back(joint_states(0));
    msg.position.push_back(joint_states(1));
    msg.position.push_back(joint_states(2));
    msg.position.push_back(joint_states(3));
    msg.position.push_back(joint_states(4));
    msg.position.push_back(joint_states(5));

    for (int i=0;i<6;i++) {
        msg.velocity.push_back(i);
        msg.effort.push_back(i);
    }

    msg.header.stamp=ros::Time::now();

    exec_joint_pub.publish(msg);
}


void ikfast_pusher(Vector3d& _q_pusher, VectorXd& joint_states, VectorXd& _q0, bool& is_success, tf::TransformListener& listener){
    double solution[6];
    double pose[7];
    double weight[6]= {1,2,3,2,1,1};
    double q0[6]= {_q0(0),_q0(1),_q0(2),_q0(3),_q0(4),_q0(5)};
    int hassol[0];
    Eigen::Quaternion<float> quat_pusher = toABBQuaternion(_q_pusher(2));

    geometry_msgs::PoseStamped pose_map;
    pose_map.header.frame_id="/origin";
    pose_map.header.stamp = ros::Time(0);
    pose_map.pose.position.x = _q_pusher(0);
    pose_map.pose.position.y = _q_pusher(1);
    pose_map.pose.position.z = .173;
    pose_map.pose.orientation.w = quat_pusher.w();
    pose_map.pose.orientation.x = quat_pusher.x();
    pose_map.pose.orientation.y = quat_pusher.y();
    pose_map.pose.orientation.z = quat_pusher.z();
    geometry_msgs::PoseStamped pose_base_link;
    while (true) {
        try {
            listener.transformPose("base_link", pose_map, pose_base_link);
            break;
        }
        catch (exception &e) {
            cout << e.what() << '\n';
        }
    }
    pose[0] = pose_base_link.pose.position.x;
    pose[1] = pose_base_link.pose.position.y;
    pose[2] = pose_base_link.pose.position.z;
    pose[3] = pose_base_link.pose.orientation.w;
    pose[4] = pose_base_link.pose.orientation.x;
    pose[5] = pose_base_link.pose.orientation.y;
    pose[6] = pose_base_link.pose.orientation.z;

    for (int i; i<6; i++){
        q0[i] = joint_states(i);
    }

    ikfastAndFindBest(solution, pose, weight,  q0,  hassol);

    for (int i; i<6; i++){
        joint_states(i) = solution[i];
    }

    if (hassol[0] == 1){
        is_success = true;
    }
    else{
        is_success = false;
    }
}
