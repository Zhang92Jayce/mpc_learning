#include "control.h"
#include "tf/tf.h"
#include "sys/time.h"
#include "ar_msgs/control.h"
#include "ar_topic/common_topic.h"
#include "tf/transform_datatypes.h"
namespace robot {
namespace ppcControl {

PathFollowControl::PathFollowControl():m_nhPrivate("~")
{
//加载参数
    m_nhPrivate.param("forward_len", m_config.forwardLen, 4.0);
    m_nhPrivate.param("forward_len_on_line", m_config.forwardLenOnLine, 4.0);
    m_nhPrivate.param("fb_gain", m_config.fbGain, 1.0);
    m_nhPrivate.param("fb_gain_on_line", m_config.fbGainOnLine, 1.0);
    m_nhPrivate.param("turningK", m_turningK, 1.0);
    m_nhPrivate.param("kd", m_config.kd, 0.3);
    m_nhPrivate.param("ki", m_config.ki, 0.05);
    m_nhPrivate.param("kp", m_config.kp, 1.0);
    m_nhPrivate.param("save_weight", m_config.saveWeight, true);
    m_nhPrivate.param("controlDt", m_controlDt, false);
    m_nhPrivate.param("slowDownDistMS", m_slowDownDistMS, 4.0);

    m_nhPrivate.param("kd_on_line", m_config.kdOnLine, 0.3);
    m_nhPrivate.param("ki_on_line", m_config.kiOnLine, 0.05);
    m_nhPrivate.param("kp_on_line", m_config.kpOnLine, 1.0);
    m_nhPrivate.param("y_dist", m_yDist, 10.0);
    m_nhPrivate.param("x_dist", m_dist, 10.0);
    m_nhPrivate.param("on_line_dist", m_config.onLineDist, 0.1);
    m_nhPrivate.param("on_line_theta", m_config.onLineTheta, 0.2);
    m_nhPrivate.param("kt", m_config.kt, 0.05);
    m_nhPrivate.param("integral_threshold", m_config.integralThreshold, 0.1);
    m_nhPrivate.param("saturation", m_config.saturation, 10.0);
    m_nhPrivate.param("slope", m_config.slope, 2.0);
    m_nhPrivate.param("slope_on_line", m_config.slopeOnLine, 1.5);
    m_nhPrivate.param("tau", m_config.tau, 0.4);
    m_nhPrivate.param("vel_low", m_config.velLow, 0.3);
    m_nhPrivate.param("vel_up", m_config.velUp, 10.0);
    m_nhPrivate.param("wp", m_config.wp, 1.0);
    m_nhPrivate.param("max_speed", m_maxSpeed, 1.0);
    m_nhPrivate.param("backward_speed", m_config.backwardSpeed, 1.0);
    m_nhPrivate.param("wheel_distanceB", m_config.wheelDistanceB, 2.0);
    m_nhPrivate.param("wheel_distanceF", m_config.wheelDistanceF, 1.5);
    m_nhPrivate.param("wheel_base", m_config.wheelBase, 1.0);
    m_nhPrivate.param("hz", m_hz, 10.0);
    m_nhPrivate.param("accuration", m_accuration, 0.05);
    m_nhPrivate.param("prepare_to_stop", m_prepareToStop, 2.0);
    m_nhPrivate.param("min_start_vel", m_minStartVel, 0.2);
    m_nhPrivate.param("slow_down_extra_dist", m_config.slowDownExtraDist, 1.0);
    m_nhPrivate.param("back_weight", m_config.backWeight, 0.0);
    m_nhPrivate.param("min_angle", m_config.minAngle, 10.0);
    m_nhPrivate.param("min_dist", m_config.minDist, 0.1);

    m_nhPrivate.param("origin_x", m_originX, 0.0);
    m_nhPrivate.param("origin_y", m_originY, 0.0);

    m_nhPrivate.param("e_zone", m_config.eZone, 0.03);
    m_nhPrivate.param("ec_zone", m_config.ecZone, 0.15);
    m_nhPrivate.param("max_PW", m_config.maxPW, 0.0);
    m_nhPrivate.param("max_delta_angle", m_config.maxDeltaAngel, 10.0);
    m_nhPrivate.param("max_delta_angle2", m_config.maxDeltaAngel2, 20.0);
    m_nhPrivate.param("pose_weight", m_config.posWeight, 0.5);
    m_nhPrivate.param("start_vel", m_config.startVel, 0.4);
    m_nhPrivate.param("ab_mode", m_abMode, false);
    if(m_abMode)
        m_start = true;
    m_nhPrivate.param("ka", m_config.ka, 0.0);
    m_nhPrivate.param("kat", m_config.kat, 0.0);
    m_nhPrivate.param("smp_kp", m_smpKp, 1.0);
    m_nhPrivate.param("smp_kd", m_smpKd, 0.3);
    m_nhPrivate.param("smp_ki", m_smpKi, 0.1);
    m_nhPrivate.param("lon_kp_t", m_lonKpT, 1.0);
    m_nhPrivate.param("lon_kd_t", m_lonKdT, 0.3);
    m_nhPrivate.param("lon_ki_t", m_lonKiT, 0.1);
    m_nhPrivate.param("jerk", m_jerk, 3.0);
    m_nhPrivate.param("dj", m_dj, 0.1);

    m_nhPrivate.param("start_clutch", m_startClutch, 30);
    m_nhPrivate.param("start_brake", m_startBrake, 30);
    m_nhPrivate.param("slow_down_cur", m_config.slowDownCurvature, 0.05);
    m_nhPrivate.param("slow_down_clutch", m_slowDownClutch, 30);
    m_nhPrivate.param("slow_down_brake", m_slowDownBrake, 30);
    m_nhPrivate.param("slow_down_dist", m_config.slowDownDist, 4.0);
    m_nhPrivate.param("slow_down_rate", m_config.slowDownRate, 0.5);
    m_nhPrivate.param("max_acc", m_config.maxAcc, 2.0);
    m_nhPrivate.param("max_dec", m_config.maxDec, 4.0);
    m_nhPrivate.param("delta_angle", m_deltaAngle, 0.0);
    m_nhPrivate.param("lon_accuration", m_lonAccuration, 0.3);
    m_nhPrivate.param("max_error", m_maxError, 100.0);
    m_nhPrivate.param("master_mode", m_masterMode, false);
    m_nhPrivate.param("backward_mode", m_backwardMode, false);
    m_nhPrivate.param("steeringDeltaAngle", m_steeringDeltaAngle, 0.0);
    m_nhPrivate.param("accuracy_thredhold", m_config.accuracyThredhold, 0.03);
    m_nhPrivate.param("outLineDist", m_config.outLineDist, 8.0);
    double r = 0.0;
    m_nhPrivate.param("min_radius", r, 5.0);
    m_config.maxCurvature = 0.9/r;
    m_config.minAngle = m_config.minAngle*M_PI/180.0;
    m_nhPrivate.param("calculate_accuracy", m_config.calculateAccuracy, true);
    m_nhPrivate.param("simulation", m_sim, true);
    m_pathController.reset(new ControllerPathFollow(m_config, m_hz));
    m_poseSub = m_nh.subscribe(FLAGS_localization_odometry, 1, &PathFollowControl::poseCallBack, this);
   // pose_sub_ = nh_.subscribe("/current_pose", 1, &PathFollowControl::poseCallBack, this);
    m_velPub = m_nh.advertise<std_msgs::Float32>("/control/velocity", 1);
    m_pathSub = m_nh.subscribe(FLAGS_planning_local_trajectory, 8, &PathFollowControl::pathCallBack, this);

   // m_pathSub = m_nh.subscribe("/path_test1", 8, &PathFollowControl::pathCallBack, this);
   // m_pathProtoSub = m_nh.subscribe("/pathProto", 8, &PathFollowControl::pathProtoCallBack, this);
    m_cmdPub = m_nh.advertise<geometry_msgs::Twist>(FLAGS_control_cmd_vel, 1);
   // cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
   // m_alarmPub = m_nh.advertise<bgy_ar_msg::control_alarm_info>(FLAGS_control_alarm_info, 1);
    m_statusPub = m_nh.advertise<ar_msgs::control>(FLAGS_control_basic_info, 1);
    m_statusMsPub = m_nh.advertise<control::control_basic>("control_basic_ms", 1);
    m_lonPub = m_nh.advertise<ar_msgs::chassis_control>(FLAGS_control_chassis_control, 1);
   // m_actionSub = m_nh.subscribe(FLAGS_task_control_control_cmd, 8, &PathFollowControl::actionCallBack, this);
   // m_actionSub = m_nh.subscribe("action", 8, &PathFollowControl::actionCallBack, this);
    m_actionSub = m_nh.subscribe(FLAGS_planning_chassis_control, 8, &PathFollowControl::actionMCallBack, this);
  //  m_actionSub2 = m_nh.subscribe("action2", 8, &PathFollowControl::action2CallBack, this);
  //  m_masterVelSub = m_nh.subscribe("master_vel", 1, &PathFollowControl::masterVelCallBack, this);
    m_masterPoseSub = m_nh.subscribe("planning/ms_pose", 1, &PathFollowControl::masterPoseCallBack, this);
   // m_imuSub = m_nh.subscribe("imu", 1, &PathFollowControl::imuCallBack, this);
   // m_pathProtoPub = m_nh.advertise<std_msgs::String>("pathProto", 1);
  //  m_start = false;
    m_autoMode = true;

    m_gpsStatus = sensor_msgs::NavSatStatus::STATUS_FIX;
    m_deltaT = 1.0/m_hz;
    // m_lonPID.reset(new ControllerPid(m_lonKp, m_lonKi, m_lonKd, m_config.tau, 1.0/m_hz,
    //                                   m_config.integralThreshold, m_config.saturation));
    m_lonPIDThrottle.reset(new ControllerPid(m_lonKpT, m_lonKiT, m_lonKdT, m_config.tau, m_deltaT,
                                              m_config.integralThreshold, m_config.saturation));
    m_slavePosePID.reset(new ControllerPid(m_smpKp, m_smpKi, m_smpKd, m_config.tau, m_deltaT,
                                              m_config.integralThreshold, m_config.saturation));

    m_slaveVelPID.reset(new ControllerPid(m_smvKp, m_smvKi, m_smvKd, m_config.tau, m_deltaT,
                                              m_config.integralThreshold, m_config.saturation));

    m_lastClutch = m_startClutch;
    m_lastBrake = m_startBrake;


    std::cout << "control_node is ready!" << std::endl;
}

void PathFollowControl::actionMCallBack(const ar_msgs::chassis_control& msg)
{
    if(msg.chassisStateControl == ar_msgs::chassis_control::CHASSIS_FORWARD
       || msg.chassisStateControl == ar_msgs::chassis_control::CHASSIS_BACK)
    {
        if(m_start)
            return;
        if(m_autoMode)
           m_start = true;
        m_getOnLine = false;
        m_getOnLineOut = false;
        m_finish = false;
        m_stop = false;
      //  m_lastThrottle = 0.0;
        m_lastClutch = m_startClutch;
        m_lastBrake = m_startBrake;
    }
    if(msg.chassisStateControl == ar_msgs::chassis_control::CHASSIS_STOP ||
       msg.chassisStateControl == ar_msgs::chassis_control::CHASSIS_STOP_PREPARE)
    {
        if(!m_start)
            return;
       // std::cout << "get 0" << std::endl;
        m_start = false;
        m_getOnLine = false;
        m_getOnLineOut = false;
       // m_throthTemp = m_lastThrottle;
       // m_lastThrottle = 0.0;
        m_lastClutch = m_startClutch;
        m_lastBrake = m_startBrake;
        m_pathController->reset();
    }
    /*
    if(msg.data == 2)
        m_autoMode = false;
    if(msg.data == 3)
    {
        m_autoMode = true;
        m_getOnLine = false;
        m_start = false;
        m_finish = false;
        m_lastClutch = m_startClutch;
        m_lastBrake = m_startBrake;
    }
    if(msg.data == 4)
        m_slaveMode = true;
    if(msg.data == 5)
        m_slaveMode = false;
*/
    std::cout << "action commond: " << msg.chassisStateControl << std::endl;
  //  Console_Warn(" commond: %d", int(msg.data));
}

void PathFollowControl::masterPoseCallBack(const control::ms_pose &msg)
{
    boost::unique_lock<boost::recursive_mutex> lock(m_masterPoseMutex);
    if(fabs(msg.x) > 0.1 && fabs(msg.y) > 0.1)
    {
      m_masterPose.x = msg.x - m_originX;
      m_masterPose.y = msg.y - m_originY;
    }
    m_msYaw = msg.yaw;

    m_slaveMode = msg.follow;

    if(m_slaveMode)
    {
       if(msg.v < 0.9)
           m_masterVel = msg.v;
    }
    else
       m_masterVel = msg.v;
    lock.unlock();
    //std::cout << "ms_pose_x: " << m_masterPose.x << " ms_pose_y: " << m_masterPose.y << " slave mode: "
    //          << m_slaveMode << std::endl;
    m_getMasterPose = true;
}


//车体坐标的回调函数，此处是控制入口
void PathFollowControl::poseCallBack(const nav_msgs::Odometry& msg)
{
   // timeval start_time, end_time;
   // gettimeofday(&start_time, NULL);

    geometry_msgs::Twist cmd_vel;
    ControlCommand command;
    ar_msgs::control basicInfo;
    control::control_basic msBasicInfo;
    ar_msgs::chassis_control msgt;
    double deltaDist = 0.0;
    double deltaDistY = 0.0;
    double maxSpeed = 1.0;
    double deltaVel = 0.0;
    double deltaYaw = 0.0;
    double goalS = 0.0;
    boost::unique_lock<boost::recursive_mutex> lock(m_pathMutex);
    geometry::Path path = m_path;
    maxSpeed = m_maxSpeed;
    goalS = m_goalS;
    lock.unlock();

    if(path.size() < 2)
        return;
    m_num++;
    double vel = 0.0;
    msgt.cmd_type = ar_msgs::chassis_control::CMD_NULL;
    double err = 0.0;
    double poseErr = 0.0;
    double angleErr = 0.0;
    m_robotState.position.x = msg.pose.pose.position.x - m_originX;
    m_robotState.position.y = msg.pose.pose.position.y - m_originY;
    m_robotState.goalS = goalS;
    m_RobotVel = msg.twist.twist.linear.x;
    double roll, pitch, yaw;
    tf::Quaternion qua;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, qua);
    tf::Matrix3x3(qua).getRPY(roll, pitch, yaw);
 /*   if(m_backwardMode)
    {
        yaw += M_PI;
        yaw = geometry::NormalizeAngle(yaw);
    }*/
    m_robotState.yaw = yaw - m_deltaAngle;
    m_robotState.roll = roll;
    m_robotState.pitch = pitch;
    boost::unique_lock<boost::recursive_mutex> lockImu(m_imuMutex);
    m_robotState.gotImu = m_gotImu;
    m_robotState.angularSpeedPitch = m_imuMsg.angular_velocity.y;
    m_robotState.angularSpeedRoll = m_imuMsg.angular_velocity.x;
    m_robotState.angularSpeedYaw = m_imuMsg.angular_velocity.z;
    lockImu.unlock();
    m_poses.push_back(m_robotState.position);
    m_pitches.push_back(pitch);
    m_stop = false;
    if(m_poses.size() > 5){
        m_poses.pop_front();
        m_pitches.pop_front();
    }

    if(m_poses.size() > 4 && m_pitches.size() > 4)
    {
       // double vel = (m_poses.back() - m_poses.front()).length() * m_hz/(m_poses.size() - 1);
        double velAngle  = (m_poses.back() - m_poses.front()).heading();
        double deltaA = velAngle - m_robotState.yaw;
        double deltaS = 0.0;
        for(size_t i = 0; i < m_poses.size() - 1; i++)
        {
            deltaS += (m_poses[i + 1] - m_poses[i]).length()/cos(m_pitches[i]);
        }
        vel = deltaS * m_hz/double(m_poses.size() - 1);

        if(deltaA > M_PI)
            deltaA -= 2.0 * M_PI;
        if(deltaA < -M_PI)
            deltaA += 2.0 * M_PI;
        if(fabs(deltaA) > M_PI * 0.5)
            vel = -vel;
       // if(m_backwardMode)
        //    vel = -vel;

        msBasicInfo.ms_v = vel;
       // vel = m_RobotVel;
        //if(fabs(vel) > 0.2)
        {
        std_msgs::Float32 vMsg;
        vMsg.data = vel;
        //msBasicInfo.ms_v = vel;
        m_velPub.publish(vMsg);
        }
    }


    boost::unique_lock<boost::recursive_mutex> lock1(m_masterPoseMutex);
    geometry::Vec2d masterPose = m_masterPose;
    bool slaveMode = m_slaveMode;
    double msV = m_masterVel;
    lock1.unlock();

    geometry::Vec2d pathVector = path[path.size() - 1].position
                                 - path[path.size() - 3].position;
    if(m_masterMode)
    {
        pathVector = path[0].position - path[2].position;
    }
    if(m_masterMode || slaveMode){
    geometry::Vec2d msVec = masterPose - m_robotState.position;
    deltaDist = msVec.dot(pathVector)/pathVector.length() - m_dist;
    deltaDistY = fabs(msVec.cross(pathVector)/pathVector.length()) - m_yDist;
    msBasicInfo.control_devy = deltaDistY;
    msBasicInfo.control_devx = deltaDist;
    msBasicInfo.ms_v = m_RobotVel;
    if(!m_getMasterPose)
    {
        msBasicInfo.control_devy = 999.9;
        msBasicInfo.control_devx = 999.9;
    }
    }



//开始信号触发后进入的主要控制循环
    if(!m_finish && m_start && !m_pause && m_autoMode)
    {
        m_startDelay++;
        geometry::Vec2d dist = m_goal.position - m_robotState.position;
        double delta = dist.length();
        double speed = maxSpeed;

        if(delta < 8.5)
        {
            if(delta < 2.5)
                speed = std::max(0.5, delta * maxSpeed/2.0);
          //  geometry::Vec2d endRobot = m_robotState.position - m_goal.position;
         //   double distL = m_endVec.dot(endRobot)/m_endVec.length();
         //   std::cout << "dist: " << distL << std::endl;
            if(delta < m_prepareToStop)
            {
                std::cout << "prepare to stop!" << std::endl;
                m_stop = true;
            }
            if(delta < m_accuration)
            {
                m_finish = true;
               // m_throthTemp = m_lastThrottle;
               // m_lastThrottle = 0.0;
                std::cout << "finished!" << std::endl;
            }
        }
        bool finished = false;
        if(!m_pathController->setControl(&path, m_robotState, &command, speed, delta, &err,
                                         &poseErr, &angleErr, &m_getOnLine, m_getOnLineOut, &finished, m_reverse))
        {
            command.rightVel = 0.0;
            command.leftVel = 0.0;
            command.angleVel = 0.0;
            command.linearVel = 0.0;
            m_error = true;
        }
        else
        {
            m_error = false;
        }

        if(finished)
        {
            m_finish = finished;
        }

       // cmd_vel.angular.z = (command.rightVel - command.leftVel)/m_config.wheelDistance;
       // cmd_vel.linear.x = (command.rightVel + command.leftVel)/2;
        cmd_vel.angular.z = command.angleVel;
        cmd_vel.linear.x = command.linearVel;

        msgt.vir_steer_angle = m_turningK * (command.frontAngle + m_steeringDeltaAngle);
        msgt.vir_steer_angle = geometry::clamp(msgt.vir_steer_angle, -100.0f, 100.0f);
        msgt.left_steer_angle = command.leftFrontAngle + m_steeringDeltaAngle;
        msgt.right_steer_angle = command.rightFrontAngle + m_steeringDeltaAngle;
        msgt.linear_velocity_x = command.linearVel;
        msgt.angular_velocity_z = command.angleVel;

       /* if(m_backwardMode){
            cmd_vel.linear.x = -cmd_vel.linear.x;
            msgt.linear_velocity_x = -msgt.linear_velocity_x;
        }*/

        if(fabs(poseErr) < m_config.onLineDist && fabs(poseErr) > 0.001
           && fabs(angleErr) < m_config.onLineTheta && fabs(angleErr) > 0.001){
            m_getOnLine = true;
            m_getOnLineOut = true;
        }

      //  m_num++;
        //if(m_num%10 == 0)
        {
            basicInfo.control_dev = poseErr;


            if(m_getOnLineOut)
            {
                basicInfo.online_state = 1;
               // msBasicInfo.online_state = 2;
            }
           // m_statusPub.publish(basicInfo);
        }

        if(m_poses.size() > 4 && m_pitches.size() > 4)
        {
           // double vel = (m_poses.back() - m_poses.front()).length() * m_hz/(m_poses.size() - 1);
         /*   double velAngle  = (m_poses.back() - m_poses.front()).heading();
            double deltaA = velAngle - m_robotState.yaw;
            double deltaS = 0.0;
            for(size_t i = 0; i < m_poses.size() - 1; i++)
            {
                deltaS += (m_poses[i + 1] - m_poses[i]).length()/cos(m_pitches[i]);
            }
            vel = deltaS * m_hz/double(m_poses.size() - 1);

            if(deltaA > M_PI)
                deltaA -= 2.0 * M_PI;
            if(deltaA < -M_PI)
                deltaA += 2.0 * M_PI;
            if(fabs(deltaA) > M_PI * 0.5)
                vel = -vel;
           // if(m_backwardMode)
            //    vel = -vel;

            msBasicInfo.ms_v = vel;
           // vel = m_RobotVel;
            //if(fabs(vel) > 0.2)
            {
            std_msgs::Float32 vMsg;
            vMsg.data = vel;
            //msBasicInfo.ms_v = vel;
            m_velPub.publish(vMsg);
            }
            */




            double jerk = 0.0;
            if(fabs(vel) < 0.3)
            {
                jerk = m_jerk - m_dj;
            }
            else
            {
                jerk = m_jerk;
            }

//上线之前纵向控制
            if(!m_getOnLineOut)
            {
                if(fabs(vel) < m_minStartVel){
                m_lastBrake = m_lastBrake - m_deltaClutch;
                m_lastClutch = m_lastClutch - m_deltaClutch;
                cmd_vel.angular.z = 0.0;
                command.frontAngle = 0.0;
                }

                if(fabs(vel) >= m_minStartVel && fabs(vel) <= m_config.startVel){
                m_lastBrake = m_lastBrake - m_deltaClutch;
                m_lastClutch = m_lastClutch - m_deltaClutch;
                }

                m_lastBrake = std::max(m_lastBrake, 0.0);
                m_lastClutch = std::max(m_lastClutch, 0.0);
                msgt.brake = std::round(m_lastBrake);
                msgt.clutch = std::round(m_lastClutch);


              //  msgt.vir_steer_angle = m_turningK * (command.frontAngle + m_steeringDeltaAngle);
              //  msgt.vir_steer_angle = geometry::clamp(msgt.vir_steer_angle, -100.0f, 100.0f);
                msgt.throttle = 0;

                double throttle = 0.0;

                if(!m_controlDt){
                throttle = m_lonPIDThrottle->setControl(fabs(cmd_vel.linear.x) - fabs(vel), m_deltaT);
                double deltaThrottle = geometry::clamp(throttle - m_lastThrottle, -jerk, jerk);
                throttle = m_lastThrottle + deltaThrottle;
                }
                else {
                double deltaThro = m_lonPIDThrottle->setControl(fabs(cmd_vel.linear.x) - fabs(vel), m_deltaT);
                deltaThro = geometry::clamp(deltaThro, -jerk, jerk);
                throttle = m_lastThrottle + deltaThro;

                }

                throttle = geometry::clamp(throttle, 0.0, 100.0);
                if(msgt.clutch > 2.0)
                    throttle = 0.0;
                m_lastThrottle = throttle;
                msgt.throttle = std::round(m_lastThrottle);
            }
//上线之后的纵向控制
            else
            {
                double throttle;

                if(!slaveMode){
                   if(!m_controlDt){
                   throttle = m_lonPIDThrottle->setControl(fabs(cmd_vel.linear.x) - fabs(vel), m_deltaT);
                   double deltaThrottle = geometry::clamp(throttle - m_lastThrottle, -jerk, jerk);
                   //std::cout << "deltaThro: " << deltaThrottle << std::endl;
                   throttle = m_lastThrottle + deltaThrottle;
                   }
                   else {
                   double deltaThro = m_lonPIDThrottle->setControl(fabs(cmd_vel.linear.x) - fabs(vel), m_deltaT);
                   deltaThro = geometry::clamp(deltaThro, -jerk, jerk);
                   throttle = m_lastThrottle + deltaThro;
                   }
                }
                else
                {
                 /* if(!m_getMasterPose)
                  {
                      std::cout << "no master pose!" << std::endl;
                      m_start = false;
                      return;
                  }*/


                  deltaYaw = geometry::NormalizeAngle(m_robotState.yaw - pathVector.heading());

                  //&& fabs(deltaYaw) < m_config.onLineTheta
                  if(fabs(deltaDistY) < 1.0 && m_getOnLine){


                  if(fabs(deltaDist) < m_lonAccuration)
                    {
                       m_attached = true;
                    }
                  if(fabs(deltaDist) > m_lonAccuration + 0.2)
                    {
                       m_attached = false;
                    }

                  double dv = m_slavePosePID->setControl(deltaDist, m_deltaT);
                  double cmdV = dv + msV;

                  if(deltaDist > 0.2 && deltaDist < m_slowDownDistMS)
                  {
                      cmdV = m_masterVel + 0.3;
                  }

                  cmdV = geometry::clamp(cmdV, m_lastVel - m_config.maxDec, m_lastVel + m_config.maxAcc);



                  cmdV = geometry::clamp(cmdV, 0.1, maxSpeed);
                  m_lastVel = cmdV;


                  //throttle = m_slaveVelPID->setControl(deltaVel, m_deltaT);
                  if(cmd_vel.linear.x > 0.01)
                    {
                       double r = cmd_vel.angular.z/cmd_vel.linear.x;
                       cmd_vel.angular.z = r * cmdV;
                       cmd_vel.linear.x = cmdV;
                    }

                 deltaVel = cmdV - vel;

                 if(!m_controlDt){
                 throttle = m_lonPIDThrottle->setControl(deltaVel, m_deltaT);
                 double deltaThrottle = geometry::clamp(throttle - m_lastThrottle, -jerk, jerk);
                 //std::cout << "deltaThro: " << deltaThrottle << std::endl;
                 throttle = m_lastThrottle + deltaThrottle;
                 }
                 else{
                 double deltaThro = m_lonPIDThrottle->setControl(deltaVel, m_deltaT);
                 deltaThro = geometry::clamp(deltaThro, -jerk, jerk);
                 throttle = m_lastThrottle + deltaThro;
                 }
                 }
                 else
                 {

                   if(!m_controlDt){
                   throttle = m_lonPIDThrottle->setControl(fabs(cmd_vel.linear.x) - fabs(vel), m_deltaT);
                   double deltaThrottle = geometry::clamp(throttle - m_lastThrottle, -jerk, jerk);
                       // std::cout << "deltaThro: " << deltaThrottle << std::endl;
                   throttle = m_lastThrottle + deltaThrottle;
                   }
                   else {
                   double deltaThro = m_lonPIDThrottle->setControl(fabs(cmd_vel.linear.x) - fabs(vel), m_deltaT);
                   deltaThro = geometry::clamp(deltaThro, -jerk, jerk);
                   throttle = m_lastThrottle + deltaThro;
                   }

                 }
                 }


                 throttle = geometry::clamp(throttle, 0.0, 100.0);
                 m_lastThrottle = throttle;
                 msgt.throttle = std::round(m_lastThrottle);
                 msgt.brake = 0;
                 msgt.clutch = 0;
               //  msgt.vir_steer_angle = m_turningK * (command.frontAngle + m_steeringDeltaAngle);
               //  msgt.vir_steer_angle = geometry::clamp(msgt.vir_steer_angle, -100.0f, 100.0f);

                if(m_stop)
                {
                    msgt.brake = m_slowDownBrake;
                    msgt.clutch = m_slowDownClutch;

                    m_lastThrottle -= m_jerk;
                    double throttle = std::max(m_lastThrottle, 0.0);
                    m_lastThrottle = throttle;
                    msgt.throttle = std::round(throttle);
                   // msgt.vir_steer_angle = m_turningK * (command.frontAngle + m_steeringDeltaAngle);
                   // msgt.vir_steer_angle = geometry::clamp(msgt.vir_steer_angle, -100.0f, 100.0f);
                    std::cout << "goal dist: " << delta << std::endl;
                }
                if(m_getOnLineOut && poseErr > m_maxError && !m_abMode)
                {
                   // m_lastThrottle = 0.0;
                    m_pathController->reset();
                    m_start = false;
                }
            }
            std::cout << "vel: " << vel << " vCmd: " << cmd_vel.linear.x
                      << " throttle: " << int(msgt.throttle)
                     // << " lastThrottle: " << m_lastThrottle
                      << " brake: " << int(msgt.brake)
                      << " clutch: " << int(msgt.clutch)
                     // << " delta distx: " << deltaDist
                     // << " delta disty: " << deltaDistY
                     // << " deltaV: " << deltaVel
                      << std::endl;
        }

        m_lastVel = cmd_vel.linear.x;
    }

//如果到达终点或者有停车指令则停止
    if(m_finish == true || !m_start || m_pause)
    {
        if(m_finish)
            std::cout << "finished" << std::endl;
        if(m_finish)
        {
          if(fabs(vel) < 0.2)
            basicInfo.arrive_state = 1;
        }
        //m_statusPub.publish(basicInfo);
     //   m_statusMsPub.publish(msBasicInfo);
       // if(m_gpsStatus != sensor_msgs::NavSatStatus::STATUS_FIX)
        //    Console_Warn("gps status error");
           // std::cout << "gps status error" << std::endl;
        m_lastThrottle -= m_jerk + 1.5;
        m_lastThrottle = std::max(m_lastThrottle, 0.0);


        if(m_lastVel >= 0.0){
            m_lastVel -= m_config.maxDec / m_hz;
            m_lastVel = std::max(m_lastVel, 0.0);
        }
        else
        {
            m_lastVel += m_config.maxDec / m_hz;
            m_lastVel = std::min(m_lastVel, 0.0);
        }

        cmd_vel.angular.z = 0.0;
        cmd_vel.linear.x = m_lastVel;
        msgt.linear_velocity_x = m_lastVel;
        msgt.angular_velocity_z = 0.0;
        msgt.left_steer_angle = m_steeringDeltaAngle;
        msgt.right_steer_angle = m_steeringDeltaAngle;
        msgt.vir_steer_angle = m_steeringDeltaAngle;

        msgt.throttle = std::round(m_lastThrottle);
        msgt.brake = m_slowDownBrake;
        msgt.clutch = m_slowDownClutch;
        if(fabs(cmd_vel.linear.x) < 0.1)
        {
            msgt.brake = 100;
            msgt.clutch = 100;
        }
       // msgt.angle = 0.0f;
        m_pathController->reset();
        if(m_autoMode)
            std::cout << "brake: " << int(msgt.brake) << " clutch: " << int(msgt.clutch)
                      << " throttle: " << int(msgt.throttle) << " cmd_vel: " << m_lastVel << std::endl;
    }
    if(m_num%10 == 0)
    {
        m_statusPub.publish(basicInfo);
    }
    m_statusMsPub.publish(msBasicInfo);

    if(m_autoMode)
    {
       // if(m_reverse)
       //     msgt.throttle = -msgt.throttle;
      //  if(m_error)
       //     msgt.brake = 100;
       // if(m_backwardMode)
        //    msgt.throttle = -msgt.throttle;
      //  Console_Warn(" brake: %d clutch:%d throttle: %d", int(msgt.brake), int(msgt.clutch), int(msgt.throttle));
        if(m_backwardMode)
        {
            msgt.vir_steer_angle = m_turningK * (-command.frontAngle + m_steeringDeltaAngle);
            msgt.vir_steer_angle = geometry::clamp(msgt.vir_steer_angle, -100.0f, 100.0f);
            msgt.left_steer_angle = -command.rightFrontAngle + m_steeringDeltaAngle;
            msgt.right_steer_angle = -command.leftFrontAngle + m_steeringDeltaAngle;
        }
        m_cmdPub.publish(cmd_vel);
        m_lonPub.publish(msgt);
    }

   // gettimeofday(&end_time, NULL);
   // double delta_time = (end_time.tv_sec-start_time.tv_sec)*1000000+end_time.tv_usec-start_time.tv_usec;
   // std::cout << "processing time: " << delta_time << std::endl;
}
//路径数据的回调函数，接受的是ros的标准数据格式
void PathFollowControl::pathCallBack(const ar_msgs::local_trajectory& msg)
{
    if(msg.trajectory_point.size() < 5)
        return;
    m_reverse = false;
    geometry::Path path;
    path.clear();
    double s = 0.0;

 //   geometry::Vec2d pathV1(msg.trajectory_point[1].x - msg.trajectory_point[0].x,
 //                         msg.trajectory_point[1].y - msg.trajectory_point[0].y);
    for(size_t i = 0; i < msg.trajectory_point.size(); i++)
    {
        geometry::PathPoint point;
        point.position.x = msg.trajectory_point[i].x - m_originX;
        point.position.y = msg.trajectory_point[i].y - m_originY;
        point.speed = msg.trajectory_point[0].v;
        point.curvature = msg.trajectory_point[i].c;
        point.s = msg.trajectory_point[i].l;
       // path.push_back(point);
        if(i == 0)
        {
           point.s = 0;
           path.push_back(point);           
        }
        else
        {
            geometry::Vec2d pathV2 = point.position - path.back().position;
            if(pathV2.length() > 0.05)
            {
                point.s = path.back().s + pathV2.length();
                path.push_back(point);
            }

        }
    }
    if(path.size() < 5)
        return;
    geometry::Path pathC;
    for(int i = 0; i < path.size(); i++)
    {
        if(i == 0 || i == path.size() - 1)
        {
           geometry::PathPoint point = path[i];
           pathC.push_back(point);
        }
        else
        {
            geometry::Vec2d vF = path[i].position - path[i - 1].position;
            geometry::Vec2d vB = path[i + 1].position - path[i].position;
            double deltaAngle = geometry::NormalizeAngle(vB.heading() - vF.heading());
            geometry::PathPoint point = path[i];
            point.curvature = deltaAngle/(vB.length());
            pathC.push_back(point);
        }
    }
    if(m_finish)
    {
        if((m_goal.position - pathC.back().position).length() > 0.01)
        {
            m_finish = false;
            m_stop = false;
        }
    }
    m_goal = pathC.back();
    m_endVec = pathC.at(pathC.size() - 3).position - m_goal.position;
    double angle = atan2(pathC[pathC.size() - 1].position.y - pathC[pathC.size() - 3].position.y,
                         pathC[pathC.size() - 1].position.x - pathC[pathC.size() - 3].position.x);
    double sinA = sin(angle);
    double cosA = cos(angle);
    s = pathC.back().s;
    double goalS = s;

    for(int i = 0; i < 80; i++)
    {
        geometry::PathPoint point;
        geometry::PathPoint lastPoint = pathC.back();
        point.position.x = lastPoint.position.x + 0.1 * double(cosA);
        point.position.y = lastPoint.position.y + 0.1 * double(sinA);
        point.speed = msg.trajectory_point[0].v;
        s += 0.1;
        point.s = s;
        pathC.push_back(point);
    }
    if(msg.trajectory_point[0].v < 0.0)
        m_reverse = true;
   /* if(m_backwardMode)
    {
        if(msg.trajectory_point[0].v < 0.0)
            m_reverse = false;
        else
            m_reverse = true;
    }*/
    boost::unique_lock<boost::recursive_mutex> lock(m_pathMutex);
    m_path = pathC;
    m_goalS = goalS;
    if(m_masterMode)
    {
        m_maxSpeed = msg.trajectory_point[0].v;
    }
    lock.unlock();
}

}

}
