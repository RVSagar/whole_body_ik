#ifndef __HUMANOIDWBC_H__
#define __HUMANOIDWBC_H__
#include <whole_body_ik/pin_wrapper.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <reemc_direct_position_control/JointPositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <whole_body_ik_msgs/HumanoidAction.h>
#include <eigen3/Eigen/Dense>
#include <actionlib/server/simple_action_server.h>

using std::cerr;
using std::endl;
using std::string;

using namespace Eigen;


class humanoid_wbc
{
private:
    /// ROS nodehanlder
    ros::NodeHandle nh;
    ros::Subscriber joint_state_sub, odom_sub;
    std::string modelname, base_link_frame, lfoot_frame, rfoot_frame, lhand_frame, rhand_frame, head_frame, joint_cmd_topic;
    ros::Publisher cmd_pub;
    ros::Publisher joint_command_pub;
    whole_body_ik_msgs::HumanoidResult result_;
    whole_body_ik_msgs::HumanoidFeedback feedback_;
    std::vector<linearTask> ltaskVec;
    std::vector<angularTask> ataskVec;
    std::vector<dofTask> dtaskVec;
    Eigen::VectorXd q,dq, qd, dqd;
    Vector3d pwb, vwb, omegawb;
    Quaterniond qwb;
    pin_wrapper *pin;
    void init();

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ~humanoid_wbc();
    humanoid_wbc(ros::NodeHandle nh_);
    void run();
    void controlCb(Eigen::VectorXd& qd, Eigen::VectorXd& dqd, const whole_body_ik_msgs::HumanoidGoal msg);
    void ActionServercontrolCb(const whole_body_ik_msgs::HumanoidGoalConstPtr msg);
    Vector3d getDesiredLLegPosition();
    Vector3d getDesiredRLegPosition();
    Quaterniond getDesiredLLegOrientation();
    Quaterniond getDesiredRLegOrientation();
    Quaterniond getDesiredLHandOrientation();
    Quaterniond getDesiredRHandOrientation();
    Quaterniond getDesiredHeadOrientation();
    int ndof;

    pin_wrapper  *desired_pin;
    Eigen::VectorXd jointNominalConfig, jointNominalVelocity;
    actionlib::SimpleActionServer<whole_body_ik_msgs::HumanoidAction> *as_; 

    std::vector<std::string> joint_states_list = {
    "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint", "arm_left_5_joint", "arm_left_6_joint", "arm_left_7_joint",
    "arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint", "arm_right_4_joint", "arm_right_5_joint", "arm_right_6_joint", "arm_right_7_joint",
    "head_1_joint", "head_2_joint",
    "leg_left_1_joint", "leg_left_2_joint", "leg_left_3_joint", "leg_left_4_joint", "leg_left_5_joint", "leg_left_6_joint",
    "leg_right_1_joint", "leg_right_2_joint", "leg_right_3_joint", "leg_right_4_joint", "leg_right_5_joint", "leg_right_6_joint",
    "torso_1_joint","torso_2_joint"};

void swapQuatWXYZ(Eigen::VectorXd &input_)
{
  Eigen::VectorXd tmp(input_.size());
  tmp = input_;
  input_[3] = tmp[6];
  input_[4] = tmp[3];
  input_[5] = tmp[4];
  input_[6] = tmp[5];
}
void swapQuatXYZW(Eigen::VectorXd &input_)
{
  Eigen::VectorXd tmp(input_.size());
  tmp = input_;
  input_[3] = tmp[4];
  input_[4] = tmp[5];
  input_[5] = tmp[6];
  input_[6] = tmp[3];
}



};
#endif
