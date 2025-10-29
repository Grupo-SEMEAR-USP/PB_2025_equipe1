#ifndef HW_INTERFACE_HPP
#define HW_INTERFACE_HPP

#define HW_IF_UPDATE_FREQ 10
#define HW_IF_TICK_PERIOD (1.0 / HW_IF_UPDATE_FREQ)

// Inclusão de bibliotecas
#include <ros/ros.h>
#include <ros/console.h>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>                         
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// Mensagens padrão
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
// Mensagens para tratar cmd_vel e odometria
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
// Mensagens criadas para comunicação
#include "robot_communication/velocity_comm.h"
#include "robot_communication/encoder_comm.h"


class HwInterface {
public:
    HwInterface(ros::NodeHandle& nh); // Recebe NodeHandle por referência
    // Funções de callback
    void cb_cmdVel(const geometry_msgs::Twist::ConstPtr& msg); 
    void cb_encoder(const robot_communication::encoder_comm::ConstPtr& msg);
    void cb_cmdTimeout(const ros::TimerEvent&); // Callback para timeout sem cmd_vel
    // Funções de publicação
    void publishWheelSpeeds();
    void updateOdometry();
    // Segurança
    void updateWheelSpeedForDeceleration(); // Desaceleração
    double clampSpeed(double v_input); // Limita velocidade para intervalos possíveis 

private:
    /*——— ROS ———*/
    ros::NodeHandle nh_;
    ros::Publisher velocity_command_pub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber encoder_sub_;
    ros::Publisher odom_pub_;
    ros::Timer command_timeout_; // Timer de segurança
    tf::TransformBroadcaster odom_broadcaster_;

    /*——— Comando para Hardware ———*/
    robot_communication::velocity_comm commanded_vel_msg_;
    double commanded_left_vel_ = 0.0;
    double commanded_right_vel_ = 0.0;

    /*——— Parâmetros Físicos ———*/
    float wheel_radius_ = 0.0;      // Raio das rodas [m]
    float track_width_ = 0.0;       // Distância entre as rodas (bitola) [m]
    float deceleration_rate_ = 0.0; // Taxa de desaceleração [rad/s por tick]
    float max_speed_ = 0.0;         // Velocidade máxima [rad/s]
    float min_speed_ = 0.0;         // Velocidade mínima (negativa) [rad/s]    

    /*——— Odometria ———*/
    double odom_x_ = 0.0;  // [m]
    double odom_y_ = 0.0;  // [m]
    double odom_yaw_ = 0.0; // Yaw [rad]

    /*——— Estado Atual ———*/
    double current_linear_vel_x_ = 0.0;  // [m/s]
    double current_angular_vel_z_ = 0.0; // [rad/s]

    /*——— Controle de Tempo ———*/
    ros::Time current_time_;
    ros::Time last_time_;
};

#endif // HW_INTERFACE_HPP