#include "hw_interface.hpp"

HwInterface::HwInterface(ros::NodeHandle& nh)
: nh_(nh), // Inicializa a cópia do NodeHandle
  command_timeout_(nh_.createTimer(ros::Duration(0.2), 
                                   &HwInterface::cb_cmdTimeout,
                                   this, true, false))
{
    /*——— Publishers e Subscribers ———*/
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10, &HwInterface::cb_cmdVel, this);
    encoder_sub_ = nh_.subscribe("encoder_data", 10, &HwInterface::cb_encoder, this);
    imu_sub_ = nh_.subscribe("/imu/data_fused", 10, &HwInterface::cb_imu, this);
    
    velocity_command_pub_ = nh_.advertise<robot_communication::velocity_comm>("velocity_cmd", 10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
    
    /*——— Carregamento de Parâmetros (wheel_specification.yaml) ———*/
    nh_.getParam("wheel_specification/wheel_radius", wheel_radius_);
    nh_.getParam("wheel_specification/track_width", track_width_);
    nh_.getParam("wheel_specification/deceleration_rate", deceleration_rate_);
    nh_.getParam("wheel_specification/max_speed", max_speed_);
    nh_.getParam("wheel_specification/min_speed", min_speed_);

    /*——— Inicialização de Variáveis ———*/
    odom_x_ = 0.0;
    odom_y_ = 0.0;
    odom_yaw_ = 0.0;
    current_linear_vel_x_ = 0.0;
    current_angular_vel_z_ = 0.0;

    commanded_vel_msg_.left_vel = 0.0;
    commanded_vel_msg_.right_vel = 0.0;

    current_time_ = ros::Time::now();
    last_time_ = ros::Time::now();
}

/* ——————————————————— Callbacks ———————————————————————— */

// Callback da IMU
void HwInterface::cb_imu(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Extrai Orientação
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Zera a orientação no início
    if (!imu_initialized_) {
        imu_initial_offset_ = yaw;
        imu_initialized_ = true;
        ROS_INFO("IMU Inicializada. Offset de Yaw: %.2f rad", imu_initial_offset_);
    }

    // Calcula o yaw relativo ao offset inicial
    double relative_yaw = yaw - imu_initial_offset_;

    // Normaliza para garantir -PI a PI
    while (relative_yaw > M_PI) relative_yaw -= 2.0 * M_PI;
    while (relative_yaw < -M_PI) relative_yaw += 2.0 * M_PI;

    imu_yaw_ = relative_yaw;

    // Extrai a velocidade angular
    imu_angular_vel_z_ = msg->angular_velocity.z;
}

void HwInterface::cb_cmdVel(const geometry_msgs::Twist::ConstPtr& msg)
{
    // Obtenção das velocidades conforme /cmd_vel
    double vx = msg->linear.x;
    double omega = msg->angular.z;

    /*——— Cinemática Inversa ———*/
    // Converte (vx, omega) para velocidades de roda (rad/s)
    // V_r = (2*vx + omega*L) / (2*R)
    // V_l = (2*vx - omega*L) / (2*R)
    // Onde L = track_width_, R = wheel_radius_
    commanded_right_vel_ = clampSpeed((vx + (omega * track_width_ / 2.0)) / wheel_radius_);
    commanded_left_vel_ = clampSpeed((vx - (omega * track_width_ / 2.0)) / wheel_radius_);

    // Timeout
    command_timeout_.stop();
    command_timeout_.setPeriod(ros::Duration(0.2), true);
    command_timeout_.start();
}

void HwInterface::cb_encoder(const robot_communication::encoder_comm::ConstPtr& msg)
{    
    // [rad/s] ———> [m/s]
    float v_right_ms = msg->right_enc * wheel_radius_;
    float v_left_ms  = msg->left_enc * wheel_radius_;

    /*——— Cinemática Direta ———*/
    // vx = (V_r + V_l) / 2
    // omega = (V_r - V_l) / L
    current_linear_vel_x_ = (v_right_ms + v_left_ms) / 2.0;
    current_angular_vel_z_ = (v_right_ms - v_left_ms) / (double)track_width_;

    // Filtro de sanidade
    if (current_linear_vel_x_ >= max_speed_ || current_linear_vel_x_ <= min_speed_) {
        current_linear_vel_x_ = 0;
    }
}

double HwInterface::clampSpeed(double v_input)
{
    return std::min(std::max(v_input, (double)min_speed_), (double)max_speed_);
}

void HwInterface::publishWheelSpeeds()
{
    // Preenche a mensagem customizada
    commanded_vel_msg_.left_vel = (float)commanded_left_vel_;
    commanded_vel_msg_.right_vel = (float)commanded_right_vel_;

    // Publica para o hardware
    velocity_command_pub_.publish(commanded_vel_msg_);
}

void HwInterface::cb_cmdTimeout(const ros::TimerEvent&)
{
    // Se o timer estourar, começa a desacelerar
    updateWheelSpeedForDeceleration();
}

void HwInterface::updateWheelSpeedForDeceleration()
{
    // Função inline para desacelerar um valor 'v'
    auto decel = [this](double& v){
        if (std::abs(v) > deceleration_rate_) 
            v -= deceleration_rate_ * ((v > 0) ? 1 : -1); // Reduz 'v' mantendo o sinal
        else 
            v = 0.0;
    };

    decel(commanded_left_vel_);
    decel(commanded_right_vel_);

    // Se o robô ainda não parou, re-agenda o timer para continuar desacelerando
    if (commanded_left_vel_ != 0.0 || commanded_right_vel_ != 0.0)
    {
        command_timeout_.stop();
        command_timeout_.setPeriod(ros::Duration(0.05), true); // Loop de desaceleração (20Hz)
        command_timeout_.start();
    }
}

void HwInterface::updateOdometry()
{
    current_time_ = ros::Time::now();
    
    // Calcula o tempo real desde a última atualização
    double dt = (current_time_ - last_time_).toSec();
    if (dt <= 0.0) {
        if (last_time_.toSec() == 0) last_time_ = current_time_;
        return; 
    }

    /*——— Cálculo da Odometria ———*/

    // Variação no ângulo (yaw)
    double yaw_for_calculation;
    double angular_vel_for_odom;

    if (imu_initialized_) {
        //  ela manda na rotação (é muito mais precisa)
        // Se temos IMU, sobrescreve o Yaw acumulado dos encoders pelo da IMU
        odom_yaw_ = imu_yaw_; 
        yaw_for_calculation = imu_yaw_;
        angular_vel_for_odom = imu_angular_vel_z_;
    } else {
        // Fallback: Se a IMU não ligou, usa encoders
        double delta_th = current_angular_vel_z_ * dt;
        odom_yaw_ += delta_th;
        yaw_for_calculation = odom_yaw_;
        angular_vel_for_odom = current_angular_vel_z_;
    }

    // Variação em X e Y
    double delta_x = (current_linear_vel_x_ * cos(yaw_for_calculation)) * dt;
    double delta_y = (current_linear_vel_x_ * sin(yaw_for_calculation)) * dt;

    // Acumula a posição
    odom_x_ += delta_x;
    odom_y_ += delta_y;
    
    last_time_ = current_time_; // Salva o tempo para o próximo cálculo

    /*——— Publicação ———*/

    // Definição da Orientação
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odom_yaw_);
    q.normalize();
    geometry_msgs::Quaternion odom_quat = tf2::toMsg(q);

    // Publica a transformação TF
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp    = current_time_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id  = "base_link";
    odom_trans.transform.translation.x = odom_x_;
    odom_trans.transform.translation.y = odom_y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation      = odom_quat;
    odom_broadcaster_.sendTransform(odom_trans);

    // Publica a mensagem de odometria
    nav_msgs::Odometry odom;
    odom.header.stamp    = current_time_;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = odom_x_;
    odom.pose.pose.position.y = odom_y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x  = current_linear_vel_x_;
    odom.twist.twist.linear.y  = 0.0; // Por ser diferencial
    odom.twist.twist.angular.z = angular_vel_for_odom; 
    odom_pub_.publish(odom);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hw_interface");
    ros::NodeHandle nh;

    HwInterface controller(nh); // Cria o objeto

    ros::Rate rate(HW_IF_UPDATE_FREQ);
    ros::AsyncSpinner spinner(4); // Callbacks (assíncronos) rodam em threads separadas
    spinner.start();

    while (ros::ok())
    {
        // Loop Síncrono
        controller.publishWheelSpeeds(); // Envia comandos
        controller.updateOdometry();     // Publica estado
        rate.sleep();
    }
    return 0;
}