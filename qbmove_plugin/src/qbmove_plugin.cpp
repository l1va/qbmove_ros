
/*
 *  Copyright (C) 2015 Bioengineering and Robotics Research Center "E.Piaggio"
 *  Author: Valeria Parnenzini
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <qbmove_plugin/qbmove_plugin.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/PID.hh>
#include <stdio.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>
#include <iostream>

/* servo motor parameters */
#define J 0.001
#define b 0.01
#define Kt 0.8
#define Kb 1.3
#define R 2.3
#define L 4

//################################################################
/* matrixes used for state space representation */

#define Ts 0.001

double Ad[N][N] = {{1.0, 0.0008},
                   {0,   0.6299}
};

double Bd[N][U] = {{0.0001, 0.0004},
                   {0.2785, 0.8008}
};

double Cd[M][N] = {{1, 0}};

double Dd[M][U] = {{0, 0}};

//###############################################################
/* PID parameters in discrete domain */
#define Kp 0.001
#define Ki 0.0
#define Kd 0.008

using namespace gazebo;
using namespace std;

void qbmovePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    int argc = 0;
    char **argv;
    ros::init(argc, argv, "qbmove_plugin");

    this->model = _parent;

    std::string joint_name = _sdf->GetElement("joint")->Get<string>();
    std::string command_topic = _sdf->GetElement("command_topic")->Get<string>();
    std::string echo_topic = _sdf->GetElement("pos_topic")->Get<string>();

    this->joint = this->model->GetJoint(joint_name);
    //  this->joint_motor1 = this->model->GetJoint("joint_motor1");
    //  this->joint_motor2 = this->model->GetJoint("joint_motor2");

    /* target angle for motor 1 in radians */
    this->q1_ref = 0.0;
    this->old_q1_ref = 0.0;
    this->old_value_motor1_1 = 0.0;

    /* target angle for motor 2 in radians */
    this->q2_ref = 0.0;
    this->old_q2_ref = 0.0;
    this->old_value_motor1_2 = 0.0;

    this->value_k_1 = 0.0;
    this->value2_k_1 = 0.0;

    this->tau_ext = 0.0;
    this->tau2_ext = 0.0;

    this->e_k_2 = 0.0;
    this->e_k_3 = 0.0;

    this->e2_k_2 = 0.0;
    this->e2_k_3 = 0.0;

    /* state initialization */
    for (int i = 0; i < N; i++) {
        x1_k[i] = 0.0;
        x2_k[i] = 0.0;
    }
    /* output initialization */
    for (int i = 0; i < M; i++) {
        y1_k[i] = 0.0;
        y2_k[i] = 0.0;
    }
    /* input initialization */
    for (int i = 0; i < U; i++) {
        u1_k[i] = 0.0;
        u2_k[i] = 0.0;
    }

    this->val_motor1 = 0.0;
    this->val_motor2 = 0.0;

    sub = n.subscribe(command_topic, 1, &qbmovePlugin::stiff_pos_Callback, this);
    pub = n.advertise<qbmove_msg::Position>(echo_topic, 500);

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&qbmovePlugin::OnUpdate, this, _1));

}


void qbmovePlugin::stiff_pos_Callback(const qbmove_msg::CommandConstPtr &msg) {
    this->val_motor1 = msg->motor1;
    this->val_motor2 = msg->motor2;
}


void qbmovePlugin::ref_generation(double mot1, double mot2) {
    const int pi = 3.14;

    /* saturation limit for reference values */
    double limit_value = pi;

    /* check if mot1 value is over saturation limit*/
    if (mot1 < -limit_value) {
        mot1 = -limit_value;
    }

    if (mot1 > limit_value) {
        mot1 = limit_value;
    }

    /* check if mot2 value is over saturation limit*/
    if (mot2 < -limit_value) {
        mot2 = -limit_value;
    }

    if (mot2 > limit_value) {
        mot2 = limit_value;
    }

    /* conversion value from radians to ticks */
    double conversion_value = 16434.0 / pi;

    /* q1_ref and q2_ref old values */
    this->old_q1_ref = this->q1_ref;
    this->old_q2_ref = this->q2_ref;

    /* conversion from radians to ticks */
    this->q1_ref = mot1 * conversion_value;
    this->q2_ref = mot2 * conversion_value;
}


double qbmovePlugin::pwm_modulation(double u_pid) {
    /* PWM parameters */
    /* dead zone value */
    double dead_zone = 0.3;
    /* normalization factor after PWM  modulation */
    double norm_factor = 1. / (1. - 0.3);
    /* motor voltage input */
    double Vcc = 8;
    /* command saturation limit */
    int saturation_limit = 1.0;
    /* command given to motor after PWM modulation */
    double u_pwm = 0.0;

    /* command saturation */
    if (u_pid > saturation_limit) {
        u_pwm = saturation_limit;

    }

    if (u_pid < -saturation_limit) {
        u_pwm = -saturation_limit;
    }

    /* dead zone */
    if (u_pid > dead_zone) {
        u_pwm = u_pwm - dead_zone;
    }

    if (u_pid < -dead_zone) {
        u_pwm = u_pwm + dead_zone;
    }

    if ((u_pid >= -dead_zone) && (u_pid <= dead_zone)) {
        u_pwm = 0.0;
    }

    /* normalization factor */
    u_pwm = u_pwm * norm_factor;
    /* output value multiplied for tension value */
    u_pwm = u_pwm * Vcc;

    return u_pwm;

}


void qbmovePlugin::system_update(double *x, double *y, double u1, double u2) {

    /* each motor is simulated as a state-space system */
    /* system update */

    /* output update */
    for (int i = 0; i < M; i++) {
        y[i] = 0.0;

        for (int k = 0; k < N; k++) {
            y[i] += Cd[i][k] * x[k];
        }

    }

    /* state update */
    for (int i = 0; i < N; i++) {
        x1[i] = 0.0;

        for (int j = 0; j < N; j++) {
            x1[i] += Ad[i][j] * x[j];
        }
    }

    for (int i = 0; i < N; i++) {
        x2[i] = Bd[i][0] * u1 + Bd[i][1] * u2;
    }

    for (int i = 0; i < N; i++) {
        x[i] = x1[i] + x2[i];
    }

}


double qbmovePlugin::friction_torque_computation(double motor1, int index) {
    /* index : 1 for motor 1, 2 for motor 2 , 3 for the output shaft */
    /* variables names are the same of Simulink scheme */

    /* friction torque parameters */
    double friction_torque = 0.0;
    double z_max = 0.03;
    double static_friction_torque = 0.0;
    double sign1 = 0.0;
    /* output value from Switch1 block in the Simulink scheme */
    double switch1_out = 0.0;
    /* output value from Switch block in the Simulink scheme */
    double switch_out = 0.0;
    double old_value = 0.0;


    switch (index) {
        case 1:
            old_value = this->value_k_1;
            static_friction_torque = 0.8;
            break;

        case 2:
            old_value = this->value2_k_1;
            static_friction_torque = 0.8;
            break;

        case 3:
            old_value = this->value_k_1_output_shaft;
            static_friction_torque = 0.01;
            break;

        default:
            break;

    }

    sign1 = (motor1 - old_value) / z_max;

    if (sign1 >= -1) {
        switch1_out = old_value;
    } else {
        switch1_out = motor1 + z_max;
    }


    if (sign1 > 1) {
        switch_out = motor1 - z_max;
    } else {
        switch_out = switch1_out;
    }


    if (sign1 > 1) {
        sign1 = 1;
    }

    if (sign1 < -1) {
        sign1 = -1;
    }

    switch (index) {
        case 1:
            this->value_k_1 = switch_out;
            break;

        case 2:
            this->value2_k_1 = switch_out;
            break;

        case 3:
            this->value_k_1_output_shaft = switch_out;
            break;

        default:
            break;
    }

    friction_torque = sign1 * static_friction_torque;
    return friction_torque;

}

void qbmovePlugin::OnUpdate(const common::UpdateInfo & /*_info*/) {
    /* command after PWM modulation */
    double u_pwm = 0.0;

    /* deformation angle */
    double def_angle = 0.0;

    /* friction torque parameter */
    double friction_torque = 0.0;

    /* parameters used to compute motor 1 non linear spring torque  */
    double k1 = 0.022;
    double a1 = 6.85;

    /* parameters used to compute motor 2 non linear spring torque */
    double k2 = 0.022;
    double a2 = 6.85;

    /* descrete PID parameters */
    double P = Kp;
    double I = Ki;
    double D = Kd;

    /* motor 1 error at k-th step */
    double e_k;
    /* motor 2 error at k-th step */
    double e2_k;

    /* factor conversion used for magnetic sensors */
    double conversion_fact_2 = 65536. / (2. * 3.14);

    /* output value from magnetic sensor on motor 1*/
    double magn_sensor_output_q1 = 0.0;

    /* output value from magnetic sensor on motor 2*/
    double magn_sensor_output_q2 = 0.0;

    /* non linear spring torque on motor 1*/
    double torque1 = 0.0;

    /* non linear spring torque on motor 2*/
    double torque2 = 0.0;

    /* motor 1 current */
    double curr_mot_1 = 0.0;

    /* motor 2 current */
    double curr_mot_2 = 0.0;

    /* voltage input to motors */
    double Vcc = 8.0;

    /* output shaft angle */
    this->output_shaft_angle = this->joint->GetAngle(0).Radian();

    /* q1_ref e q2_ref are reference values for motor 1 and motor 2, respectively */
    /* q1_ref and q2_ref update */
    ref_generation(this->val_motor1, this->val_motor2);

    /***************************************************************/
    /***                   MOTOR 1 DYNAMICS                        */
    /***************************************************************/

    /* motor 2 angle is sensed by a magnetic sensor */
    /* value considered is the one obtained from the previous step */
    magn_sensor_output_q1 = this->old_value_motor1_1 * conversion_fact_2;

    /* error in ticks */
    /* in realtà questo è l'errore al passo precedente */
    e_k = this->old_q1_ref - magn_sensor_output_q1;

    /* PID command update */
    u1_k[0] = u1_k[0] + P * (e_k - this->e_k_2) + I * e_k + D * (e_k - 2 * this->e_k_2 + this->e_k_3);

    /* error values update */
    this->e_k_3 = this->e_k_2;
    this->e_k_2 = e_k;

    /* PWM computation */
    u_pwm = pwm_modulation(u1_k[0]);

    /* second input is the extern torque */
    u1_k[1] = this->tau_ext;

    /* old angular motor1 used for error computation*/
    this->old_value_motor1_1 = y1_k[0];

    /* output and state system update */
    system_update(x1_k, y1_k, u_pwm, u1_k[1]);

    /* friction torque computation */
    friction_torque = friction_torque_computation(y1_k[0], 1);

    /* deformation angle between output shaft and motor 1 */
    def_angle = this->output_shaft_angle - y1_k[0];

    /* non linear spring torque computation */
    torque1 = k1 * (sinh(a1 * def_angle));

    /* motor 1 torque update */
    this->tau_ext = torque1 - friction_torque;

    /***************************************************************/
    /***                   MOTOR 2 DYNAMICS                        */
    /***************************************************************/

    /* motor 2 angle is sensed by a magnetic sensor */
    /*  value considered is the one obtained from the previous step */
    magn_sensor_output_q2 = this->old_value_motor1_2 * conversion_fact_2;

    /* error in ticks */
    /* error computed for the previous step */
    e2_k = this->old_q2_ref - magn_sensor_output_q2;

    /* PID command update */
    u2_k[0] = u2_k[0] + P * (e2_k - this->e2_k_2) + I * e2_k + D * (e2_k - 2 * this->e2_k_2 + this->e2_k_3);

    /* error values update */
    this->e2_k_3 = this->e2_k_2;
    this->e2_k_2 = e2_k;

    /* PWM modulation */
    u_pwm = pwm_modulation(u2_k[0]);

    /* second input is the extern torque */
    u2_k[1] = this->tau2_ext;

    /* old angular motor1 used for error computation */
    this->old_value_motor1_2 = y2_k[0];

    /* output and state system update */
    system_update(x2_k, y2_k, u_pwm, u2_k[1]);

    /* friction torque computation */
    friction_torque = friction_torque_computation(y2_k[0], 2);

    /* deformation angle computation between output shaft and motor 2 */
    def_angle = this->output_shaft_angle - y2_k[0];

    /* non linear spring torque computation */
    torque2 = k2 * (sinh(a2 * def_angle));

    /* motor 2 torque update */
    this->tau2_ext = torque2 - friction_torque;

    /***************************************************************/
    /***           OUTPUT SHAFT TORQUE COMPUTATION                 */
    /***************************************************************/

    /* friction torque computation */
    friction_torque = friction_torque_computation(this->output_shaft_angle, 3);

    /* tau_L (output shaft torque) computation */
    this->tau_L = -torque1 - torque2 - friction_torque;

    /* output shaft torque update */
    this->joint->SetForce(0, this->tau_L);

    /* motor 1 motor1 update */
    this->a.SetFromRadian(y1_k[0]);
    //this->joint_motor1->Setmotor1(0, y1_k[0]);

    /* motor 2 motor1 update */
    this->a.SetFromRadian(y2_k[0]);
    //this->joint_motor2->Setmotor1(0, y2_k[0]);

    /* motor 1 current computation */
    curr_mot_1 = Vcc / R - (Kb / R) * x1_k[1];

    /* motor 2 current computation */
    curr_mot_2 = Vcc / R - (Kb / R) * x2_k[1];

    msg_echo.pos_out_shaft = this->output_shaft_angle;
    msg_echo.pos_mot_1 = y1_k[0];
    msg_echo.pos_mot_2 = y2_k[0];
    msg_echo.curr_mot_1 = curr_mot_1;
    msg_echo.curr_mot_2 = curr_mot_2;

    pub.publish(msg_echo);

}
