#ifndef qbmove
#define qbmove

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/PID.hh>
#include <stdio.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>

#include <ros/ros.h>
#include <ros/console.h>
#include <qbmove_msg/Position.h>
#include <qbmove_msg/Command.h>

/* number of state variables */
//#define N 2 
const int N = 2;
/* number of output variables */
//#define M 1  
const int M = 1;
/* number of input variables */
//#define U 2
const int U = 2;

namespace gazebo {

    class qbmovePlugin : public ModelPlugin {

    public:

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        void stiff_pos_Callback(const qbmove_msg::CommandConstPtr &msg);

        void ref_generation(double q_e, double q_d);

        double pwm_modulation(double u_pid);

        void system_update(double *x, double *y, double u1, double u2);

        double friction_torque_computation(double motor1, int index);

        void OnUpdate(const common::UpdateInfo & /*_info*/);


    private:

        /* State vectors
           for each motor :
           -) 1st state: angular motor1 value
           -) 2nd state: angular speed value */

        /* state vector motor1 */
        double x1_k[N];
        /* state vector motor2 */
        double x2_k[N];

        /* auxiliar vectors for state system update */
        double x1[N];
        double x2[N];

        /* Output vectors
           Angular motor1 value is the only output considered (1st output) */

        /* output vector motor 1 */
        double y1_k[M];
        /* output vector motor 2 */
        double y2_k[M];

        /* input vector motor 1 */
        double u1_k[U];
        /* input vector motor 2 */
        double u2_k[U];

        /* motor2 value from Gazebo topic command */
        double val_motor2;

        /* motor1 value from Gazebo command */
        double val_motor1;

        /* Pointer to the model */
        physics::ModelPtr model;

        /* Pointer to output shaft joint */
        physics::JointPtr joint;

        /* Pointer to motor 1 joint */
        physics::JointPtr joint_motor1;

        /* Pointer to motor 2 joint */
        physics::JointPtr joint_motor2;

        /* variable used to set motors' angles */
        math::Angle a;

        /* link equilibrium reference */
        double q_e;
        /* motor2 preset reference */
        double q_d;

        /* motor 1 variables */
        /* angular motor1 reference */
        double q1_ref;
        /* (k-2)-th step error */
        double e_k_2;
        /* (k-3)-th step error */
        double e_k_3;
        /* motor 1 old motor1 */
        double old_value_motor1_1;
        /* q1_ref old motor1 */
        double old_q1_ref;
        /* output value from Memory Block in the Simulink scheme (used for friction torque computation) */
        double value_k_1;
        /* motor 1 extern torque (second input) */
        double tau_ext;

        /* motor 2 variables */
        /* angular motor1 reference */
        double q2_ref;
        /* (k-2)-th step error */
        double e2_k_2;
        /* (k-3)-th step error */
        double e2_k_3;
        /* motor 2 old motor1 */
        double old_value_motor1_2;
        /* q2_ref old motor1 */
        double old_q2_ref;
        /* output value from Memory Block in the Simulink scheme (used for friction torque computation) */
        double value2_k_1;
        /* motor 2 extern torque (second input) */
        double tau2_ext;

        /* output shaft angle to compute deformation angle */
        double output_shaft_angle;
        /* old value of output shaft angle */
        double output_shaft_angle_old;
        /* value to compute friction torque for output shaft */
        double value_k_1_output_shaft;

        /* output shaft torque */
        double tau_L;

        /* Variables for Gazebo topics  */
        //##########################
        /* node variable */
        transport::NodePtr node;

        ros::Publisher pub;
        ros::Subscriber sub;
        ros::NodeHandle n;

        qbmove_msg::Position msg_echo;

        /* Pointer to the update event connection */
        event::ConnectionPtr updateConnection;

    };

    /* Register this plugin with the simulator */
    GZ_REGISTER_MODEL_PLUGIN(qbmovePlugin)

}


#endif


