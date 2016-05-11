#include <ros/ros.h>

#include "tf/transform_datatypes.h"

#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher pose_pub;

std_msgs::Float64MultiArray x_msg;
geometry_msgs::PoseStamped pose_msg;

void x_callback(const std_msgs::Float64MultiArray &msg)
{
    x_msg = msg;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_simulator");

    ros::NodeHandle nh;

    ros::Publisher u_pub = nh.advertise<std_msgs::Float64MultiArray>("u", 1);

    ros::Subscriber x_sub = nh.subscribe("x", 1, x_callback);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
    ros::Rate loop_rate(100);
    std_msgs::Float64MultiArray u_msg;
    int m = 6;
    int n = 16;
    u_msg.data.clear();
    x_msg.data.clear();
    for (size_t i = 0; i < 6; i++)
        u_msg.data.push_back(0.);
    for (size_t i = 0; i < n; i++)
        x_msg.data.push_back(0.);


    while (ros::ok())
    {
        std::vector<double> X = x_msg.data;
        double q2d = X[15];
        double x   = X[0];
        double xd  = X[1];
        double y   = X[2];
        double yd  = X[3];
        double z   = X[4];
        double zd  = X[5];
        double thetad = X[9];
        double psid = X[11];
        double q1d = X[13];
        double phid = X[7];
        double q2 = X[14];
        double q1 = X[12];
        double theta = X[8];
        double psi = X[10];
        double phi = X[6];
        double Ix = 0.01;
        double Iy = 0.01;
        double Iz = 0.02;
        double mass = 1;
        double g = 9.81;
        double la = 0.21;
        double b = 9.29 - 5.;
        double d = 1.1 - 6.;
        double m1 = 0.1;
        double m2 = 0.1;
        double l1 = 0.1;
        double l2 = 0.1;
        double lc1 = l1/2.;
        double lc2 = l2/2.;
        double I1 = 1./12.*m1*pow(l1, 2.);
        double I2 = 1./12.*m2*pow(l2, 2.);
        double xdes = -0.0;
        double ydes = 0.0;
        double zdes = 0;

        double tau1 = 0;

        double ux = 2.7 * (xd - 0) + 4*(x - xdes);
        double uy = 2.7 * (yd - 0) + 4*(y - ydes);

        double kd = 2*5*2.2;
        double kp = 4*25;
	u_msg.data[4] = 0;//-0.2*(q1-M_PI/2.0)-0.1*q1d;
	u_msg.data[5] = 0;//-0.1*(q2-1.0)-0.01*q2d;
tau1=u_msg.data[4];
	 u_msg.data[0] = (mass+m1+m2) * (g - 2.7*(zd-0) - 4*(z-zdes));
    u_msg.data[1] = Ix/la * (-kd * (phid - 0) - kp*(phi-0)) + tau1/la + uy * cos(psi) - ux * sin(psi);
    u_msg.data[2] = Iy/la * (-kd * (thetad - 0) - kp*(theta-0)) - ux*cos(psi) - uy * sin(psi);
    u_msg.data[3] = Iz * (-0.0175 * (psid-0) - 0.06125 * (psi - 0 * M_PI / 180.));
    u_msg.data[3] = 0;
    u_msg.data[2] = 0;
    u_msg.data[1] = 0;
u_msg.data[0] = (mass+m1+m2)*g;
//        u_msg.data[0] = 5 + sin(2*M_PI*10*ros::Time::now().toSec());
        u_pub.publish(u_msg);
        
        ros::spinOnce();

       loop_rate.sleep();
    }
}
