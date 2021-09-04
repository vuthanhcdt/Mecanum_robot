#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16MultiArray.h"
#include "math.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

float he_so_giam_toc=19.2;
float r_banh=0.076,r_x=0.22,r_y=0.22;
int v_max_dong_co=10000,pwm_max=255;

double theta;
float goc_di_chuyen;
double pi = 3.141592653589793238462643;

double v_banh_1,v_banh_2,v_banh_3,v_banh_4;
int16_t pwm1,pwm2,pwm3,pwm4;

double x=0,y=0;
double v_x=0,v_y=0,v_goc=0;

std::string topic_v;
std::string topic_vt;
std::string topic_vdai_vgoc;
ros::Publisher  Pub_v;
ros::Subscriber sub_v;
std_msgs::Int16MultiArray msg;
std::string topic_odom;


int16_t convert_pwm(double v_banh)
{
    int16_t pwm;
    pwm=v_banh*he_so_giam_toc;
    pwm=((double)pwm/v_max_dong_co)*(pwm_max);
    return pwm;
}


void read_vt(const geometry_msgs::Twist::ConstPtr& vt)
{
    v_x=vt->linear.x;
    v_y=vt->linear.y;
    v_goc=vt->angular.z;
}

void kinetic_inv()
{
    
    v_banh_1=(1/r_banh)*(v_x-v_y-(r_x+r_y)*v_goc);
    v_banh_2=(1/r_banh)*(v_x+v_y+(r_x+r_y)*v_goc);
    v_banh_3=(1/r_banh)*(v_x+v_y-(r_x+r_y)*v_goc);
    v_banh_4=(1/r_banh)*(v_x-v_y+(r_x+r_y)*v_goc);

    pwm1= convert_pwm(v_banh_1*(60/(2*pi))); //vong/phut  
    pwm2= convert_pwm(v_banh_2*(60/(2*pi)));
    pwm3= convert_pwm(v_banh_3*(60/(2*pi)));
    pwm4= convert_pwm(v_banh_4*(60/(2*pi)));
    // ROS_INFO("%f,%f,%d,%d",vtgoc_trai,vtgoc_phai,v_banh_trai,v_banh_phai);

}
void tinh_v()
{

    kinetic_inv();
    msg.data.resize(0);
    msg.data.push_back(pwm1);
    msg.data.push_back(pwm2);
	msg.data.push_back(pwm3);
	msg.data.push_back(pwm4);
    Pub_v.publish(msg);
}

int main(int argc, char **argv)
{
    //========================khai bao ban dau=====================================//
    ros::init(argc, argv, "kinetic_inv");//khai bao ten node 
    ros::NodeHandle private_node_handle("~");//node param
    ros::NodeHandle n;

    private_node_handle.param<std::string>("topic_v", topic_v, "van_toc_banh");//cong usb
    private_node_handle.param<std::string>("topic_vt", topic_vt, "van_toc_xe");//cong usb
    
	Pub_v = n.advertise<std_msgs::Int16MultiArray>(topic_v, 10); //publish van toc qua topicv
    sub_v = n.subscribe(topic_vt, 1, read_vt);
    ros::Rate rate_sleep(100);
    while (ros::ok())
    {
      tinh_v();
      rate_sleep.sleep();
      ros::spinOnce();
    }

}
