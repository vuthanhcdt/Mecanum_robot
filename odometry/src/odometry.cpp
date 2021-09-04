#include "ros/ros.h"
#include "serial/serial.h"
#include "std_msgs/Int32.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int16MultiArray.h"



//khai bao serial
std::string port;
serial::Serial ser;

double dt;
float r_banh=0.076,r_x=0.22,r_y=0.22;
float so_xung_encoder=2000;
float he_so_giam_toc=19.2;



double x=0,y=0,theta=0,x_pre=0,y_pre=0,theta_pre=0;
double delta_x,delta_y,delta_theta;
double pi = 3.141592653589793238462643;
double rad_to_deg = 180/3.141592653589793238462643;//chuyen doi sang radian
double deg_to_rad=3.14159265359/1800;
double odom_goc;



uint8_t rec_data,count;
uint8_t buff_data[12];
int16_t goc,goc1,goc_pre,en1,en2,en3,en4,pre_goc;
double theta1,theta2,theta3,theta4;

sensor_msgs::Imu imu;
ros::Publisher odom_pub; //node Publisher
ros::Publisher imu_pub;
std::string topic_odom,topic_imu;
std::string topic_v;
ros::Time measurement_time,measurement_time_imu;
std::string fixed_frame_odom;
ros::Time pre_measurement_time,pre_measurement_time_imu;

void odometry()
{
   theta1=en1/so_xung_encoder*2*pi/he_so_giam_toc;
   theta2=en2/so_xung_encoder*2*pi/he_so_giam_toc;
   theta3=en3/so_xung_encoder*2*pi/he_so_giam_toc;
   theta4=en4/so_xung_encoder*2*pi/he_so_giam_toc;

    odom_goc=theta*rad_to_deg*10;
    delta_x=(theta1+theta2+theta3+theta4)*r_banh/4;
    delta_y=(-theta1+theta2+theta3-theta4)*r_banh/4;
    delta_theta=(-theta1+theta2-theta3+theta4)*r_banh/(4*(r_x+r_y));

   	x=x_pre+delta_x*cos(theta_pre)-delta_y * sin(theta_pre);
	y=y_pre+delta_x * sin(theta_pre)+delta_y*cos(theta_pre);
	theta=theta_pre+delta_theta;

    // ROS_INFO("x=%lf, y=%lf, theta=%lf",x,y,theta);

    x_pre=x;
	y_pre=y;
	theta_pre=theta;

}

void odom()
{
    measurement_time=ros::Time::now(); 
    odometry();
    dt= (measurement_time-pre_measurement_time).toSec();
    geometry_msgs::Quaternion odom_quat= tf::createQuaternionMsgFromYaw(theta);
    nav_msgs::Odometry odom;
    odom.header.stamp=measurement_time;
    odom.header.frame_id = fixed_frame_odom;

    odom.pose.pose.position.x=x;
    odom.pose.pose.position.y=y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;    
    odom.child_frame_id="base_link";

    odom.twist.twist.linear.x=delta_x/dt;
    odom.twist.twist.linear.y=delta_y/dt;
    odom.twist.twist.angular.z=delta_theta/dt;


    odom_pub.publish(odom);
    pre_measurement_time=measurement_time;

}

void imu_data()
{
     //=====================imu====================//
    measurement_time_imu = ros::Time::now();
    dt=(measurement_time-pre_measurement_time_imu).toSec();
    geometry_msgs::Quaternion imu_quad= tf::createQuaternionMsgFromYaw(goc*deg_to_rad);
    imu.header.stamp = measurement_time_imu;
    imu.header.frame_id = "base_link";
    imu.orientation=imu_quad;
    imu.angular_velocity.z=((goc-pre_goc)*deg_to_rad)/dt;
    imu_pub.publish(imu);
    pre_measurement_time_imu=measurement_time_imu;
    pre_goc=goc;
    //ROS_INFO("goc=%d",goc);
}



uint8_t byte_control[11]={0x02,0,0,0,0,0,0,0,0,0,0x03};

void write_driver(uint8_t byte_reset, int16_t v_banh_1,int16_t v_banh_2,int16_t v_banh_3,int16_t v_banh_4)
{
    uint8_t dir_banh_1,dir_banh_2,dir_banh_3,dir_banh_4;
    
    if(v_banh_1<0)
    {
        dir_banh_1=1;
        v_banh_1=-v_banh_1;

    }
    else if(v_banh_1>0)
    {
        dir_banh_1=0;
    }
    if(v_banh_1>255)
        v_banh_1=255;


    if(v_banh_2<0)
    {
        dir_banh_2=0;
        v_banh_2=-v_banh_2;

    }
    else if(v_banh_2>0)
    {
        dir_banh_2=1;
    }
    if(v_banh_2>255)
        v_banh_2=255;




     if(v_banh_3<0)
    {
        dir_banh_3=0;
        v_banh_3=-v_banh_3;

    }
    else if(v_banh_3>0)
    {
        dir_banh_3=1;
    }
    if(v_banh_3>255)
        v_banh_3=255;

            if(v_banh_4<0)
    {
        dir_banh_4=0;
        v_banh_4=-v_banh_4;

    }
    else if(v_banh_4>0)
    {
        dir_banh_4=1;
    }
    if(v_banh_4>255)
        v_banh_4=255;


    byte_control[1]=byte_reset;

    byte_control[2]=(dir_banh_1<<7)|0x01;
    byte_control[3]=v_banh_1;

    byte_control[4]=(dir_banh_2<<7)|0x02;
    byte_control[5]=v_banh_2;

    byte_control[6]=(dir_banh_3<<7)|0x03;
    byte_control[7]=v_banh_3;

    byte_control[8]=(dir_banh_4<<7)|0x04;
    byte_control[9]=v_banh_4;

    ser.write(byte_control,11);

}


void read_v(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    // ROS_INFO("%d,%d",msg->data[0],msg->data[1]);
    write_driver(0,msg->data[0],msg->data[1],msg->data[2],msg->data[3]);
}

//ket noi serial
void ket_noi_serial()
{
    try
    {
        ser.setPort(port);
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
        ros::Duration(5).sleep();
    }
    if (ser.isOpen())
    {
        ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized and opened.");
    }
}


void read_serial()
{
    while (ser.available())
        {
            ser.read(&rec_data,1);
            if(count==0&&rec_data!=0x02)
                break;
            if(count==11&&rec_data!=0x03)
                break;
            if(count==11&&rec_data==0x03)
            {  
                goc=buff_data[1]<<8|buff_data[2];
                en1=buff_data[3]<<8|buff_data[4];
                en2=buff_data[5]<<8|buff_data[6];
                en3=buff_data[7]<<8|buff_data[8];
                en4=buff_data[9]<<8|buff_data[10];
                // ROS_INFO("goc=%d,gt en1=%d, gt en2=%d, gt en3=%d, gt en4=%d",goc,en1,en2,en3,en4);
                odom();
                imu_data();
                count=0;
            }
            else
            {
                buff_data[count]=rec_data;
                count++;
            }
        }
}

int main(int argc, char **argv)
{
    //========================khai bao ban dau=====================================//
    ros::init(argc, argv, "kinetic_iv");//khai bao ten node 
    ros::NodeHandle private_node_handle("~");//node param
    ros::NodeHandle n;

    private_node_handle.param<std::string>("port", port, "/dev/ttyUSB0");//cong usb
    private_node_handle.param<std::string>("topic_odom", topic_odom, "odom_encoder");//cong usb
    private_node_handle.param<std::string>("topic_imu", topic_imu, "IMU");//cong usb
    private_node_handle.param<std::string>("fixed_frame_odom", fixed_frame_odom, "odom");
    private_node_handle.param<std::string>("topic_v", topic_v, "van_toc_banh");//cong usb


    odom_pub = n.advertise<nav_msgs::Odometry>(topic_odom, 10); //publish tọa độ x, y, theta cho follow qua topic odom
    imu_pub = n.advertise<sensor_msgs::Imu>(topic_imu, 10);
    ros::Subscriber sub = n.subscribe(topic_v,1, read_v);
    ket_noi_serial();
    ros::Rate rate_sleep(1000);
    write_driver(0x61,0,0,0,0);
    while (ros::ok())
    {
        read_serial();
        rate_sleep.sleep();
        ros::spinOnce();
    }
write_driver(0x61,0,0,0,0);

}