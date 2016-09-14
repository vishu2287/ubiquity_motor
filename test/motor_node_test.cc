#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ubiquity_motor/motor_serial.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

static geometry_msgs::Twist vel;

void Vel_Odom_loopback() {
    MotorSerial motorSerial("/tmp/S1", 9600, 1000);

    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);

    ros::Time last_time;
    ros::Time current_time;

    current_time = ros::Time::now();
    last_time = current_time;
    ros::Duration elasped;

    int16_t vel_left  = 0;
    int16_t vel_right = 0;

    while (ros::ok()) {
        while (motorSerial.commandAvailable()) {
            MotorMessage mm_vel = motorSerial.receiveCommand();
            if (mm_vel.getRegister() == MotorMessage::REG_BOTH_SPEED_SET) {
                int32_t vel = mm_vel.getData();
                vel_left  = (vel >> 16) & 0xffff;
                vel_right = vel & 0xffff;
            }
        }

        current_time = ros::Time::now();
        elasped = current_time - last_time;
        last_time = current_time;

        // TODO figure out this fudging factor
        vel_left *= elasped.toSec() * 2.7;
        vel_right *= elasped.toSec() * 2.7;

        MotorMessage mm_odom;
        mm_odom.setData((vel_left << 16) | (vel_right & 0x0000ffff));
        mm_odom.setType(MotorMessage::TYPE_RESPONSE);
        mm_odom.setRegister(MotorMessage::REG_BOTH_ODOM);
        ROS_ERROR("Writing %f", ros::Time::now().toSec());
        motorSerial.transmitCommand(mm_odom);

        cmd_vel_pub.publish(vel);
        ros::spinOnce();
        boost::this_thread::interruption_point();
        ros::Duration(0.1).sleep();
    }
}

class MotorNodeTests : public ::testing::Test {
 protected:
  virtual void SetUp() {
      cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
      odom_sub = nh.subscribe("/odom", 1, &MotorNodeTests::odom_callback, this);
      num_odom = 0;
  }
  virtual void TearDown() {}

  void odom_callback(nav_msgs::Odometry odom) {
      odom_msg = odom;
      num_odom++;
  }

  ros::NodeHandle nh;
  ros::Publisher cmd_vel_pub;
  ros::Subscriber odom_sub;
  nav_msgs::Odometry odom_msg;
  int num_odom;
};

TEST_F(MotorNodeTests, sendVelGetOdomVel) {
    while (!num_odom);
    vel.linear.x = 0.5;

    // Let acceleration happen
    while (num_odom < 10);

    // TODO make this a more exact equal
    ASSERT_NEAR(0.5, odom_msg.twist.twist.linear.x, 0.1);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "node_test");
    ros::NodeHandle nh;

    boost::thread loopback_thread(&Vel_Odom_loopback);
    return RUN_ALL_TESTS();
}