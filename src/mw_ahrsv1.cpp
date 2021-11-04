#include "Serial.h"
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
typedef struct {
  // Euler
  float roll;
  float pitch;
  float yaw;

} Euler;

class MW_AHRS {

private:
  // Device Name
  int dev = 0;

  Euler euler;

  // Data buffer
  char buffer[100];
  unsigned char Tx[5];

  // Serperate Euler Angle Variable
  int ang_count = 0;

  // ASCII CODE
  const unsigned char A = 0x61;
  const unsigned char N = 0x6E;
  const unsigned char G = 0x67;
  const unsigned char CR = 0x0D;
  const unsigned char LF = 0x0A;

  ros::NodeHandle m_nh;
  ros::Publisher m_imu_publisher;
  ros::Subscriber m_sub;

  sensor_msgs::Imu m_imu_msg;

public:
  MW_AHRS() {
    for (int i = 0; i < 100; i++)
      buffer[i] = 0;

    // int open_serial(char *dev_name, int baud, int vtime, int vmin);
    dev = open_serial((char *)"/dev/ttyUSB0", 115200, 0, 0);

    Tx[0] = A;  // a
    Tx[1] = N;  // n
    Tx[2] = G;  // g
    Tx[3] = CR; // CR
    Tx[4] = LF; // LF

    m_imu_publisher = m_nh.advertise<sensor_msgs::Imu>("imu/data", 5);

    // ros::Time::init();
  }
  ~MW_AHRS() { close_serial(dev); }

  Euler get_data(void) {
    write(dev, Tx, 5);
    read(dev, &buffer, 100);

    if (buffer[0] == 'a' && buffer[1] == 'n' && buffer[2] == 'g') {
      char *ptr = strtok(buffer, " ");

      ang_count = 0;

      while (ptr != NULL) {
        ang_count++;

        ptr = strtok(NULL, " ");

        if (ang_count == 1) {
          euler.roll = atof(ptr);
        } else if (ang_count == 2) {
          euler.pitch = atof(ptr);
        } else if (ang_count == 3) {
          euler.yaw = atof(ptr);
        }
      }
    }

    return euler;
  }

  void pub_msg() {
    euler = get_data();

    tf::Quaternion orientation =
        tf::createQuaternionFromRPY(euler.roll, euler.pitch, euler.yaw);

    ros::Time now = ros::Time::now();

    m_imu_msg.header.stamp = now;

    m_imu_msg.header.frame_id = "imu_link";
    // orientation
    m_imu_msg.orientation.x = orientation[0];
    m_imu_msg.orientation.y = orientation[1];
    m_imu_msg.orientation.z = orientation[2];
    m_imu_msg.orientation.w = orientation[3];

    std::cout << "x = " << m_imu_msg.orientation.x << std::endl;
    std::cout << "roll = " << euler.roll << std::endl;
    std::cout << "pitch = " << euler.pitch << std::endl;
    std::cout << "yaw = " << euler.yaw << std::endl;
    std::cout << std::endl;

    m_imu_publisher.publish(m_imu_msg);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "mw_ahrsv1");

  MW_AHRS ahrs_obj;
  ros::Rate rate(100);

  while (ros::ok()) {

    ahrs_obj.pub_msg();

    rate.sleep();
  }

  return 0;
}
