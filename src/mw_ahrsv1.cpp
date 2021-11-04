#include "Serial.h"
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
typedef struct {
  float roll;
  float pitch;
  float yaw;

  float linear_acc_x;
  float linear_acc_y;
  float linear_acc_z;

  float angular_vel_x;
  float angular_vel_y;
  float angular_vel_z;

} IMUMsg;

class MW_AHRS {

private:
  // Device Name
  int dev = 0;

  IMUMsg imu_raw_data;

  // Data buffer
  char buffer[100];
  char small_buffer[10];

  unsigned char angle_cmd[5] = {0x61, 0x6E, 0x67, 0x0D, 0x0A}; // ang Enter
  unsigned char reset_cmd[5] = {0x7A, 0x72, 0x6F, 0x0D, 0x0A}; // zro Enter
  unsigned char av_cmd[7] = {0x61, 0x76, 0x3D, 0x31,
                             0x30, 0x0D, 0x0A}; // av = 10
  unsigned char speed_cmd[7] = {0x73, 0x70, 0x3D, 0x31,
                                0x30, 0x0D, 0x0A}; // sp=10 Enter
  unsigned char ros_data_cmd[6] = {0x73, 0x73, 0x3D,
                                   0x37, 0x0D, 0x0A}; // ss=7 Enter

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

    m_imu_publisher = m_nh.advertise<sensor_msgs::Imu>("imu/data", 5);
  }
  ~MW_AHRS() { close_serial(dev); }

  void reset_imu() {
    write(dev, reset_cmd, 5);
    read(dev, &buffer, sizeof(buffer));

    buffer[sizeof(buffer)] = '\0';

    if (buffer[0] == 'z' && buffer[1] == 'r' && buffer[2] == 'o') {
      printf("IMU Reset\n");
    }
  }

  void speed_setup() {
    write(dev, speed_cmd, 7);
    read(dev, &buffer, sizeof(buffer));

    buffer[sizeof(buffer)] = '\0';

    if (buffer[0] == 's' && buffer[1] == 'p' && buffer[2] == '=' &&
        buffer[3] == '1' && buffer[4] == '0') {
      printf("Serial Speed Reset\n");
    }
  }

  void start_data_stream() {
    write(dev, ros_data_cmd, 6);
    read(dev, &buffer, sizeof(buffer));

    buffer[sizeof(buffer)] = '\0';
  }

  IMUMsg get_angle_data(void) {
    read(dev, &buffer, 100);

    buffer[sizeof(buffer)] = '\0';
    std::cout << buffer << std::endl;

    IMUMsg raw_data;

    char *ptr = strtok(buffer, " ");
    std::cout << "First Data : " << atof(ptr) << std::endl;

    // if (buffer[0] == 's' && buffer[1] == 's' && buffer[2] == '=') {
    //   char *ptr = strtok(buffer, " ");

    //   ang_count = 0;

    //   while (ptr != NULL) {
    //     ang_count++;

    //     ptr = strtok(NULL, " ");

    //     switch (ang_count) {
    //     case 1:
    //       raw_data.linear_acc_x = atof(ptr);
    //       break;
    //     case 2:
    //       raw_data.linear_acc_y = atof(ptr);
    //       break;
    //     case 3:
    //       raw_data.linear_acc_z = atof(ptr);
    //       break;
    //     case 4:
    //       raw_data.angular_vel_x = atof(ptr);
    //       break;
    //     case 5:
    //       raw_data.angular_vel_x = atof(ptr);
    //       break;
    //     case 6:
    //       raw_data.angular_vel_x = atof(ptr);
    //       break;
    //     case 7:
    //       raw_data.roll = atof(ptr);
    //       break;
    //     case 8:
    //       raw_data.pitch = atof(ptr);
    //       break;
    //     case 9:
    //       raw_data.yaw = atof(ptr);
    //       break;
    //     default:
    //       std::cout << "Unknown";
    //       break;
    //     }
    //   }
    // }

    // std::cout << raw_data.angular_vel_x << std::endl;
    // std::cout << raw_data.angular_vel_y << std::endl;
    // std::cout << raw_data.angular_vel_z << std::endl;

    return raw_data;
  }

  void pub_msg() {
    imu_raw_data = get_angle_data();

    tf::Quaternion orientation = tf::createQuaternionFromRPY(
        imu_raw_data.roll, imu_raw_data.pitch, imu_raw_data.yaw);

    ros::Time now = ros::Time::now();

    m_imu_msg.header.stamp = now;

    m_imu_msg.header.frame_id = "imu_link";
    // orientation
    m_imu_msg.orientation.x = orientation[0];
    m_imu_msg.orientation.y = orientation[1];
    m_imu_msg.orientation.z = orientation[2];
    m_imu_msg.orientation.w = orientation[3];

    std::cout << "x = " << m_imu_msg.orientation.x << std::endl;
    std::cout << "roll = " << imu_raw_data.roll << std::endl;
    std::cout << "pitch = " << imu_raw_data.pitch << std::endl;
    std::cout << "yaw = " << imu_raw_data.yaw << std::endl;
    std::cout << std::endl;

    m_imu_publisher.publish(m_imu_msg);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "mw_ahrsv1");

  MW_AHRS ahrs_obj;
  ros::Rate rate(100);

  ahrs_obj.reset_imu();
  ahrs_obj.speed_setup();
  ahrs_obj.start_data_stream();

  while (ros::ok()) {

    ahrs_obj.get_angle_data();

    rate.sleep();
  }

  return 0;
}