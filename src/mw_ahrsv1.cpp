#include "Serial.h"
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
typedef struct {
  // ang
  float roll;
  float pitch;
  float yaw;

  // acc
  float linear_acc_x;
  float linear_acc_y;
  float linear_acc_z;

  // gyr
  float angular_vel_x;
  float angular_vel_y;
  float angular_vel_z;

} IMUMsg;

class MW_AHRS {

private:
  // Device Name
  int dev = 0;

  // Data buffer
  char buffer[120];
  char small_buffer[10];

  // utils for serial cmd
  unsigned char angle_cmd[5] = {0x61, 0x6E, 0x67, 0x0D, 0x0A}; // ang Enter
  unsigned char gyr_cmd[5] = {0x67, 0x79, 0x72, 0x0D, 0x0A};   // ang Enter
  unsigned char acc_cmd[5] = {0x61, 0x63, 0x63, 0x0D, 0x0A};   // ang Enter
  unsigned char reset_cmd[5] = {0x7A, 0x72, 0x6F, 0x0D, 0x0A}; // zro Enter
  unsigned char av_cmd[7] = {0x61, 0x76, 0x3D, 0x31,
                             0x30, 0x0D, 0x0A}; // av = 10
  unsigned char speed_cmd[8] = {0x73, 0x70, 0x3D, 0x31,
                                0x30, 0x30, 0x0D, 0x0A}; // sp=100 Enter
  unsigned char ros_data_cmd[6] = {0x73, 0x73, 0x3D,
                                   0x37, 0x0D, 0x0A}; // ss=7 Enter

  // hold raw data
  IMUMsg imu_raw_data;

  // Serperate Euler Angle Variable
  int ang_count = 0;

  // ASCII CODE
  const unsigned char A = 0x61;
  const unsigned char N = 0x6E;
  const unsigned char G = 0x67;
  const unsigned char CR = 0x0D;
  const unsigned char LF = 0x0A;

  // Unit converting constants
  double convertor_g2a = 9.80665; // for linear_acceleration (g to m/s^2)
  double convertor_d2r =
      M_PI / 180.0; // for angular_velocity (degree to radian)
  double convertor_r2d =
      180.0 / M_PI;                // for easy understanding (radian to degree)
  double convertor_ut2t = 1000000; // for magnetic_field (uT to Tesla)
  double convertor_c = 1.0;        // for temperature (celsius)

  // ROS Parts

  // whether pub tf or not
  bool publish_tf = true;

  ros::NodeHandle m_nh;
  ros::Publisher m_imu_publisher;
  ros::Subscriber m_sub;

  sensor_msgs::Imu m_imu_msg;
  tf::TransformBroadcaster broadcaster_;

public:
  MW_AHRS() {
    for (int i = 0; i < sizeof(buffer); i++)
      buffer[i] = 0;

    // int open_serial(char *dev_name, int baud, int vtime, int vmin);
    dev = open_serial((char *)"/dev/ttyUSB0", 115200, 0, 0);

    m_imu_publisher = m_nh.advertise<sensor_msgs::Imu>("imu/data", 5);
  }

  ~MW_AHRS() {
    ROS_INFO("Destructor...");
    close_serial(dev);
  }

  void reset_imu() {
    write(dev, reset_cmd, 5);
    read(dev, &buffer, sizeof(buffer));

    buffer[sizeof(buffer)] = '\0';

    if (buffer[0] == 'z' && buffer[1] == 'r' && buffer[2] == 'o') {
      printf("IMU Reset\n");
    }
  }

  void speed_setup() {
    write(dev, speed_cmd, 8);
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

  void get_angle_data(IMUMsg &msg_in) {
    write(dev, angle_cmd, 5);
    read(dev, &buffer, sizeof(buffer));

    if (buffer[0] == 'a' && buffer[1] == 'n' && buffer[2] == 'g') {
      char *ptr = strtok(buffer, " ");

      ang_count = 0;

      while (ptr != NULL) {
        ang_count++;

        ptr = strtok(NULL, " ");

        if (ang_count == 1) {
          msg_in.roll = atof(ptr);
        } else if (ang_count == 2) {
          msg_in.pitch = atof(ptr);
        } else if (ang_count == 3) {
          msg_in.yaw = atof(ptr);
        }
      }
    }
  }

  void parse_ss_data(IMUMsg &msg_in) {
    read(dev, &buffer, sizeof(buffer));

    if (int(buffer[0]) < 97) {
      // std::cout << buffer << std::endl;
      char *rest;
      char *token;
      char *ptr = buffer;

      ang_count = 0;

      while (token = strtok_r(ptr, " ", &rest)) {
        ang_count++;

        if (ang_count == 1) {
          msg_in.linear_acc_x = atof(token);
        } else if (ang_count == 2) {
          msg_in.linear_acc_y = atof(token);
        } else if (ang_count == 3) {
          msg_in.linear_acc_z = atof(token);
        } else if (ang_count == 4) {
          msg_in.angular_vel_x = atof(token);
        } else if (ang_count == 5) {
          msg_in.angular_vel_y = atof(token);
        } else if (ang_count == 6) {
          msg_in.angular_vel_z = atof(token);
        } else if (ang_count == 7) {
          msg_in.roll = atof(token) * -convertor_d2r;
        } else if (ang_count == 8) {
          msg_in.pitch = atof(token) * -convertor_d2r;
        } else if (ang_count == 9) {
          msg_in.yaw = atof(token) * -convertor_d2r;
        }
        ptr = rest;
      }

      // printf("%0.3f %0.3f %0.3f ", msg_in.linear_acc_x, msg_in.linear_acc_y,
      //        msg_in.linear_acc_z);
      // printf("%0.3f %0.3f %0.3f ", msg_in.angular_vel_x,
      // msg_in.angular_vel_y,
      //        msg_in.angular_vel_z);
      // printf("%0.3f %0.3f %0.3f \n \n", msg_in.roll, msg_in.pitch,
      // msg_in.yaw);
    }
  }

  void pub_msg() {

    parse_ss_data(imu_raw_data);

    tf::Quaternion orientation = tf::createQuaternionFromRPY(
        imu_raw_data.roll, imu_raw_data.pitch, imu_raw_data.yaw);

    // tf::Quaternion yaw_rotate(0, 0, -0.7071068, 0.7071068);
    // tf::Quaternion yaw_rotate(0, 0, -1, 0);

    // tf::Quaternion new_orientation = orientation * yaw_rotate;

    ros::Time now = ros::Time::now();

    m_imu_msg.header.stamp = now;

    m_imu_msg.header.frame_id = "imu_link";

    // orientation
    m_imu_msg.orientation.x = orientation[0];
    m_imu_msg.orientation.y = orientation[1];
    m_imu_msg.orientation.z = orientation[2];
    m_imu_msg.orientation.w = orientation[3];

    // original data used the g unit, convert to m/s^2
    m_imu_msg.linear_acceleration.x = imu_raw_data.linear_acc_x * convertor_g2a;
    m_imu_msg.linear_acceleration.y = imu_raw_data.linear_acc_y * convertor_g2a;
    m_imu_msg.linear_acceleration.z = imu_raw_data.linear_acc_z * convertor_g2a;

    // original data used the degree/s unit, convert to radian/s
    m_imu_msg.angular_velocity.x = imu_raw_data.angular_vel_x * convertor_d2r;
    m_imu_msg.angular_velocity.y = imu_raw_data.angular_vel_y * convertor_d2r;
    m_imu_msg.angular_velocity.z = imu_raw_data.angular_vel_z * convertor_d2r;

    m_imu_msg.linear_acceleration_covariance[0] =
        m_imu_msg.linear_acceleration_covariance[4] =
            m_imu_msg.linear_acceleration_covariance[8] = 1000;

    m_imu_msg.angular_velocity_covariance[0] =
        m_imu_msg.angular_velocity_covariance[4] =
            m_imu_msg.angular_velocity_covariance[8] = 1;

    m_imu_msg.orientation_covariance[0] = m_imu_msg.orientation_covariance[4] =
        m_imu_msg.orientation_covariance[8] = 0;

    m_imu_publisher.publish(m_imu_msg);

    if (publish_tf) {
      broadcaster_.sendTransform(tf::StampedTransform(
          tf::Transform(orientation, tf::Vector3(0.0, 0.0, 0.0)),
          ros::Time::now(), "imu_link", "base_link"));
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "mw_ahrsv1");

  MW_AHRS ahrs_obj;
  ros::Rate rate(10);

  ahrs_obj.reset_imu();
  ahrs_obj.speed_setup();
  ahrs_obj.start_data_stream();

  // ros::Duration(1.5).sleep();
  IMUMsg test_imu_raw_data;

  while (ros::ok()) {

    // ahrs_obj.parse_ss_data(test_imu_raw_data);
    ahrs_obj.pub_msg();

    rate.sleep();
  }

  return 0;
}