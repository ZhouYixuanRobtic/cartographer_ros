// author: Li Chunjing, Echiev, Beijing
//2017-12-27


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <sensor_msgs/Imu.h>  
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <time.h>
#include <termios.h>
#include <unistd.h>
// #include <string>

using namespace std;


// #define DEVICENAME                      "/dev/ttyUSB1"
// #define BAUDRATE                        115200

#define LOBYTE(w)       ((unsigned char)(((unsigned long)(w)) & 0xff))
#define HIBYTE(w)       ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))

// #define M_PI   3.141592653
#define M_G    9.81  
#define DEG_TO_RAD(a) (a/180.0*M_PI) 
#define RAD_TO_DEG(a) (a/M_PI*180.0)   





class PortHandler
{
  private:
    int     socket_fd_;
    int     baudrate_;
    char    port_name_[30];

    double  packet_start_time_;
    double  packet_timeout_;
    double  tx_time_per_byte;

  public:
    static const int DEFAULT_BAUDRATE_ = 115200;

    bool   is_using_;

  public:
      PortHandler(const char *port_name);
     	~PortHandler() { }

     bool    openPort();
     void    closePort();
     void    clearPort();
     void    Port_init(const char *port_name, const int baudrate);

     bool    setBaudRate(const int baudrate);
     bool    setupPort(int cflag_baud);
     int     getCFlagBaud(int baudrate);

     int     readPort(uint8_t *packet, int length);
     int     writePort(uint8_t *packet, int length);
};


PortHandler::PortHandler(const char *port_name) 
  : socket_fd_(-1),
    baudrate_(DEFAULT_BAUDRATE_),
    packet_start_time_(0.0),
    packet_timeout_(0.0),
    tx_time_per_byte(0.0)
{
  is_using_ = false;
  strcpy(port_name_, port_name);
}

bool PortHandler::openPort()
{
  return setBaudRate(baudrate_);
}

void PortHandler::closePort()
{
  if(socket_fd_ != -1)
    close(socket_fd_);
  socket_fd_ = -1;
}

void PortHandler::clearPort()
{
  tcflush(socket_fd_, TCIOFLUSH);
}

void PortHandler::Port_init(const char *port_name, const int baudrate)
{
    // Open port
  if (openPort())
  {
    printf("Succeeded to open the port %s !\n",port_name);
  }
  else
  {
    printf("Failed to open the port!\n");
  }

// Set port baudrate
  if (setBaudRate(baudrate))
  {
    printf("Succeeded to change the baudrate %d !\n",baudrate);
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
  }
}

bool PortHandler::setBaudRate(const int baudrate)
{
  int baud = getCFlagBaud(baudrate);

  closePort();

  if(baud <= 0)   // custom baudrate
  {
    printf("Please check the baudrate!\n");
    return false;
  }
  else
  {
    baudrate_ = baudrate;
    return setupPort(baud);
  }
}

bool PortHandler::setupPort(int cflag_baud)
{
  struct termios newtio;

 // socket_fd_ = open(port_name_, O_RDWR|O_NOCTTY|O_NONBLOCK);
  socket_fd_ = open(port_name_, O_RDWR|O_NOCTTY);  

  if(socket_fd_ < 0)
  {
    printf("[PortHandler::SetupPort] Error opening serial port!\n");
    return false;
  }

  bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

  newtio.c_cflag = cflag_baud | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag      = 0;
  newtio.c_lflag      = 0;
  newtio.c_cc[VTIME]  = 20;  //每个单位是0.1秒  20就是2秒
  newtio.c_cc[VMIN]   = 10;   //

  // clean the buffer and activate the settings for the port
  tcflush(socket_fd_, TCIFLUSH);
  tcsetattr(socket_fd_, TCSANOW, &newtio);

  tx_time_per_byte = (1000.0 / (double)baudrate_) * 10.0;
  return true;
}

int PortHandler::readPort(uint8_t *packet, int length)
{
  return read(socket_fd_, packet, length);
}

int PortHandler::writePort(uint8_t *packet, int length)
// int PortHandler::writePort(const char *packet, int length)
{
  return write(socket_fd_, packet, length);
}

int PortHandler::getCFlagBaud(int baudrate)
{
  switch(baudrate)
  {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1250000:           //added by li chunjing 2017-03-04
      return B1152000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 2500000:
      return B2500000;
    case 3000000:
      return B3000000;
    case 3500000:
      return B3500000;
    case 4000000:
      return B4000000;
    default:
      return -1;
  }
}

class File_RW
{
  private:
      int fd;        //文件描述符
      char buf[100];
      int ret;

  public:
    File_RW(){fd = -1; ret = -1;};
    ~File_RW(){};

    void file_open(char *port_name);
    void file_write(float data);
    void file_write(double data);
    void file_write(int32_t data);
    void file_write(char ch);
    void file_close();
};

void File_RW::file_open(char *port_name)
{
  fd = open(port_name, O_CREAT | O_TRUNC | O_RDWR, 0666);
 // fd = creat(port_name, O_RDWR);
  //fd = open(port_name, O_CREAT);
     if (-1 == fd)        
     {
        printf("文件打开错误\n");
     }
     else
     {
        printf("文件打开成功，fd = %d.\n", fd);
     }
}

void File_RW::file_write(float data)
{
  sprintf(buf, "%.6f", data);
  ret = write(fd, buf, strlen(buf));
}

void File_RW::file_write(double data)
{
  sprintf(buf, "%.6f", data);
  ret = write(fd, buf, strlen(buf));
}

void File_RW::file_write(int32_t data)
{
  sprintf(buf, "%d", data);
  ret = write(fd, buf, strlen(buf));
}

void File_RW::file_write(char ch)
{
  buf[0] = ch;
  ret = write(fd, buf, 1);
}

void File_RW::file_close()
{
  close(fd);
  printf("file closed succeeded，fd = %d.\n", fd);
}

void getNowTime(char *result)
{
  timespec time;
  tm nowTime;

  clock_gettime(CLOCK_REALTIME, &time);  //获取相对于1970到现在的秒数
  localtime_r(&time.tv_sec, &nowTime);
  // char current[1024];
  // ROS_INFO("%04d_%02d_%02d_%02d:%02d:%02d", nowTime.tm_year + 1900, nowTime.tm_mon+1, nowTime.tm_mday, 
  // nowTime.tm_hour, nowTime.tm_min, nowTime.tm_sec);

  sprintf(result, "/home/x260-16/WORK_SPACE_LCJ/test_imu/data/%04d_%02d_%02d_%02d_%02d_%02d.txt",nowTime.tm_year + 1900, nowTime.tm_mon+1, nowTime.tm_mday, 
  nowTime.tm_hour, nowTime.tm_min, nowTime.tm_sec); // 产生"123"
}


// PortHandler portHandler(DEVICENAME);

uint8_t packet[100];

// union double_union
//   {
//      double d;
//      uint8_t data[8];
//   }time_s, time_ns, Q_x, Q_y, Q_z, Q_w, Pos_x, Pos_y, Pos_z;



struct STime
{
  unsigned char ucYear;
  unsigned char ucMonth;
  unsigned char ucDay;
  unsigned char ucHour;
  unsigned char ucMinute;
  unsigned char ucSecond;
  unsigned short usMiliSecond;
};


struct SAcc
{
  short a[3];
  short T;
};

struct SGyro
{
  short w[3];
  short T;
};

struct SAngle
{
  short Angle[3];
  short T;
};

struct SMag
{
  short h[3];
  short T;
};

struct SQuat
{
  short Q[4];
};

struct STime    stcTime;
struct SAcc     stcAcc;
struct SGyro    stcGyro;
struct SAngle   stcAngle;
struct SMag     stcMag;
struct SQuat    stcQuat;

//file write:
File_RW file_rw;

int main(int argc, char **argv)  //订阅节点需要做四件事情：(1)初始化；(2)从主题订阅消息；(3)然后等待消息到达；(4)当消息到达时，chatterCallback()被回调
{
    ros::init(argc,argv,"imu_9250_node");
    ros::NodeHandle n;


    char device_name[50];
    string dev_name;  
    ros::param::get("~port",dev_name);  
    strcpy(device_name, dev_name.c_str());

    int baudrate;
    ros::param::get("~baudrate",baudrate);  

    string frame_id;
    ros::param::get("~frame_id",frame_id);



    float Roll = 0.0;
    float Pitch = 0.0;
    float Yaw = 0.0;

    float Gyro_x = 0.0;
    float Gyro_y = 0.0;
    float Gyro_z = 0.0;

    double Acc_x = 0.0;
    double Acc_y = 0.0;
    double Acc_z = 0.0;

    double Q_x = 0.0;
    double Q_y = 0.0;
    double Q_z = 0.0;
    double Q_w = 0.0;

    sensor_msgs::Imu imu_data;


    double timeScanCur = 0.0;

    ros::Publisher IMU_pub = n.advertise<sensor_msgs::Imu>("/imu/data", 50);


    // PortHandler portHandler(DEVICENAME);

    // portHandler.Port_init(DEVICENAME, BAUDRATE);

    PortHandler portHandler(device_name);

    portHandler.Port_init(device_name, baudrate);

   // char file_name[100];
   // getNowTime(file_name);
    //file_rw.file_open(file_name);

    int upd = 0;

    int cnt = 0;
    int read_success = 0;

        //指定循环的频率 
    ros::Rate loop_rate(1000); 
    while(ros::ok()) 
    { 
       portHandler.readPort(packet,1);
       if(packet[0] == 0x55)
       {
          portHandler.readPort(packet,10);
          switch(packet[0])
          {
             // case 0x50:  memcpy(&stcTime,&packet[1],8);break;
             case 0x51:  memcpy(&stcAcc,&packet[1],8);upd = 1;break;
             case 0x52:  memcpy(&stcGyro,&packet[1],8);break;
             case 0x53:  memcpy(&stcAngle,&packet[1],8);break;
            // case 0x54:  memcpy(&stcMag,&packet[1],8);break;
            // case 0x55:  memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
            // case 0x56:  memcpy(&stcPress,&ucRxBuffer[2],8);break;
            // case 0x57:  memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
            // case 0x58:  memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
             case 0x59:  memcpy(&stcQuat,&packet[1],8);break;
          }
        }
       else
          continue;


     Roll = (float)stcAngle.Angle[0]/32768*180;
     Pitch = (float)stcAngle.Angle[1]/32768*180;
     Yaw = (float)stcAngle.Angle[2]/32768*180;

     Gyro_x = (float)stcGyro.w[0]/32768*2000/57.3;
     Gyro_y = (float)stcGyro.w[1]/32768*2000/57.3;
     Gyro_z = (float)stcGyro.w[2]/32768*2000/57.3;

     Q_x = (double)((float)stcQuat.Q[1]/32768);
     Q_y = (double)((float)stcQuat.Q[2]/32768);
     Q_z = (double)((float)stcQuat.Q[3]/32768);
     Q_w = (double)((float)stcQuat.Q[0]/32768);

     Acc_x = (double)stcAcc.a[0]/32768*16*M_G;
     Acc_y = (double)stcAcc.a[1]/32768*16*M_G;
     Acc_z = (double)stcAcc.a[2]/32768*16*M_G;
     


     imu_data.header.stamp = ros::Time::now();
     // imu_data.header.frame_id = "camera_init";
     imu_data.header.frame_id = frame_id;

     imu_data.orientation.x = Q_x;
     imu_data.orientation.y = Q_y;
     imu_data.orientation.z = Q_z;
     imu_data.orientation.w = Q_w;

     imu_data.linear_acceleration.x = Acc_x;
     imu_data.linear_acceleration.y = Acc_y;
     imu_data.linear_acceleration.z = Acc_z;

/*
     imu_data.linear_acceleration.x = 0;
     imu_data.linear_acceleration.y = 0;
     imu_data.linear_acceleration.z = 9.8;
*/
     
     imu_data.angular_velocity.x = Gyro_x;
     imu_data.angular_velocity.y = Gyro_y;
     imu_data.angular_velocity.z = Gyro_z;

     if(upd == 1)
     {   
          cnt++;
          if(cnt >= 10)
          {
            cnt = 0;
            IMU_pub.publish(imu_data);
            //ROS_INFO("Roll:%.3f Pitch:%.3f Yaw:%.3f, Gyro_x:%.3f Gyro_y:%.3f Gyro_z:%.3f ", Roll, Pitch, Yaw, Gyro_x, Gyro_y, Gyro_z);
          }
       upd = 0;
     }


    //file_rw.file_write(Roll);
    //file_rw.file_write('\t');
    //file_rw.file_write(Pitch);
    //file_rw.file_write('\t');
    //file_rw.file_write(Yaw);
    //file_rw.file_write('\t');
    //file_rw.file_write(Gyro_x);
    //file_rw.file_write('\t');
    //file_rw.file_write(Gyro_y);
    //file_rw.file_write('\t');
    //file_rw.file_write(Gyro_z);
    //file_rw.file_write('\t');
    //file_rw.file_write(Q_x);
    //file_rw.file_write('\t');
    //file_rw.file_write(Q_y);
    //file_rw.file_write('\t');
   // file_rw.file_write(Q_z);
   // file_rw.file_write('\t');
    //file_rw.file_write(Q_w);
    //file_rw.file_write('\t');
    //file_rw.file_write(Acc_x);
   // file_rw.file_write('\t');
    //file_rw.file_write(Acc_y);
   // file_rw.file_write('\t');
    //file_rw.file_write(Acc_z);
    //file_rw.file_write('\t');

    //file_rw.file_write('\r');  
    //file_rw.file_write('\n');

        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 

    return 0;
}
