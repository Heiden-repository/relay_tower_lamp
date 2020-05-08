#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include "std_msgs/Int8.h"

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <time.h>
#include <thread>
#include <mutex>
#include <vector>

#define turn_off_all 0 

#define turn_off_red_light 10
#define turn_on_red_light 11
#define flashing_red_light 12

#define turn_off_yellow_light 20
#define turn_on_yellow_light 21
#define flashing_yellow_light 22

#define turn_off_green_light 30
#define turn_on_green_light 31
#define flashing_green_light 32

#define turn_on_all 1
#define flashing_duration 0.2

#define protocol_size 7
#define ascii_turn_on 0x31
#define ascii_turn_off 0x30
#define ascii_R 0x52
#define ascii_Y 0x59
#define ascii_Null 0x20
#define ascii_red_light 0x31
#define ascii_yellow_light 0x32
#define ascii_green_light 0x33
#define ascii_last_CR 0x0D
#define ascii_all_light_A 0x41

class Relay_tower_lamp
{
private:
    ros::NodeHandle &nh_;
    int serial_port;

    int prev_light_signal;
    bool stop_flashing;

    void initValue(void);
    void initSubscriber();
    bool serial_connect(void);

    void recieve_lamp_msg_callback(const std_msgs::Int8::ConstPtr &relay_lamp_msg);
    bool send_serial_protocol_to_relay(int light_signal_num);
    bool turn_off_all_light(unsigned char send_serial_protocol[]);
    bool send_protocol(unsigned char send_serial_protocol[]);
    bool flashing_lamp(unsigned char send_serial_protocol[]);
    //Subscriber
    ros::Subscriber relay_tower_lamp_sub;

public:
    void runLoop(void);

    Relay_tower_lamp(ros::NodeHandle &_nh) : nh_(_nh)
    {
        initValue();
        initSubscriber();
        //serial_connect();
    }

    ~Relay_tower_lamp()
    {
    }
};