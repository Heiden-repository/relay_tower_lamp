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

#define turn_on_red_light 1
#define turn_off_red_light 2

#define turn_on_yellow_light 3
#define turn_off_yellow_light 4

#define turn_on_green_light 5
#define turn_off_green_light 6

#define turn_on_all 7

class Relay_tower_lamp
{
private:
    ros::NodeHandle &nh_;
    int serial_port;

    void initValue(void);
    void initSubscriber();
    bool serial_connect(void);

    void recieve_lamp_msg_callback(const std_msgs::Int8::ConstPtr &relay_lamp_msg);
    bool send_serial_protocol_to_relay(int light_signal_num);

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