#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include "relay_tower_lamp/relay_tower_lamp.h"

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <time.h>
#include <thread>
#include <mutex>

class Relay_tower_lamp
{
private:
    ros::NodeHandle &nh_;
    int serial_port;

    unsigned char red_light;
    unsigned char yellow_light;
    unsigned char green_light;

    void initValue(void);
    void initSubscriber(void);
    bool serial_connect(void);

    void recieve_lamp_msg_callback(const relay_tower_lamp::relay_tower_lamp::ConstPtr &relay_lamp_msg);
    bool send_serial_protocol_to_relay(void);

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