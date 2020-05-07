#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include "std_msgs/Bool.h"
#include "std_msgs/ByteMultiArray.h"

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

    void initValue(void);
    void initSubscriber();
    bool serial_connect(void);

    void recieve_lamp_msg_callback(const std_msgs::ByteMultiArray::ConstPtr &relay_lamp_msg);
    bool send_serial_protocol_to_relay(void);
    
    //Subscriber
    ros::Subscriber relay_tower_lamp_sub;
    ros::Subscriber test_sub;

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