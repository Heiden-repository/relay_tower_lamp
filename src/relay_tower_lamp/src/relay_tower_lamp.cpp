#include "relay_tower_lamp/relay_tower_lamp.hpp"

void Relay_tower_lamp::initValue(void)
{
    serial_port = 0;
}

void Relay_tower_lamp::initSubscriber()
{
    relay_tower_lamp_sub = nh_.subscribe("/lamp_signal", 10, &Relay_tower_lamp::recieve_lamp_msg_callback, this);
}

void Relay_tower_lamp::recieve_lamp_msg_callback(const std_msgs::Int32::ConstPtr &relay_lamp_msg)
{
    int light_signal_num = relay_lamp_msg->data;
    printf("light_signal_num : %d\n",light_signal_num);
    bool send_success = send_serial_protocol_to_relay(light_signal_num);
    if(send_success) ROS_INFO("change lights");
    else ROS_INFO("fail to change lights");
}

bool Relay_tower_lamp::serial_connect(void)
{
    while (ros::ok())
    {
        serial_port = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
        if (serial_port < 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            printf("Error %i from open: %s\n", errno, strerror(errno));
        }
        else
            break;
    }

    struct termios termi;

    memset(&termi, 0, sizeof(termi));

    termi.c_cflag = B9600;
    termi.c_cflag |= CS8;
    termi.c_cflag |= CLOCAL;
    termi.c_cflag |= CREAD;
    termi.c_iflag = 0;
    termi.c_oflag = 0;
    termi.c_lflag = 0;
    termi.c_cc[VTIME] = 0;
    termi.c_cc[VMIN] = 0;

    tcflush(serial_port, TCIFLUSH);
    tcsetattr(serial_port, TCSANOW, &termi);
    printf("Relay connection\n");
}

bool Relay_tower_lamp::send_serial_protocol_to_relay(int light_signal_num)
{
    int write_data = -1;

    while (1)
    {
        //write_data = write(serial_port, for_writing, 1);
        if (write_data > 0)
        {
            break;
        }
    }
}

void Relay_tower_lamp::runLoop(void)
{
    ros::Rate r(10);

    while (ros::ok())
    {
        //printf("red_light : %u yellow_light : %u green_light : %u\n",red_light, yellow_light, green_light);
        ros::spinOnce();
        r.sleep();
    }
}
