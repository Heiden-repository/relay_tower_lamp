#include "relay_tower_lamp/relay_tower_lamp.hpp"

void Relay_tower_lamp::initValue(void)
{
    serial_port = 0;
}

void Relay_tower_lamp::initSubscriber()
{
    relay_tower_lamp_sub = nh_.subscribe("/lamp_signal", 10, &Relay_tower_lamp::recieve_lamp_msg_callback, this);
}

void Relay_tower_lamp::recieve_lamp_msg_callback(const std_msgs::ByteMultiArray::ConstPtr &relay_lamp_msg)
{
    std::cout << "call_back" << std::endl;
    int msg_data_size = relay_lamp_msg->layout.dim.size;
    unsigned char to_send_protocol_data[msg_data_size];
    for(int i=0;i<msg_data_size;i++)
        to_send_protocol_data[i] = relay_lamp_msg->data[i];

    //send_serial_protocol_to_relay(to_send_protocol_data,msg_data_size);
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

bool Relay_tower_lamp::send_serial_protocol_to_relay(unsigned char to_send_protocol_data[],int send_data_size)
{
    int write_data = -1;
    while(1)
    {
        write_data = write(serial_port,to_send_protocol_data,send_data_size);
        if(write_data > 0)
            return;
    }
}

void Relay_tower_lamp::runLoop(void)
{
    ros::Rate r(10);

    while (ros::ok())
    {
        //printf("red_light : %u yellow_light : %u green_light : %u\n",red_light, yellow_light, green_light);

        r.sleep();
    }
}
