#include "relay_tower_lamp/relay_tower_lamp.hpp"

void Relay_tower_lamp::initValue(void)
{
    serial_port = 0;

    red_light = 0;
    yellow_light = 0;
    green_light = 0;
}

void Relay_tower_lamp::initSubscriber(void)
{
    relay_tower_lamp_sub = nh_.subscribe("/lamp_signal",10,&Relay_tower_lamp::recieve_lamp_msg_callback,this);
}

void  Relay_tower_lamp::recieve_lamp_msg_callback(const relay_tower_lamp::relay_tower_lamp::ConstPtr &relay_lamp_msg)
{
    red_light = relay_lamp_msg->red_light;
    yellow_light = relay_lamp_msg->yellow_light;
    green_light = relay_lamp_msg->green_light;

    //printf("red_light : %u yellow_light : %u green_light : %u\n",red_light,yellow_light,green_light);
    std::cout << "red_light : "<< red_light <<  " yellow_light : " << yellow_light << " green_light : " << green_light << std::endl;
}

bool Relay_tower_lamp::serial_connect(void)
{
    while (ros::ok())
    {
		serial_port = open( "/dev/ttyUSB0", O_RDWR | O_NOCTTY );
		if(serial_port<0)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            printf("Error %i from open: %s\n", errno, strerror(errno));
		}
        else
            break;
    }

    struct termios termi;

    memset( &termi, 0, sizeof(termi) );

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

bool Relay_tower_lamp::send_serial_protocol_to_relay(void)
{
    
}

void Relay_tower_lamp::runLoop(void)
{
    ros::Rate r(200);

    while (ros::ok())
    {
        
         r.sleep();
    }
}
