#include "relay_tower_lamp/relay_tower_lamp.hpp"

void Relay_tower_lamp::initValue(void)
{
    serial_port = -1;
    prev_light_signal = -1;
    stop_flashing = false;
}

void Relay_tower_lamp::initSubscriber()
{
    relay_tower_lamp_sub = nh_.subscribe("/lamp_signal", 10, &Relay_tower_lamp::recieve_lamp_msg_callback, this);
}

void Relay_tower_lamp::recieve_lamp_msg_callback(const std_msgs::Int8::ConstPtr &relay_lamp_msg)
{
    int light_signal_num = relay_lamp_msg->data;
    //printf("light_signal_num : %d\n",light_signal_num);

    if(prev_light_signal != light_signal_num)
        send_serial_protocol_to_relay(light_signal_num);
    else return;
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

bool Relay_tower_lamp::turn_off_all_light(unsigned char send_serial_protocol[])
{
    int write_data = -1;

    send_serial_protocol[3] = ascii_all_light_A;
    send_serial_protocol[5] = ascii_turn_off;

    while (1)
    {
        write_data = write(serial_port, send_serial_protocol, protocol_size);
        if (write_data > 0)
        {
            return 1;
        }
    }

    return 0;
}

bool Relay_tower_lamp::send_protocol(unsigned char send_serial_protocol[])
{
    int write_data = -1;
    while (1)
    {
        write_data = write(serial_port, send_serial_protocol, protocol_size);
        if (write_data > 0)
        {
            return 1;
        }
    }

    return 0;
}

bool Relay_tower_lamp::flashing_lamp(unsigned char send_serial_protocol[])
{
    std::thread flashing_thread([&]() {
    stop_flashing = true;
    bool switching_turn_on_off = true;
    while(stop_flashing)
    {
        if(switching_turn_on_off)
        {
             send_serial_protocol[5] = ascii_turn_on;
             switching_turn_on_off = false;
        }
        else 
        {
            send_serial_protocol[5] = ascii_turn_off;
            switching_turn_on_off = true;
        }
        send_protocol(send_serial_protocol);
        std::this_thread::sleep_for(std::chrono::duration<int>(1));
    }
    return 1; });

    flashing_thread.detach();
}

bool Relay_tower_lamp::send_serial_protocol_to_relay(int light_signal_num)
{
    unsigned char send_serial_protocol[protocol_size];
    memset(send_serial_protocol, 0, protocol_size);
    send_serial_protocol[0] = ascii_R;
    send_serial_protocol[1] = ascii_Y;
    send_serial_protocol[2] = ascii_Null;
    send_serial_protocol[3] = 0;
    send_serial_protocol[4] = ascii_Null;
    send_serial_protocol[5] = 0;
    send_serial_protocol[6] = ascii_last_CR;

    if (light_signal_num == turn_off_all)
    {
        stop_flashing = false;
        turn_off_all_light(send_serial_protocol);
        return 1;
    }
    else if (light_signal_num == turn_on_all)
    {
        stop_flashing = false;
        turn_off_all_light(send_serial_protocol);
        send_serial_protocol[3] = ascii_all_light_A;
        send_serial_protocol[5] = ascii_turn_on;
        send_protocol(send_serial_protocol);
        return 1;
    }
    else if (light_signal_num == turn_off_red_light)
    {
        stop_flashing = false;
        turn_off_all_light(send_serial_protocol);
        return 1;
    }
    else if (light_signal_num == turn_on_red_light)
    {
        stop_flashing = false;
        turn_off_all_light(send_serial_protocol);
        send_serial_protocol[3] = ascii_red_light;
        send_serial_protocol[5] = ascii_turn_on;
        send_protocol(send_serial_protocol);
        return 1;
    }
    else if (light_signal_num == flashing_red_light)
    {
        stop_flashing = false;
        turn_off_all_light(send_serial_protocol);
        send_serial_protocol[3] = ascii_red_light;
        flashing_lamp(send_serial_protocol);
        return 1;
    }
    else if (light_signal_num == turn_off_yellow_light)
    {
        stop_flashing = false;
        turn_off_all_light(send_serial_protocol);
        return 1;
    }
    else if (light_signal_num == turn_on_yellow_light)
    {
        stop_flashing = false;
        turn_off_all_light(send_serial_protocol);
        send_serial_protocol[3] = ascii_yellow_light;
        send_serial_protocol[5] = ascii_turn_on;
        send_protocol(send_serial_protocol);
        return 1;
    }
    else if (light_signal_num == flashing_yellow_light)
    {
        stop_flashing = false;
        turn_off_all_light(send_serial_protocol);
        send_serial_protocol[3] = ascii_yellow_light;
        flashing_lamp(send_serial_protocol);
        return 1;
    }
    else if (light_signal_num == turn_off_green_light)
    {
        stop_flashing = false;
        turn_off_all_light(send_serial_protocol);
        return 1;
    }
    else if (light_signal_num == turn_on_green_light)
    {
        stop_flashing = false;
        turn_off_all_light(send_serial_protocol);
        send_serial_protocol[3] = ascii_green_light;
        send_serial_protocol[5] = ascii_turn_on;
        send_protocol(send_serial_protocol);
        return 1;
    }
    else if (light_signal_num == flashing_green_light)
    {
        stop_flashing = false;
        turn_off_all_light(send_serial_protocol);
        send_serial_protocol[3] = ascii_green_light;
        flashing_lamp(send_serial_protocol);
        return 1;
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
