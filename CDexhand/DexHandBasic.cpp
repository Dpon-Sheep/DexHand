#include "DexHandBasic.hpp"

DexHandBasic::DexHandBasic(const char *portName)
{
    this->serial_port = itas109::CSerialPort();
    this->serial_port.init(portName,115200);
    this->serial_port.open();
    if(this->serial_port.isOpen())
        printf("Open Serial Port %s Successfully!\n",portName);
    else
        printf("Failed to Open Serial Port!");

    this->serial_port.connectReadEvent(this);
}
DexHandBasic::~DexHandBasic()
{
    this->serial_port.close();
}

void DexHandBasic::onReadEvent(const char *portName, unsigned int readBufferLen)
{
    int recLen = this->serial_port.readData(this->data, readBufferLen);

    if (recLen > 0)
    {
        this->data[recLen] = '\0';
        std::cout << portName << ", Length: " << recLen << ", Str: " << this->data << std::endl;
        for(int i=0;i<recLen;i++)
        {
            if(this->message_cnt == 0)
            {
                if((this->data[i]>>4)!=0)
                    continue;
                this->mymessage[0]=this->data[i];
                this->message_cnt++;
                continue;
            }
            this->mymessage[this->message_cnt++]=this->data[i];
            if(this->message_cnt==9)
            {
                this->update_hand_data();
            }
        }
    }
}

void DexHandBasic::update_hand_data()
{
    this->task = (HAND_TASK)mymessage[0];
    this->now_force = ((mymessage[1]<<8) + mymessage[2])/100.0;
    this->goal_force = ((mymessage[3]<<8) + mymessage[4])/100.0;
    this->pos = ((((mymessage[5]<<8)+mymessage[6]<<8)+mymessage[7]<<8)+mymessage[8])/10000.0; 
    this->message_cnt = 0;   
    this->callback();
}

void DexHandBasic::uniform_send(const char *message, int message_len)
{
    this->serial_port.writeData(message,message_len);
}

void DexHandBasic::stop_gripper()
{
    char message[5] = {0x00};
    this->uniform_send(message,1);
}

void DexHandBasic::start_gripper()
{
    char message[5] = {0x00,0x01};
    this->uniform_send(message,2);
}
        
void DexHandBasic::close_gripper(int speed = 500)
{
    if (speed > 2000)
    {
        printf("Error: goal speed cannot larger than 2000\n");
        return;
    }
    char message[5] = {0x01};
    message[1] = (speed>>8);
    message[2] = (speed&0xff);
    this->uniform_send(message,3);
}
void DexHandBasic::open_gripper(int speed = 500)
{
    if (speed > 2000)
    {
        printf("Error: goal speed cannot larger than 2000\n");
        return;
    }
    char message[5] = {0x02};
    message[1] = (speed>>8);
    message[2] = (speed&0xff);
    this->uniform_send(message,3);
}
void DexHandBasic::gripper_pending()
{
    this->close_gripper(0);
}

void DexHandBasic::set_gripper_zero()
{
    char message[5] = {03};
    this->uniform_send(message,1);
}
void DexHandBasic::gripper_goto(int pos)
{
    char message[5] = {04,00,00,00};
    message[1]=(pos>>8);
    message[2]=(pos&0xff);
    this->uniform_send(message,4);
}
void DexHandBasic::set_weight_param(double weighttime)
{
    char message[5] = {05,00,00};
    int weighttimedata = (int)(weighttime*10000);
    message[1] = (weighttimedata>>8);
    message[2] = (weighttimedata&0xff);
    this->uniform_send(message,3);
}
void DexHandBasic::approching()
{
    char message[5] = {0x08};
    this->uniform_send(message,1);
}
void DexHandBasic::forcecontrol(double force)
{
    int forcedata = (int)(force*100);
    char message[7] = {06,00,0x32,01,00,00};
    int kdata = round(this->kestimator.k*10000);
    message[1]=(forcedata>>8);
    message[2]=(forcedata & 0xff);
    message[4]=(kdata>>8);
    message[5]=(kdata & 0xff);
    this->uniform_send(message,6);
}
void DexHandBasic::preload(double force)
{
    int forcedata = (int)(force*100);
    char message[4] = {0x06,0x03,(char)0xE8,0x00};
    message[1]=(forcedata>>8);
    message[2]=(forcedata & 0xff);
    this->uniform_send(message,4);
}
void DexHandBasic::set_pid_param(double Kp,double Ki,int max_int)
{
    char message[6] = {07,00,00,00,00,00};
    int KP = (int)(Kp*100);
    int KI = (int)(Ki*10000);
    message[1] = (KP>>8);
    message[2] = (KP&0xff);
    message[3] = (KI>>8);
    message[4] = (KI&0xff);
    message[5] = (max_int);
    this->uniform_send(message,6);
}

void DexHandBasic::data_push_in()
{
    this->kestimator.data_push_in(this->pos,this->now_force);
}

void DexHandBasic::report_data()
{
    printf("%s: nowforce = %.2f N, goalforce = %.2f N, pos = %.4f mm, k= %.4f",hand_task_string[this->task],this->now_force,this->goal_force,this->pos,this->kestimator.k);
}
