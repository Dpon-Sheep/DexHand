#include "cSerialPort/include/CSerialPort/SerialPort.h"
#include "cSerialPort/include/CSerialPort/SerialPortInfo.h"
#include "kEstimator.hpp"
#include "iostream"

// class MyListener : public itas109::CSerialPortListener
// {
//     public:
//     MyListener(){};
//     MyListener(itas109::CSerialPort *sp)
//         : serialport(sp){};

//     void onReadEvent(const char *portName, unsigned int readBufferLen);
//     private:
//     itas109::CSerialPort *serialport;
//     int message_cnt = 0;
//     char mymessage[9];
// };

const std::string hand_task_string[7]={"NORMAL","STOP","START","CONTACT","SETZERO","GOTO","SETFORCE"};
enum HAND_TASK{NORMAL=0,STOP,START,CONTACT,SETZERO,GOTO,SETFORCE};
class DexHandBasic : public itas109::CSerialPortListener
{

    public:
        DexHandBasic();
        DexHandBasic(const char *portName);
        ~DexHandBasic();
    

        bool now_force;
        double goal_force;
        double pos;
        HAND_TASK task;

    protected:
        void uniform_send(const char *message, int message_len);
        void stop_gripper();
        void start_gripper();
        void close_gripper(int speed);
        void open_gripper(int speed);
        void gripper_pending();
        void set_gripper_zero();
        void gripper_goto(int pos);
        void set_weight_param(double weight_time);
        void approching();
        void forcecontrol(double force);
        void preload(double force);
        void set_pid_param(double Kp,double Ki,int maxi);
        void data_push_in();
        void report_data();
        void onReadEvent(const char *portName, unsigned int readBufferLen);
        void callback(){printf("Why?");};
        kEstimator kestimator;

    private:
        itas109::CSerialPort serial_port;
        
        void update_hand_data();
        int message_cnt = 0;
        char mymessage[9];
        char data[4096];
};


