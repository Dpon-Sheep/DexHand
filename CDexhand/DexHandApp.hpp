#include "DexHandBasic.hpp"

class DexHand : public DexHandBasic
{
    public:
        DexHand(const char *COMName,int report_period = 5,bool if_report = true);
        void contact();
        void grasp(double force);
        void force_control(double force);
        void pos_control(double pos);
    private:
        bool contactFlag = false;
        bool forceControlFlag = false;
        int report_period;
        int report_cnt = 0 ;
        bool if_report = if_report;
        bool update_flag = false;
        void periodic_report();
        void callback();
};