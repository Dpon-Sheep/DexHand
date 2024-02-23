#include "DexHandApp.hpp"

void DexHand::callback()
{
    this->update_flag = true;
    if(this->if_report)
    {
        this->report_cnt++;
        if(this->report_cnt == this->report_period)
        {
            this->report_data();
            this->report_cnt = 0;
        }
    }
    if(this->forceControlFlag)
        this->data_push_in();
}

DexHand::DexHand(const char *COMName,int report_period,bool if_report)
    : DexHandBasic(COMName)
{
    this->report_period = report_period;
    this->if_report = if_report;
    this->report_cnt = 0;
    this->update_flag = false;
    this->set_gripper_zero();
    while(this->task != SETZERO);
    while(this->task == SETZERO);

    this->set_weight_param(1.883);
    printf("DexHand has initialized successfully.\n");
}

void DexHand::contact()
{
    this->approching();
    this->if_report = true;
    this->kestimator.init();
    this->update_flag = false;
    int cnt = 0;
    while(this->task != CONTACT)
    {
        if (this->update_flag == true)
        {
            this->update_flag = false;
            cnt++;
            if(cnt>=10)
                this->approching();
        }
    }
    while(this->task == CONTACT);
    this->contactFlag = true;
    this->if_report = false;
    printf("DexHand has contacted an object!\n");
}

void DexHand::grasp(double force)
{
    if(force > 20)
    {
        printf("Error: The goal force cannot larger than 20N!");
        return;
    }
    if(this->contactFlag == false)
    {
        printf("Warning: DexHand may not have contact an object. Now it is approching!");
        this->contact();
    }

    this->contactFlag =true;
    this->forceControlFlag = true;
    this->if_report = true;

    this->preload(force);
    int cnt=0;
    this->update_flag = false;
    while(this->task != SETFORCE)
    {
        if (this->update_flag == true)
        {
            this->update_flag = false;
            cnt++;
            if(cnt>=10)
                this->preload(force);
        }
    }
    while(this->task == SETFORCE);
    printf("DexHand has contacted an object!");
    this->forceControlFlag = false;
    this->if_report = false;
}

void DexHand::force_control(double force)
{
    if(force > 20)
    {
        printf("Error: The goal force cannot larger than 20N!");
        return;
    }
    this->forceControlFlag = true;
    this->if_report = true;
    this->forcecontrol(force);
}

void DexHand::pos_control(double pos)
{
    if(pos > 57.0)
    {
        printf("Error: Goal position cannot be greater than 57.0!");
        return;
    }

    this->if_report = true;
    this->gripper_goto(pos*100);
    int cnt = 0;
    this->update_flag = false;
    while(this->task != GOTO)
    {
        if (this->update_flag == true)
        {
            this->update_flag = false;
            cnt++;
            if(cnt>=10)
                this->gripper_goto(pos*100);
        }
    }
    int forcecnt = 0;
    this->update_flag = false;
    while(this->task != GOTO)
    {
        if (this->update_flag == true)
        {
            this->update_flag = false;
            if(this->now_force <=0.03)
            {
                forcecnt++;
                if(forcecnt>=5)
                {
                    this->contactFlag = false;
                    this->forceControlFlag = false;
                }
            }
            else 
                forcecnt = 0;
        }
    }
    printf("DexHand has reached goal position");
    this->if_report = false;
}

