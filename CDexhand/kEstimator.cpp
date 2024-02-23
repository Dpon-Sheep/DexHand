#include "kEstimator.hpp"

kEstimator::kEstimator()
{
    this->init();
}
void kEstimator::init()
{
    this->data_cnt = 0;
    this->k = 0.07;    
}

void kEstimator::data_push_in(double now_pos,double now_force)
{
    if(this->data_cnt == 0)
    {
        this->pos0 = now_pos-20;
        this->data_cnt++;
        now_pos = now_pos - this->pos0;
        this->theta << this->k,now_force-this->k*now_pos;

        this->phi << now_pos,1;
        this->P = this->phi * this->phi.transpose();
        this->P = this->P.inverse();
        this->last_pos = now_pos;
        return;
    }

    now_pos = now_pos - this->pos0;

    this->phi << now_pos,1;
    double err = now_force - (this->theta.transpose()*this->phi);
    double x = 130*(abs(err)-0.08);
    double lambda1 = abs(now_pos-this->last_pos);
    double lambda2 = x/(1.0+abs(x))+1.0;
    double forgot_factor = 1-lambda1*lambda2*1.575;
    if(forgot_factor < 0.3)
        forgot_factor = 0.3;

    this->K = (this->P*this->phi)/(forgot_factor + this->phi.transpose() * this->P * this->phi);
    this->P = ((Eigen::MatrixXd::Identity(2, 2) - this->K * this->phi.transpose()) * this->P ) / forgot_factor;
    this->theta = this->theta + this->K * err;
    this->k = this->theta(0);
    if(this->k < 0.01)
        this->k = 0.01;

    this->last_pos = now_pos;

}