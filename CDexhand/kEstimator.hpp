#include "Eigen/Dense"

class kEstimator{
    private:
        static const int PRELOAD_CNT = 20;
        int data_cnt; 
        Eigen::Matrix2d P;
        Eigen::Vector2<double> K;
        double pos0;
        double minff;
        double last_pos;
        Eigen::Vector2<double> theta;
        Eigen::Vector2<double> phi;
    
    public:
        kEstimator();
        void init();
        void data_push_in(double now_pos,double now_force);
        double k;

};