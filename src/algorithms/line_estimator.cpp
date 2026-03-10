#include "algorithms/line_estimator.hpp"

DiscreteLinePose estimate_discrete_line_pose(uint16_t left_val, uint16_t right_val){
    int sensor_l = left_val > threshold_r;
    int sensor_r = right_val > threshold_r;

    if(sensor_l && sensor_r){
        return DiscreteLinePose.LineBoth;
    }
    if(!sensor_l && !sensor_r){
        return DiscreteLinePose.LineNone;
    }
    if(!sensor_l && sensor_r){
        return DiscreteLinePose.LineOnLeft;
    }
    if(sensor_l && !sensor_r){
        return DiscreteLinePose.LineOnRight;
    }
}