#include "handle_points.hpp"

bool HandlePoints::is_left_point(const float x, const float y){
    if(x <= winLength){
        if(y >= 0.0 && y <= winWidth/2){
            return true;
        }
    }
    return false;
}


bool HandlePoints::is_right_point(const float x, const float y){
    if(x <= winLength){
        if(y <= 0.0 && y >= -winWidth/2){
            return true;
        }
    }
    return false;
}
