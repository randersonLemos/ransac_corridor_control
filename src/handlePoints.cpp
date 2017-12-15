#include "handlePoints.hpp"

void HandlePoints::frameTransformation(float *pt, float const theta){
    float arr[] = {pt[0], pt[1]};
    pt[0] =  arr[0]*std::cos(theta) + arr[1]*std::sin(theta);
    pt[1] = -arr[0]*std::sin(theta) + arr[1]*std::cos(theta);
}

void HandlePoints::bisectrixFrame(float *pt, float const * const coeffs){
    float theta = std::atan(-coeffs[0]/(coeffs[1]+1e-6));
    float arr1[] = {0.0, coeffs[2]/(coeffs[1]+1e-6)};
    float arr2[] = {pt[0], pt[1]};

    frameTransformation(arr1, theta);
    frameTransformation(arr2, theta);

    pt[0] = arr1[0] + arr2[0];
    pt[1] = arr1[1] + arr2[1];
}

char HandlePoints::selector(float const * const pt){
   float arr[] = {pt[0], pt[1]};
    if(arr[0] <= winLength){
        if(arr[1] >= 0.0 && arr[1] <= winWidth/2){
            return 'L';
        }
        else if(arr[1] <= 0.0 && arr[1] >= -winWidth/2){
            return 'R';
        }
    }
    return 'F'; // F of False
}

char HandlePoints::selector(float const * const pt, float const * const coeffs){
   float arr[] = {pt[0], pt[1]};
    //bisectrixFrame(arr, coeffs) // transform from car frame to reference path frame;
    if(arr[0] <= winLength){
        if(arr[1] >= 0.0 && arr[1] <= winWidth/2){
            return 'L';
        }
        else if(arr[1] <= 0.0 && arr[1] >= -winWidth/2){
            return 'R';
        }
    }
    return 'F'; // F of False
}
