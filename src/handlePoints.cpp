#include "handlePoints.hpp"

void HandlePoints::frameTransformation(float *pt, float theta){
    float arr[] = {pt[0], pt[1]};
    pt[0] =  arr[0]*cos(theta) + arr[1]*sin(theta);
    pt[1] = -arr[0]*sin(theta) + arr[1]*cos(theta);
}

void HandlePoints::bisectrixFrame(float *pt, float *coeffs){
    float theta = atan(-coeffs[0]/(coeffs[1]+1e-6));
    float arr1[] = {0.0, coeffs[2]/(coeffs[1]+1e-6)};
    float arr2[] = {pt[0], pt[1]};

    HandlePoints::frameTransformation(arr1, theta);
    HandlePoints::frameTransformation(arr2, theta);

    pt[0] = arr1[0] + arr2[0];
    pt[1] = arr1[1] + arr2[1];
}

char HandlePoints::selector(float *pt, float *coeffs){
   float arr[] = {pt[0], pt[1]};
    HandlePoints::bisectrixFrame(arr, coeffs);
    if(arr[1] >= 0.0){
        return 'L';
    }
    else{
        return 'R';
    }
}
