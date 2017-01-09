#ifndef HANDLEPOINTS_H
#define HANDLEPOINTS_H

#include <math.h>

class HandlePoints{
private:
    static void frameTransformation(float *pt, float theta); 
    static void bisectrixFrame(float *pt, float *coeffs);
public:
    static char selector(float *pt, float *coeffs);
};
#endif /* HANDLEPOINTS_H */
