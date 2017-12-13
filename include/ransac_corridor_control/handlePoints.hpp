#ifndef HANDLEPOINTS_H
#define HANDLEPOINTS_H

#include <cmath>

class HandlePoints{
private:
    float winWidth;
    float winLength;

    void frameTransformation(float *pt, float const theta); 
    void bisectrixFrame(float *pt, float const * const coeffs);
protected:
    HandlePoints (const HandlePoints &other) {}
    HandlePoints &operator= (const HandlePoints &other) {}
public:
    HandlePoints () {}
    HandlePoints (  float _winWidth
                  , float _winLength
                 )
                  : winWidth(_winWidth)
                  , winLength(_winLength)
        {}
    char selector(float const * const pt, float const * const coeffs);
};
#endif /* HANDLEPOINTS_H */
