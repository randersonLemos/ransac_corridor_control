#ifndef HANDLE_POINTS_H
#define HANDLE_POINTS_H

#include <cmath>

class HandlePoints{
private:
    float winWidth;
    float winLength;

protected:
    HandlePoints (const HandlePoints &other) {}
    HandlePoints &operator= (const HandlePoints &other) {}
public:
    HandlePoints () {}
    HandlePoints (
                   float _winWidth
                  ,float _winLength
                 ):
                   winWidth(_winWidth)
                  ,winLength(_winLength)
    {}
    bool is_left_point(const float x, const float y);
    bool is_right_point(const float x, const float y);
};
#endif /* HANDLE_POINTS_H */
