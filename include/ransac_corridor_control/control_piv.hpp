#ifndef CONTROL_PIV_H
#define CONTROL_PIV_H
#include <cmath>
#include <vector>
#include <time.h>
#include <ros/ros.h>

#define PI 3.14159265

class ControlPIV
{
private:
    static double errori;
public:
    static double LineTracking(const std::vector<double> &line,
                               const double &v_linear, const double &v_angular,
                               const double &dt,
                               const double &KPT, const double &KIT, const double &KRT, const double &KVT);
};
#endif /* CONTROL_PIV_H */
