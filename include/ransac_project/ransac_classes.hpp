#ifndef RANSAC_CLASSES_H
#define RANSAC_CLASSES_H


#include "ransac_lib.hpp"
#include "watchdog/watchdog.hpp"


class WatchDog_ransac : public WatchDog{
public:
    WatchDog_ransac(ros::NodeHandle node) : WatchDog(node){};
    int KillApplication();
    bool StartTimer(double duration);
};


class WatchDogHolder_ransac{
public:
    WatchDogHolder_ransac(WatchDog_ransac* watchdog) {dog = watchdog;};
    void operator ()(const ros::TimerEvent& event){
         dog->KillApplication();
    }
    WatchDog_ransac* dog;
};

class laser{
private:
    static laser* instance;

    ros::Publisher* pub_lines;
    ros::Publisher* pub_bisec;

    WatchDog* watchdog;

    float threshold, p_inliers, dataWidth, winWidth, winLength;
    bool verbose;
    float winAngle; // angulo utilizado para a divisao da janela
    float last_trajAngle; // angulo da ultima trajetoria estimada
    float last_winAngle;	
protected:
    laser(const ros::Publisher &p1, const ros::Publisher &p2, const ros::NodeHandle &node);
public:
    static laser* uniqueInst(const ros::Publisher &p1, const ros::Publisher &p2, const ros::NodeHandle &node);

    void laserCallback(const sensor_msgs::LaserScan& msg);

    void setthreshold(const double &x);
    void setp_inliers(const double &x);
    void setdataWidth(const double &x);
    void setwinWidth(const double &x);
    void setwinLength(const double &x);
    void setVerbose(const bool &x);
};


class ransacControl{
private:
    static ransacControl* instance;

    ros::Publisher* pub;

    WatchDog_ransac* watchdog;

    double angularVel, dt, v_linear;

    ros::Time start_time;

    /* parameters defined by the user */
    double max_v_linear, length, KPT, KIT, KRT, KVT;
    std::string which_car;
    ros::Duration ramp_time;

protected:
    ransacControl(const ros::Publisher &p, const ros::NodeHandle &node);
public:
    static ransacControl* uniqueInst(const ros::Publisher p, const ros::NodeHandle node);

    void ransacCallback(const ransac_project::Bisectrix &biMsg);
    void odometryCallback(const nav_msgs::Odometry &Odom_msg);

    void publica(const ransac_project::CarCommand &msg);
    void publica(const geometry_msgs::Twist &msg);

    void setmaxv_linear(const double &x);
    void setramp_time(const int &x);
    void setKPT(const double &x);
    void setKIT(const double &x);
    void setKRT(const double &x);
    void setKVT(const double &x);
    void setlength(const double &x);
    void setwhich_car(const std::string &x);
    std::string getwhich_car();

    void setangularVel(const double &x);
    double getangularVel();

    void configTime();
};
#endif /* RANSAC_CLASSES_H */
