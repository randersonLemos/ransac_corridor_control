#ifndef RANSAC_CLASSES_H
#define RANSAC_CLASSES_H

#include "ransac_lib.hpp"
#include "watchdog/watchdog.hpp"


class WatchDog_ransac : public WatchDog{
public:
		WatchDog_ransac(NodeHandle node) : WatchDog(node){};
		int KillApplication();
		bool StartTimer(double duration);
};


class WatchDogHolder_ransac{
public:
		WatchDogHolder_ransac(WatchDog_ransac* watchdog) {dog = watchdog;};
		void operator ()(const TimerEvent& event)		{
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
    float winAngle;		   	// angulo utilizado para a divisao da janela	// direita/esquerda na ultima iteracao
    float last_trajAngle;	// angulo da ultima trajetoria estimada
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
    Publisher* pub;
    CDisplayWindowPlots *window;
    WatchDog_ransac* watchdog;
    double angularVel, dt, max_v_linear, v_linear, lenght, KPT, KIT, KRT, KVT;
    string which_car;
    ros::Time start_time;
    ros::Duration ramp_time;
protected:
    ransacControl(Publisher p, bool wp, NodeHandle node);
public:
    ransacControl();
    static ransacControl* uniqueInst(Publisher p, bool wp, NodeHandle node);
    void ransacCallback(const ransac_project::Bisectrix &biMsg);
    void odometryCallback(const nav_msgs::Odometry &Odom_msg);
    void publica(const ransac_project::CarCommand msg);
    void publica(const geometry_msgs::Twist msg);
    void setangularVel(double x);
    double getangularVel();
    void setdt(double x);
    double getdt();
    void setv_linear(double vlin, int ramp_time);
    void setlenght(double x);
    void setKPT(double x);
    void setKIT(double x);
    void setKRT(double x);
    void setKVT(double x);
    void setwhich_car(string x);
    string getwhich_car();
};
#endif /* RANSAC_CLASSES_H */
