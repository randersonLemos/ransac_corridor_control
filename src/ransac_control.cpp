#include "topics.hpp"
#include "control.hpp"
#include "ransac_lib.hpp"
#include "ransac_classes.hpp"

/*
* parametros:
*     lenght:             distancia entre eixos do VERO
*     v_linear:           define a velocidade linear do VERO
*     which_car:          define para que carro o controle sera efetuado
*     KPT, KIT, KRT, KVT: parametros do controlador
*
* ransacCallback: Função que recebe os coeficientes (a, b, c) da reta bissetriz representada na forma ax + by +c = 0
*      Entrada:
*           mensagem ransac_project::Bisectrix que define que contem os coeficientes da reta bisetriz
*      Saidas:
*           publica a mensagen ransac_project::CarCommand, que envia o comando para o VERO sendo ele uma velocidade linear e um angulo de rotaçao ou a mensagem geometry_msgs::Twist que envia o comando para o Pioneer sendo ele uma velocidade linear e uma angular.
*/


const int sizeRudder = 25;
double arrayRudder[sizeRudder] = {};


double getAverage(double arr[], int size){
    int i;
    double sum = 0;
    double avg;
    for (i = 0; i < size; ++i){
        sum += arr[i];
    }
    avg = double(sum)/size;
    return avg;
}


void ctrlHandler(int /*x*/)
{
    ros::NodeHandle n; //n and p are just to call the member function uniqueInst
    ros::Publisher p2;
    ransacControl *rc2 = ransacControl::uniqueInst(p2, n);

    ros::Time begin = ros::Time::now() + ros::Duration(1.0);
    while(ros::Time::now() < begin){ // Let's send this command for at least one second
                                     // for ensuring that the platform stoped
        if(rc2->getwhich_car().compare("vero") == 0){ // Veros's command
            static_cast<Vero*>(rc2->getplatform())->setmsg(0.0, 0.0);
            rc2->publica(static_cast<Vero*>(rc2->getplatform())->getmsg());
        }
        else{ // Pionner's command
            static_cast<Pionner*>(rc2->getplatform())->setmsg(0.0, 0.0);
            rc2->publica(static_cast<Pionner*>(rc2->getplatform())->getmsg());
        }
    }
    sleep(1);
    exit(0);
} /* ctrlHandler */

void ransacControl::odometryCallback(const nav_msgs::Odometry &Odom_msg){
    angularVel = Odom_msg.twist.twist.angular.z;
} /* odometryCallback */

void ransacControl::ransacCallback(const ransac_project::Bisectrix &biMsg)
{
    //watchdog->IsAlive();

    double rudder;
    std::vector<double> bisectrix(4);

    bisectrix[0] = -15;
    bisectrix[1] = 15;
    bisectrix[2] = -(biMsg.bisectrix[2] + biMsg.bisectrix[0]*bisectrix[0])/biMsg.bisectrix[1];
    bisectrix[3] = -(biMsg.bisectrix[2] + biMsg.bisectrix[0]*bisectrix[1])/biMsg.bisectrix[1];

    if(v_linear < max_v_linear){ // gradually increasing speed
        v_linear = max_v_linear * (ros::Time::now() - start_time).toSec() / ramp_time.toSec();
    }
    //ROS_INFO_STREAM("CURRENT | " <<
    //                "velocity: " << v_linear <<
    //                " angular velocity: " << angularVel);


    /**************************************************************************************/
    /*Control Function*/
    rudder = Control::LineTracking(bisectrix, v_linear, angularVel, dt, KPT, KIT, KRT, KVT);
    /**************************************************************************************/

    dt = ros::Time::now().toSec();

    for(int i=0; i<sizeRudder-1; ++i){
        arrayRudder[sizeRudder-1-i] = arrayRudder[sizeRudder-1-i-1];
    }
    arrayRudder[0] = rudder;

    for(int i=0; i<5; ++i){
        ROS_INFO_STREAM(i<<": "<<arrayRudder[i]);
    }
    ROS_INFO_STREAM("RudderAvg :" <<getAverage(arrayRudder, sizeRudder));


    if(which_car.compare("vero") == 0){
        //ROS_INFO_STREAM("TARGET  | " <<
        //                "velocity: " << v_linear <<
        //                " steering: " << rudder << "\n");
        ROS_INFO_STREAM("Command send to VERO");
        static_cast<Vero*>(platform)->setmsg(v_linear, getAverage(arrayRudder, sizeRudder));
        pub->publish(static_cast<Vero*>(platform)->getmsg());
    }
    else{
        //ROS_INFO_STREAM("TARGET  | " <<
        //                "velocity: " << v_linear <<
        //                " angular velocity: " << rudder << "\n");
        ROS_INFO_STREAM("Command send to PIONEER");
        static_cast<Pionner*>(platform)->setmsg(v_linear, getAverage(arrayRudder, sizeRudder));
        pub->publish(static_cast<Pionner*>(platform)->getmsg());
    }
} /* ransacCallback */

int main(int argc, char **argv){
    ros::init(argc, argv, "ransac_control");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    /* Essential parameters to perform the vero/pioneer control */
    std::string which_car;
    double KPT, KIT, KRT, KVT;
    double max_v_linear, ramp_time, length;

    if(!nh.getParam("KPT", KPT))                  { ROS_ERROR("must specify KPT");          exit(0);}
    if(!nh.getParam("KIT", KIT))                  { ROS_ERROR("must specify KIT");          exit(0);}
    if(!nh.getParam("KRT", KRT))                  { ROS_ERROR("must specify KRT");          exit(0);}
    if(!nh.getParam("KVT", KVT))                  { ROS_ERROR("must specify KVT");          exit(0);}
    if(!nh.getParam("length", length))            { ROS_ERROR("must specify length");       exit(0);}
    if(!nh.getParam("ramp_time", ramp_time))      { ROS_ERROR("must specify ramp_time");    exit(0);}
    if(!nh.getParam("which_car", which_car))      { ROS_ERROR("must specify which_car");    exit(0);}
    if(!nh.getParam("max_v_linear", max_v_linear)){ ROS_ERROR("must specify max_v_linear"); exit(0);}

    ros::Publisher pub;
    if(which_car.compare("vero") == 0){
        pub = n.advertise<ransac_project::CarCommand>(VERO_COMMAND_TOPIC, 1);
    }
    else if(which_car.compare("pioneer") == 0){
        pub =  n.advertise<geometry_msgs::Twist>(PIONEER_COMMAND_TOPIC, 1);
    }
    else{ ROS_ERROR("which_car must be vero or pioneer"); exit(0);
    }

    ransacControl* rc;
    rc = ransacControl::uniqueInst(pub, n);

    /* Setting parameters to control vero/pioneer */
    rc->setKPT(KPT);
    rc->setKIT(KIT);
    rc->setKRT(KRT);
    rc->setKVT(KVT);
    rc->setlength(length);
    rc->setwhich_car(which_car);
    rc->setplatform(); // I WILL ORGANIZE THIS AFTER
    rc->setdt(ros::Time::now().toSec());
    rc->setv_linear(max_v_linear, ramp_time);

    signal(SIGINT, ctrlHandler);
    signal(SIGABRT, ctrlHandler);

    ros::Subscriber subOdom = n.subscribe(VERO_ODOMEMTRY_TOPIC, 1, &ransacControl::odometryCallback, rc);
    ros::Subscriber subBise = n.subscribe(RANSAC_BISECTRIX_TOPIC, 1, &ransacControl::ransacCallback, rc);
    ros::spin();

    return 0;
}
