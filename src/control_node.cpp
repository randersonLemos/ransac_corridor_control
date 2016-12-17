#include <signal.h>
#include "ransac_control.hpp"

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


void ctrlHandler(int /*x*/)
{
    ros::NodeHandle n; //n and p are just to call the member function uniqueInst
    ros::Publisher p2;
    ransacControl *rc2 = ransacControl::uniqueInst(p2, n);

    ros::Time begin = ros::Time::now() + ros::Duration(1.0);
    while(ros::Time::now() < begin){ // Let's send this command for at least one second
                                     // for ensuring that the platform stops

        if(rc2->getwhich_car().compare("vero") == 0){
            ROS_INFO_STREAM("Command send to VERO");
            ransac_project::CarCommand msgvero;
            msgvero.speedLeft  = 0.0;
            msgvero.speedRight = 0.0;
            msgvero.steerAngle = 0,0;
            rc2->publica(msgvero);
        }
        else{
            ROS_INFO_STREAM("Command send to PIONEER");
            geometry_msgs::Twist msgpionner;
            msgpionner.linear.x = 0.0 ; msgpionner.linear.y = 0.0;  msgpionner.linear.z = 0.0;
            msgpionner.angular.x = 0.0; msgpionner.angular.y = 0.0; msgpionner.angular.z = 0.0;
            rc2->publica(msgpionner);
        }
    }
    sleep(1);
    exit(0);
} /* ctrlHandler */


int main(int argc, char **argv){
    ros::init(argc, argv, "ransac_control");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");


    /* LOADING PARAMETERS */
    /* Parameters for the class ransacControl, from where the control code is executed */
    std::string which_car;
    double KPT, KIT, KRT, KVT, length, ramp_time, max_v_linear;
    if(!nh.getParam("KPT", KPT)){
        ROS_ERROR("Failed to get param 'KPT'"); exit(0);
    }
    if(!nh.getParam("KIT", KIT)){
        ROS_ERROR("Failed to get param 'KIT'"); exit(0);
    }
    if(!nh.getParam("KRT", KRT)){
        ROS_ERROR("Failed to get param 'KRT'"); exit(0);
    }
    if(!nh.getParam("KVT", KVT)){
        ROS_ERROR("Faild to get param 'KVT'"); exit(0);
    }
    if(!nh.getParam("length", length)){
        ROS_ERROR("Failed fo get param 'length'"); exit(0);
    }
    if(!nh.getParam("ramp_time", ramp_time)){
        ROS_ERROR("Failed to get param 'ramp_time'"); exit(0);
    }
    if(!nh.getParam("which_car", which_car)){
        ROS_ERROR("Failed to get param 'which_car'"); exit(0);
    }
    if(!nh.getParam("max_v_linear", max_v_linear)){
        ROS_ERROR("Failed to get param 'max_v_linear'"); exit(0);
    }

    /* Parameters for topic names*/
    std::string RANSAC_LINES_TOPIC, RANSAC_BISECTRIX_TOPIC, VERO_ODOMETRY_TOPIC,
    VERO_COMMAND_TOPIC, PIONEER_COMMAND_TOPIC;
    if(!nh.getParam("/RANSAC_LINES_TOPIC", RANSAC_LINES_TOPIC)){
        ROS_ERROR_STREAM("Failed to get param '/RANSAC_LINES_TOPIC'"); exit(0);
    }
    if(!nh.getParam("/RANSAC_BISECTRIX_TOPIC", RANSAC_BISECTRIX_TOPIC)){
        ROS_ERROR_STREAM("Failed to get param '/RANSAC_BISECTRIX_TOPIC'"); exit(0);
    }
    if(!nh.getParam("/VERO_ODOMETRY_TOPIC", VERO_ODOMETRY_TOPIC)){
        ROS_ERROR_STREAM("Failed to get param '/VERO_ODOMETRY_TOPIC'"); exit(0);
    }
    if(!nh.getParam("/VERO_COMMAND_TOPIC", VERO_COMMAND_TOPIC)){
        ROS_ERROR_STREAM("Failed to get param '/VERO_COMMAND_TOPIC'"); exit(0);
    }
    if(!nh.getParam("/PIONEER_COMMAND_TOPIC", PIONEER_COMMAND_TOPIC)){
        ROS_ERROR_STREAM("Failed to get param '/PIONEER_COMMAND_TOPIC'"); exit(0);
    }


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

    /* Setting parameters defined by user */
    rc->setKPT(KPT);
    rc->setKIT(KIT);
    rc->setKRT(KRT);
    rc->setKVT(KVT);
    rc->setlength(length);
    rc->setramp_time(ramp_time);
    rc->setwhich_car(which_car);
    rc->setmaxv_linear(max_v_linear);

    /* Adjusting remaning configuration issues */
    rc->configTime();

    signal(SIGINT, ctrlHandler);
    signal(SIGABRT, ctrlHandler);

    ros::Subscriber subOdom = n.subscribe(VERO_ODOMETRY_TOPIC, 1, &ransacControl::odometryCallback, rc);
    ros::Subscriber subBise = n.subscribe(RANSAC_BISECTRIX_TOPIC, 1, &ransacControl::ransacCallback, rc);
    ros::spin();

    return 0;
}
