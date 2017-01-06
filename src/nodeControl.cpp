#include <signal.h>
#include "control.hpp"

/*
* parametros:
*     lenght:             distancia entre eixos do VERO
*     v_linear:           define a velocidade linear do VERO
*     platform:          define para que carro o controle sera efetuado
*     KPT, KIT, KRT, KVT: parametros do controlador
*
* ransacCallback: Função que recebe os coeficientes (a, b, c) da reta bissetriz representada na forma ax + by +c = 0
*      Entrada:
*           mensagem ransac_project::Bisectrix que define que contem os coeficientes da reta bisetriz
*      Saidas:
*           publica a mensagen ransac_project::CarCommand, que envia o comando para o VERO sendo ele uma velocidade linear e um angulo de rotaçao ou a mensagem geometry_msgs::Twist que envia o comando para o Pioneer sendo ele uma velocidade linear e uma angular.
*/

void ctrlHandler(int /*x*/){
    Control *rc = Control::uniqueInst();

    ros::Time begin = ros::Time::now() + ros::Duration(1.0);
    while(ros::Time::now() < begin){ // Let's send this command for at least one second
                                     // to ensures platform stops

        if(rc->getPlatform().compare("vero") == 0){
            ROS_INFO_STREAM("Command send to VERO");
            ransac_project::CarCommand msgvero;
            msgvero.speedLeft  = 0.0;
            msgvero.speedRight = 0.0;
            msgvero.steerAngle = 0,0;
            rc->publica(msgvero);

        }
        else{
            ROS_INFO_STREAM("Command send to PIONEER");
            geometry_msgs::Twist msgpionner;
            msgpionner.linear.x = 0.0 ; msgpionner.linear.y = 0.0;  msgpionner.linear.z = 0.0;
            msgpionner.angular.x = 0.0; msgpionner.angular.y = 0.0; msgpionner.angular.z = 0.0;
            rc->publica(msgpionner);
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
    /* Parameters for the class Control, from where the control code is executed */
    std::string platform;
    double KPT, KIT, KRT, KVT, length, rampTime, maxLinearVel;
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
    if(!nh.getParam("rampTime", rampTime)){
        ROS_ERROR("Failed to get param 'rampTime'"); exit(0);
    }
    if(!nh.getParam("platform", platform)){
        ROS_ERROR("Failed to get param 'platform'"); exit(0);
    }
    if(!nh.getParam("maxLinearVel", maxLinearVel)){
        ROS_ERROR("Failed to get param 'maxLinearVel'"); exit(0);
    }
    /* Parameters for topic names*/
    std::string RANS_LINES_TOPIC,
                RANS_BISEC_TOPIC,
                VERO_ODOME_TOPIC,
                VERO_COMMA_TOPIC,
                PION_COMMA_TOPIC;
    if(!nh.getParam("/RANS_LINES_TOPIC", RANS_LINES_TOPIC)){
        ROS_ERROR_STREAM("Failed to get param '/RANS_LINES_TOPIC'"); exit(0);
    }
    if(!nh.getParam("/RANS_BISEC_TOPIC", RANS_BISEC_TOPIC)){
        ROS_ERROR_STREAM("Failed to get param '/RANS_BISEC_TOPIC'"); exit(0);
    }
    if(!nh.getParam("/VERO_ODOME_TOPIC", VERO_ODOME_TOPIC)){
        ROS_ERROR_STREAM("Failed to get param '/VERO_ODOME_TOPIC'"); exit(0);
    }
    if(!nh.getParam("/VERO_COMMA_TOPIC", VERO_COMMA_TOPIC)){
        ROS_ERROR_STREAM("Failed to get param '/VERO_COMMA_TOPIC'"); exit(0);
    }
    if(!nh.getParam("/PION_COMMA_TOPIC", PION_COMMA_TOPIC)){
        ROS_ERROR_STREAM("Failed to get param '/PION_COMMA_TOPIC'"); exit(0);
    }

    Control* rc = Control::uniqueInst();

    /* Setting parameters loaded from ROS' server */
    rc->setKPT(KPT);
    rc->setKIT(KIT);
    rc->setKRT(KRT);
    rc->setKVT(KVT);
    rc->setLength(length);
    rc->setRampTime(rampTime);
    rc->setPlatform(platform);
    rc->setMaxLinearVel(maxLinearVel);

    ros::Publisher pub;
    if(platform.compare("vero") == 0){
        pub = n.advertise<ransac_project::CarCommand>(VERO_COMMA_TOPIC, 1);
    }
    else if(platform.compare("pioneer") == 0){
        pub =  n.advertise<geometry_msgs::Twist>(PION_COMMA_TOPIC, 1);
    }
    else{ ROS_ERROR("platform must be vero or pioneer"); exit(0);
    }
    rc->setPub(pub);

    /* Adjusting remaning configuration issues */
    rc->configTime();

    signal(SIGINT, ctrlHandler);
    signal(SIGABRT, ctrlHandler);

    ros::Subscriber subOdom = n.subscribe(VERO_ODOME_TOPIC, 1, &Control::odometryCallback, rc);
    ros::Subscriber subBise = n.subscribe(RANS_BISEC_TOPIC, 1, &Control::ransacCallback, rc);

    ros::spin();

    return 0;
}
