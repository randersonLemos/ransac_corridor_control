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


void ctrlHandler(int /*x*/)
{
    ros::NodeHandle n;
    ros::Publisher p2;
    ransacControl *rc2 = ransacControl::uniqueInst(p2, n);
    if(rc2->getwhich_car().compare("vero") == 0){ // Veros's command
        ransac_project::CarCommand msg_vero;

        msg_vero.speedLeft  = 0;
        msg_vero.speedRight = 0;
        msg_vero.steerAngle = 0;

        rc2->publica(msg_vero);
    }
    else{ // Pionner's command
        geometry_msgs::Twist msg_pioneer;

        msg_pioneer.linear.x  = 0;
        msg_pioneer.linear.y  = 0;
        msg_pioneer.linear.z  = 0;
        msg_pioneer.angular.x = 0;
        msg_pioneer.angular.y = 0;
        msg_pioneer.angular.z = 0;

        rc2->publica(msg_pioneer);
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

    ROS_INFO_STREAM("CURRENT | " <<
                    "velocity: " << v_linear <<
                    " angular velocity: " << angularVel);

    /* funçao de controle */
    rudder = Control::LineTracking(bisectrix, v_linear, angularVel, dt, KPT, KIT, KRT, KVT);

    dt = ros::Time::now().toSec();

    if(which_car.compare("vero") == 0){
        ransac_project::CarCommand msg_vero;

        msg_vero.speedLeft = v_linear;
        msg_vero.speedRight = v_linear;
        msg_vero.steerAngle = rudder; //atan(rudder * lenght / v_linear);
        
        ROS_INFO("PRECISA AJEITA ESSA MENSSAGEM");
        ROS_INFO("steerAngle: (%.2f) v_linear %lf", msg_vero.steerAngle, v_linear);

        pub->publish(msg_vero);
    }
    else{
        geometry_msgs::Twist msg_pioneer;

        msg_pioneer.linear.x = v_linear; /* speedLeft = speedRight */
        msg_pioneer.linear.y = 0;
        msg_pioneer.linear.z = 0;
        msg_pioneer.angular.x = 0;
        msg_pioneer.angular.y = 0;
        msg_pioneer.angular.z = rudder;

        ROS_INFO_STREAM("TARGET  | " <<
                        "velocity: " << v_linear <<
                        " angular velocity: " << rudder << "\n");

        pub->publish(msg_pioneer);
    }
} /* ransacCallback */

int main(int argc, char **argv){
    ros::init(argc, argv, "ransac_control");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    ransacControl* rc;

    /* Essential parameters to perform the vero/pioneer control */
    std::string which_car;
    double max_v_linear, ramp_time, lenght = 0;
    double KPT, KIT, KRT, KVT;
    if(!nh.getParam("which_car", which_car)){
        ROS_ERROR("must specify vero or pioneer");
        exit(0);
    }

    if(which_car.compare("vero") == 0){ // Vero's parameters
        if(!nh.getParam("max_v_linear", max_v_linear) || !nh.getParam("KPT", KPT) || !nh.getParam("KIT", KIT) ||
        !nh.getParam("KRT", KRT) || !nh.getParam("KVT", KVT) || !nh.getParam("lenght", lenght) ||
        !nh.getParam("ramp_time", ramp_time)){
            ROS_ERROR("parameters not specified");
            exit(0);
        }
        //ros::Publisher pubTwist = n.advertise<geometry_msgs::Twist>(VERO_COMMAND_TOPIC,1);
        //rc = ransacControl::uniqueInst(pubTwist, n);
        ros::Publisher pubCarCommand = n.advertise<ransac_project::CarCommand>(VERO_COMMAND_TOPIC, 1);
        rc = ransacControl::uniqueInst(pubCarCommand, n);
    }
    else if(which_car.compare("pioneer") == 0){ // Pionner's parameters
        if(!nh.getParam("max_v_linear", max_v_linear) || !nh.getParam("KPT", KPT) || !nh.getParam("KIT", KIT) ||
        !nh.getParam("KRT", KRT) || !nh.getParam("KVT", KVT) || !nh.getParam("ramp_time", ramp_time)){
            ROS_ERROR("parameters not specified");
            exit(0);
        }
        ros::Publisher pubTwist = n.advertise<geometry_msgs::Twist>(PIONEER_COMMAND_TOPIC, 1);
        rc = ransacControl::uniqueInst(pubTwist, n);
    }
    else{
        ROS_ERROR("must specify vero or pioneer");
        exit(0);
    }

    /* Setting parameters to control vero/pioneer */
    rc->setwhich_car(which_car);
    rc->setangularVel(0);
    rc->setdt(0);
    rc->setv_linear(max_v_linear, ramp_time);
    rc->setKPT(KPT);
    rc->setKIT(KIT);
    rc->setKRT(KRT);
    rc->setKVT(KVT);
    rc->setlenght(lenght);

    signal(SIGINT, ctrlHandler);
    signal(SIGABRT, ctrlHandler);

    ros::Subscriber subOdom = n.subscribe(VERO_ODOMEMTRY_TOPIC, 1, &ransacControl::odometryCallback, rc);
    ros::Subscriber subBise = n.subscribe(RANSAC_BISECTRIX_TOPIC, 1, &ransacControl::ransacCallback, rc);
    ros::spin();

    return 0;
}
