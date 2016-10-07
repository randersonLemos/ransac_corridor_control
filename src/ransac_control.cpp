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


namespace aux{

const int sizeRudder = 25;
double Rudder[sizeRudder] = {};

/* Compute the average value of the array arr with size size */
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

/* Update array arr adding to it a new element at the initial position
 and discarding the lasted element*/
void moveWindow(double elem,double arr[], int size){
    for(int i=0; i<size; ++i){
        arr[size-1-i] = arr[size -1-i-1];
    }
    arr[0] = elem;
}
}


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

void ransacControl::odometryCallback(const nav_msgs::Odometry &Odom_msg){
    angularVel = Odom_msg.twist.twist.angular.z;
} /* odometryCallback */

void ransacControl::ransacCallback(const ransac_project::Bisectrix &biMsg)
{
    //watchdog->IsAlive();

    std::vector<double> bisectrix(4);
    bisectrix[0] = -15;
    bisectrix[1] = 15;
    bisectrix[2] = -(biMsg.bisectrix[2] + biMsg.bisectrix[0]*bisectrix[0])/biMsg.bisectrix[1];
    bisectrix[3] = -(biMsg.bisectrix[2] + biMsg.bisectrix[0]*bisectrix[1])/biMsg.bisectrix[1];

    if(v_linear < max_v_linear){ // gradually increasing speed
        v_linear = max_v_linear * (ros::Time::now() - start_time).toSec() / ramp_time.toSec();
    }

    /**************************************************************************************/
    /*Control Function*/
    double rudder;
    rudder = Control::LineTracking(bisectrix, v_linear, angularVel, dt, KPT, KIT, KRT, KVT);
    /**************************************************************************************/

    dt = ros::Time::now().toSec();

    aux::moveWindow(rudder, aux::Rudder, aux::sizeRudder);
    //ROS_INFO_STREAM("Instantaneous rudder value: " << rudder);
    //ROS_INFO_STREAM("Average rudder value:       " <<aux::getAverage(aux::Rudder, aux::sizeRudder));

    if(which_car.compare("vero") == 0){
        ROS_INFO_STREAM("Command send to VERO");
        ransac_project::CarCommand msgvero;
        msgvero.speedLeft  = v_linear;
        msgvero.speedRight = v_linear;
        msgvero.steerAngle = aux::getAverage(aux::Rudder, aux::sizeRudder);
        pub->publish(msgvero);
    }
    else{
        ROS_INFO_STREAM("Command send to PIONEER");
        geometry_msgs::Twist msgpionner;
        msgpionner.linear.x = v_linear ; msgpionner.linear.y = 0.0;  msgpionner.linear.z = 0.0;
        msgpionner.angular.x = 0.0;      msgpionner.angular.y = 0.0; aux::getAverage(aux::Rudder, aux::sizeRudder);
        pub->publish(msgpionner);
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

    ros::Subscriber subOdom = n.subscribe(VERO_ODOMEMTRY_TOPIC, 1, &ransacControl::odometryCallback, rc);
    ros::Subscriber subBise = n.subscribe(RANSAC_BISECTRIX_TOPIC, 1, &ransacControl::ransacCallback, rc);
    ros::spin();

    return 0;
}
