#include "controlPIV.hpp"

/* *************************NOTAS************************
A variável ControlPIV::errori_, que armazena o erro integral, deve ser declarada
fora desta função.
*/

double ControlPIV::errori = 0.0;

double ControlPIV::LineTracking(const std::vector<double> &line,
                             const double &v_linear, const double &v_angular,
                             const double &dt,
                             const double &KPT, const double &KIT, const double &KRT, const double &KVT){
    double trans[2], dist[2];
    double dist_val, head, rudder;

    double vel[2], desv_vel[2];
    double dd  = 0; // perpendicular distante between the vehicle and the trajectory(effective error)
    double ddv = 0; // perpendicular velocity of the vehicle in relation the trajectory
    double dd1 = 0; // perpendicular travelled distance by the vehicle in relation the trajectory
    double psirefc = 0;
    double TSAMPLETRAJ = ros::Time::now().toSec() - dt;

    /* COMPUTING dd, ddv, dd1*/
    // setting that the vehicle is aligned with the EAST
    double eastv  =  v_linear; // along the vehicle
    double northv =  0;        // perpendicular the vehicle
    //ROS_INFO_STREAM("eastv= " << eastv << " northv= " << northv << " |v|= " <<
    //                sqrt(eastv*eastv + northv*northv));

    // transition trajectory vector
    trans[0] = line[1] - line[0];
    trans[1] = line[3] - line[2];
    double n = sqrt(trans[0]*trans[0] + trans[1]*trans[1]); // transition trajectory vector normalized
    trans[0]/=n;trans[1]/=n;
    //ROS_INFO_STREAM("normalized trans = (" << trans[0] << ", " << trans[1] << ") " <<
    //                "|trans|= " << sqrt(trans[0]*trans[0] + trans[1]*trans[1]) <<
    //                " alpha=" << acos(trans[0]));

    // get distances
    dist[0] =  line[1]; /* - 0*/
    dist[1] =  line[3]; /* - 0*/
    //ROS_INFO_STREAM("dist = (" << dist[0] << ", " << dist[1] << ")");

    // perpendicular distance (effective error)
    dd = trans[0]*dist[1] - trans[1]*dist[0]; // cross product trans x dist
    //ROS_INFO_STREAM("dd = " << dd);

    // distance along the trajectory from the vehicle to the target
    dist_val = dist[0]*trans[0] + dist[1]*trans[1]; // inner product
    //ROS_INFO_STREAM("dist_val = " << dist_val);

    // vehicle velocities
    vel[0] = eastv;  // along the vehicle
    vel[1] = northv; // perpendiculer the vehicle
    //ROS_INFO_STREAM("vel = (" << vel[0] << ", " << vel[1] << ")");

    // component velocity perpendicular to the trajectory
    ddv = trans[0]*vel[1] - trans[1]*vel[0];
    //ROS_INFO_STREAM("ddv = " << ddv);

    // vehicle displacement
    desv_vel[0] = vel[0]* TSAMPLETRAJ; // parallel the vehicle
    desv_vel[1] = vel[1]* TSAMPLETRAJ; // perpendicular the vehicle
    //ROS_INFO_STREAM("desv_vel = (" << desv_vel[0] << ", " << desv_vel[1] << ")");

    // vehicle displacement perpendicular to the trajectory
    dd1 =  trans[0]*desv_vel[1] - trans[1]*desv_vel[0];
    //ROS_INFO_STREAM("dd1 = " << dd1);

    /* COMPUTING THE CONTROL SIGN*/
    ControlPIV::errori = ControlPIV::errori + dd*TSAMPLETRAJ;

    if (ControlPIV::errori >  27) ControlPIV::errori =  27;
    if (ControlPIV::errori < -27) ControlPIV::errori = -27;
    psirefc = KPT*dd + KVT*ddv + KIT*ControlPIV::errori;
    //ROS_INFO_STREAM("perr = " << dd  << " derr = " << ddv << " ierr = " << ControlPIV::errori);
    //ROS_INFO_STREAM("psirefc = " << psirefc);

    psirefc = psirefc>= 90? 89:psirefc;
    psirefc = psirefc<=-90?-89:psirefc;

    rudder = psirefc;
    rudder = KRT * rudder;

    rudder = (rudder>20?20:(rudder<-20?-20:rudder));
    
    // steering angle
    rudder = rudder*PI/180;
    //ROS_INFO_STREAM("rudder = " << rudder);

    return rudder;

} // LineTracking
