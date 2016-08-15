#ifndef RANSAC_LIB_H
#define RANSAC_LIB_H

/* Standard Libraries*/
#include <math.h>
#include <vector>
#include <time.h>
#include <signal.h>
#include <string.h>
#include <iostream>
#include <tr1/functional>

/* Mobile Robot Programming Toolkit*/
#include <mrpt/gui.h> // To plots

/* ROS */
#include "ros/ros.h"
#include "tf/transform_listener.h"

/* ROS msgs*/
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

/* User msgs*/
#include "ransac_project/BorderLines.h"
#include "ransac_project/CarCommand.h"
#include "ransac_project/Bisectrix.h"

using namespace ros;
using namespace std;
using namespace std::tr1;
using namespace tf;
using namespace mrpt::gui;

#define PI 3.14159265
#define DURATION 10 

/* bisectrixLine
 *    Calcula a linha bissetriz entre duas linhas, cada uma delas
 * definida por dois pontos
 *	
 * Entradas
 *    l1 : vetor contendo os coeficientes da primeira linha (a1, b1, c1)
 *    l2 : vetor contendo os coeficientes da segunda linha (a2, b2, c2)
 *g	
 * Saidas
 *    bisectrix	: vetor contendo os coeficientes que definem a linha bissetriz (ab, bb, cb)
*/
vector<double> bisectrixLine(vector<float> l1, vector<float> l2);


/* plotLine
 *    plota uma linha
 *	
 *	Entradas
 *    win		 : ponteiro para a janela onde a linha deve ser plotada
 *    line 	 : vetor contendo os coeficientes da primeira linha (a, b, c)
 *    format : string com o formato da linha a ser plotada
 *    name	 : string com o nome da linha a ser plotada
*/
void plotLine(CDisplayWindowPlots &win,
                const vector<float> &line, 
                const string &lineFormat, 
                const string &plotName);


/* plotPoints
 *    plota pontos 
 *	
 *	Entradas
 *    win		 : ponteiro para a janela onde a linha deve ser plotada
 *    x      : vetor contendo pontos que serao plotados
 *    y      : vetor contendo pontos que serao plotados
 *    format : string com o formato dos pontos  a ser plotada
 *    name	 : string com o nome dos pontos a ser plotada
*/
void plotPoints(CDisplayWindowPlots &win, 
                  const vector<float> &x,
                  const vector<float> &y, 
                  const string &lineFormat, 
                  const string &plotName);
#endif /* RANSAC_LIB_H */
