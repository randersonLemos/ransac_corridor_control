#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <vector>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace utils{
/**
 * Retorna os coeficientes (a, b, c) da reta bissetriz no formato
 * ax + by + c = 0. Seus coeficientes estão no mesmo formato
 * que os coeficientes da reta bissetriz, i.e. ax + by + c = 0.
 *
 * @param l1 Coeficientes da reta para o cálculo da bissetriz
 * @param l2 Coeficientes da reta para o cálculo da bissetriz
 * @return Os coeficientes da reta bissetriz
*/
std::vector<float> bisectrixLine(std::vector<float> l1, std::vector<float> l2);

std::vector<float> fromThree2TwoCoeffs(std::vector<float> _coeffs);

std::vector<float> fromTwo2ThreeCoeffs(std::vector<float> _coeffs);

void addLineToPointcloud(std::vector<float> coeffs, pcl::PointCloud<pcl::PointXYZ>& line);

}

#endif /* UTILS_H */
