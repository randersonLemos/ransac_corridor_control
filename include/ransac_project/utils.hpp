#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <vector>
#include <iostream>
#include <mrpt/gui.h>

/**
 * Retorna os coeficientes (a, b, c) da reta bissetriz no formato
 * ax + by + c = 0. Seus coeficientes estão no mesmo formato
 * que os coeficientes da reta bissetriz, i.e. ax + by + c = 0.
 *
 * @param l1 Coeficientes da reta para o cálculo da bissetriz
 * @param l2 Coeficientes da reta para o cálculo da bissetriz
 * @return Os coeficientes da reta bissetriz
*/
std::vector<double> bisectrixLine(std::vector<float> l1, std::vector<float> l2);

namespace plot{
/**
 * Imprimi dinamicamente retas ax + by + c = 0 de coeficienetes
 * (a, b, c)
 *
 * @param win Ponteiro da janela onde a reta será imprimida
 * @param line Coeficientes da reta que será imprimida 
 * @param lineFormat Imformações do formata da reta que será impressa
 * @param plotName Nome da linha que será impressa
*/
void Line(mrpt::gui::CDisplayWindowPlots &win, const std::vector<float> &line,
          const std::string &lineFormat, const std::string &plotName);

/**
 * Imprimi dinamicamente pontos (x,y)
 * 
 * @param win Ponteiro da janela onde os pontos serão impressos
 * @param x Coordenada x dos pontos que serão impressos
 * @param y Coordenada y dos pontos que serão impressos
 * @param Informações do formato dos pontos que serão impressos
 * @param plotName Nome dos pontos que serão impressos
*/
void Points(mrpt::gui::CDisplayWindowPlots &win, const std::vector<float> &x,
            const std::vector<float> &y, const std::string &lineFormat, 
            const std::string &plotName);
}
#endif /* UTILS_H */
