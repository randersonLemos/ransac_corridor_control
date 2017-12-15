#include "utils.hpp"

std::vector<float> utils::bisectrixLine(std::vector<float> l1, std::vector<float> l2){

    std::vector<float> bisectrix(3), V1(2), V2(2), VY(2);
    float a1, b1, c1, a2, b2, c2;
    float a1k, b1k, c1k, a2k, b2k, c2k, k1, k2;
    float bi1_a, bi1_b, bi1_c, bi2_a, bi2_b, bi2_c, theta1, theta2, modV1, modV2, modVY;

    a1 = l1[0]; b1 = l1[1]; c1 = l1[2];
    a2 = l2[0]; b2 = l2[1]; c2 = l2[2];

    k1 = sqrt(a1*a1 + b1*b1);
    k2 = sqrt(a2*a2 + b2*b2);

    a1k = a1/k1; b1k = b1/k1; c1k = c1/k1;
    a2k = a2/k2; b2k = b2/k2; c2k = c2/k2;

    bi1_a = (a1k - a2k); bi1_b = (b1k - b2k); bi1_c = (c1k - c2k);
    bi2_a = (a1k + a2k); bi2_b = (b1k + b2k); bi2_c = (c1k + c2k);

    // seleciona a bissetriz com menor angulo em relacao ao eixo Y
    // evitando divisoes por zero

    if(bi1_b != 0){
        V1[0] = 10;
        V1[1] = ((-bi1_c - V1[0]*bi1_a)/bi1_b) - (-bi1_c/bi1_b);
    }
    else if(bi1_a != 0){
        V1[1] = 10;
        V1[0] = ((-bi1_c - V1[1]*bi1_b)/bi1_a) - (-bi1_c/bi1_a);
    }

    if(bi2_b != 0){
        V2[0] = 10;
        V2[1] = ((-bi2_c - V2[0]*bi2_a)/bi2_b) - (-bi2_c/bi2_b);
    }
    else if(bi2_a != 0){
        V2[1] = 10;
        V2[0] = ((-bi2_c - V2[1]*bi2_b)/bi2_a) - (-bi2_c/bi2_a);
    }

    VY[0] = 0;
    VY[1] = 10;

    modV1 = sqrt(V1[0]*V1[0] + V1[1]*V1[1]);
    modV2 = sqrt(V2[0]*V2[0] + V2[1]*V2[1]);
    modVY = sqrt(VY[0]*VY[0] + VY[1]*VY[1]);

    theta1 = acos(std::abs(V1[0]*VY[0] + V1[1]*VY[1])/std::abs(modV1*modVY));
    theta2 = acos(std::abs(V2[0]*VY[0] + V2[1]*VY[1])/std::abs(modV2*modVY));

    if(std::abs(theta1)>=std::abs(theta2)){
        bisectrix[0] = bi1_a;
        bisectrix[1] = bi1_b;
        bisectrix[2] = bi1_c;
    }
    else{
        bisectrix[0] = bi2_a;
        bisectrix[1] = bi2_b;
        bisectrix[2] = bi2_c;
    }

    return bisectrix;
} /* bisectrixLine */

std::vector<float> utils::fromThree2TwoCoeffs(std::vector<float> _coeffs){
    std::vector<float> coeffs(2);
    coeffs[0] = -_coeffs[0]/(_coeffs[1] + 1e-6); // Angular coefficient
    coeffs[1] = -_coeffs[2]/(_coeffs[1] + 1e-6); // Linear coefficient
    return coeffs;
}

std::vector<float> utils::fromTwo2ThreeCoeffs(std::vector<float> _coeffs){
    std::vector<float> coeffs(3);
    coeffs[0] = -_coeffs[0];
    coeffs[1] = 1.0;
    coeffs[2] = -_coeffs[1];
    return coeffs;
}

void utils::addLineToPointcloud(std::vector<float> coeffs, pcl::PointCloud<pcl::PointXYZ>& line){

    float a, b, c;
    a = coeffs[0];
    b = coeffs[1];
    c = coeffs[2];

    for(int i = 0; i < 30; ++i){
        pcl::PointXYZ p;
        p.x = i/2.0;
        p.y = -(c+a*p.x)/b;
        p.z = 0.0;
        line.push_back(p);
    }
}
