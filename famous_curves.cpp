//// g++ 3D_line_for_RRT.cpp -o t -I/usr/include/python3.8 -lpython3.8

#include <iostream>
#include <vector>
#include <tuple>
#include <math.h>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

//-----------------------------------------------------------

void plot2D(std::tuple<std::vector<float>, std::vector<float>> data)
{

    std::vector<float> xX = std::get<0>(data);
    std::vector<float> yY = std::get<1>(data);

    plt::plot(xX, yY);
    plt::xlabel("x");
    plt::ylabel("y");
    plt::show();
}
//-----------------------------------------------------------

std::tuple<std::vector<float>, std::vector<float>> computeCardioid()
{

    float a = 0.7;
    float rad = M_PI / 180;
    int size = 360;

    std::vector<float> xX;
    std::vector<float> yY;

    for (int ii = 0; ii < size; ii++)
    {

        xX.push_back(2 * a * (1 - std::cos(rad * ii)) * std::cos(rad * ii));
        yY.push_back(2 * a * (1 - std::cos(rad * ii)) * std::sin(rad * ii));
    }

    return std::make_tuple(xX, yY);
}

//-----------------------------------------------------------

std::tuple<std::vector<float>, std::vector<float>> computeAstroind()
{

    float a = 0.7;
    float rad = M_PI / 180;
    int size = 360;

    std::vector<float> xX;
    std::vector<float> yY;

    for (int ii = 0; ii < size; ii++)
    {

        xX.push_back(a * (3 * std::cos(rad * ii) + std::cos(3 * rad * ii)) / 4);
        yY.push_back(a * (3 * std::sin(rad * ii) - std::sin(3 * rad * ii)) / 4);
    }

    return std::make_tuple(xX, yY);
}

//-----------------------------------------------------------

std::tuple<std::vector<float>, std::vector<float>> computeBicorn()
{

    float a = 0.7;
    float rad = M_PI / 180;
    int size = 180;

    std::vector<float> xX;
    std::vector<float> yY;

    for (float ii = -M_PI; ii < M_PI; ii += rad)
    {
        xX.push_back(a * std::sin(ii));
        yY.push_back(a * ((std::cos(ii) * std::cos(ii) * (2 + std::cos(ii))) / (3 + std::sin(ii) * std::sin(ii))));
    }

    return std::make_tuple(xX, yY);
}

//-----------------------------------------------------------

std::tuple<std::vector<float>, std::vector<float>> computeCayleySextic()
{

    float rad = M_PI / 180;
    int size = 180;

    std::vector<float> xX;
    std::vector<float> yY;

    for (float ii = -M_PI; ii < M_PI; ii += rad)
    {
        xX.push_back(std::pow(std::cos(ii), 3) * std::cos(3 * ii));
        yY.push_back(std::pow(std::cos(ii), 3) * std::sin(3 * ii));
    }

    return std::make_tuple(xX, yY);
}

//-----------------------------------------------------------

std::tuple<std::vector<float>, std::vector<float>> computeCycloid()
{

    float r = 0.7;
    float rad = M_PI / 180;
    int size = 180 * 10;

    std::vector<float> xX;
    std::vector<float> yY;

    for (int ii = 0; ii < size; ii++)
    {
        xX.push_back(r * (ii * rad - std::sin(ii * rad)));
        yY.push_back(r * (1 - std::cos(ii * rad)));
    }

    return std::make_tuple(xX, yY);
}

//-----------------------------------------------------------

std::tuple<std::vector<float>, std::vector<float>> computeDevilCurve()
{

    float a = std::sqrt(1);
    float b = std::sqrt(2);

    float rad = M_PI / 180;
    int size = 180 * 1;

    std::vector<float> xX;
    std::vector<float> yY;

    for (int ii = -size; ii < size; ii++)
    {

        float xi = std::cos(ii * rad) * std::sqrt((a * a * std::pow(std::sin(ii * rad), 2) - b * b * std::pow(std::cos(ii * rad), 2)) / (std::pow(std::sin(ii * rad), 2) - std::pow(std::cos(ii * rad), 2)));
        float yi = std::sin(ii * rad) * std::sqrt((a * a * std::pow(std::sin(ii * rad), 2) - b * b * std::pow(std::cos(ii * rad), 2)) / (std::pow(std::sin(ii * rad), 2) - std::pow(std::cos(ii * rad), 2)));
        xX.push_back(xi);
        yY.push_back(yi);
    }

    return std::make_tuple(xX, yY);
}

//-----------------------------------------------------------

std::tuple<std::vector<float>, std::vector<float>> computeEpiCycloid()
{

    float r = 0.3;
    float R = 1.1;
    float rad = M_PI / 180;
    int size = 180 * 10;

    std::vector<float> xX;
    std::vector<float> yY;

    for (int ii = 0; ii < size; ii++)
    {
        xX.push_back((r + R) * std::cos(ii * rad) - r * std::cos(ii * rad * ((R + r) / r)));
        yY.push_back((r + R) * std::sin(ii * rad) - r * std::sin(ii * rad * ((R + r) / r)));
    }

    return std::make_tuple(xX, yY);
}

//-----------------------------------------------------------

std::tuple<std::vector<float>, std::vector<float>> computeEpitrochoid()
{

    float r = 0.6;
    float R = 1.1;
    float d = 0.8;
    float rad = M_PI / 180;
    int size = 180 * 20;

    std::vector<float> xX;
    std::vector<float> yY;

    for (int ii = 0; ii < size; ii++)
    {
        xX.push_back((r + R) * std::cos(ii * rad) - d * std::cos(ii * rad * ((R + r) / r)));
        yY.push_back((r + R) * std::sin(ii * rad) - d * std::sin(ii * rad * ((R + r) / r)));
    }

    return std::make_tuple(xX, yY);
}

//-----------------------------------------------------------

std::tuple<std::vector<float>, std::vector<float>> computeHypoCycloid()
{

    float r = 0.3;
    float R = 1.1;
    float rad = M_PI / 180;
    int size = 180 * 10;

    std::vector<float> xX;
    std::vector<float> yY;

    for (int ii = 0; ii < size; ii++)
    {
        xX.push_back((R - r) * std::cos(ii * rad) + r * std::cos(ii * rad * ((R - r) / r)));
        yY.push_back((R - r) * std::sin(ii * rad) - r * std::sin(ii * rad * ((R - r) / r)));
    }

    return std::make_tuple(xX, yY);
}

//-----------------------------------------------------------

std::tuple<std::vector<float>, std::vector<float>> computeDeltoid()
{

    float r = 0.3;
    float R = 3 * r;
    float rad = M_PI / 180;
    int size = 180 * 10;

    std::vector<float> xX;
    std::vector<float> yY;

    for (int ii = 0; ii < size; ii++)
    {
        xX.push_back((R - r) * std::cos(ii * rad) + r * std::cos(ii * rad * ((R - r) / r)));
        yY.push_back((R - r) * std::sin(ii * rad) - r * std::sin(ii * rad * ((R - r) / r)));
    }

    return std::make_tuple(xX, yY);
}

int main()
{

    std::tuple<std::vector<float>, std::vector<float>> cardioid = computeCardioid();
    std::tuple<std::vector<float>, std::vector<float>> astroid = computeAstroind();
    std::tuple<std::vector<float>, std::vector<float>> bicorn = computeBicorn();
    std::tuple<std::vector<float>, std::vector<float>> sextic = computeCayleySextic();
    std::tuple<std::vector<float>, std::vector<float>> cycloid = computeCycloid();
    std::tuple<std::vector<float>, std::vector<float>> devil = computeDevilCurve();
    std::tuple<std::vector<float>, std::vector<float>> epicycloid = computeEpiCycloid();
    std::tuple<std::vector<float>, std::vector<float>> epitochroid = computeEpitrochoid();
    std::tuple<std::vector<float>, std::vector<float>> hypocycloid = computeHypoCycloid();
    std::tuple<std::vector<float>, std::vector<float>> deltoid = computeDeltoid();
    plot2D(cardioid);
    plot2D(astroid);
    plot2D(bicorn);
    plot2D(sextic);
    plot2D(cycloid);
    plot2D(devil);
    plot2D(epicycloid);
    plot2D(epitochroid);
    plot2D(hypocycloid);
    plot2D(deltoid);
}
