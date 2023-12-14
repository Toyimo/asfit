#include <fstream>
#include <sstream>
#include <vector>
#include <list>
#include <stdio.h>
#include <iostream>
#include <math.h>

#include "alglib_spline_fitting.h"

/* Split string by token, and save it into a list.*/
template<typename T>
bool split(const std::string& input, std::list<T>& result, const char& token = ',' )
{
    std::istringstream ss(input);
    std::string element;
    result.clear();
    while(std::getline(ss, element, token)){
        try
        {
            if (std::is_same<T, double>::value) {
                double value = std::stod(element);
                result.push_back(value);
            } else if (std::is_same<T, float>::value) {
                float value = std::stof(element);
                result.push_back(value);
            } else if (std::is_same<T, int>::value) {
                int value = std::stoi(element);
                result.push_back(value);
            }
            // } else if (std::is_same<T, std::string>::value) {
            //     result.push_back(element);
            // }
            else {
                std::cout << "ERROR.split(): the list type is unknown." << std::endl;
                return false;
            }            
        }
        catch(const std::exception& e)
        {
            std::cerr << "ERROR.split(): split failed, " << e.what() << ".\n";
            return false;
        }
    }
    return true;
}

/* Calculate length by x and y.*/
inline double length(const double& x0, const double& y0, const double& x1, const double& y1)
{
    return std::sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
}

/* Read points from txt and save to list<double>.*/
bool reader(const std::string& file_path, std::list<double>& x, std::list<double>& y, std::list<double>& z)
{
    std::ifstream file(file_path);
    std::string line;
    if(file.is_open()) {
        std::list<double> coords;
        while(std::getline(file, line)){
            if(split<double>(line, coords, ' ')){
                if(coords.size() < 3){
                    continue;
                }
                auto iter = coords.begin();
                x.push_back(*iter);
                ++iter;
                y.push_back(*iter);
                ++iter;
                z.push_back(*iter);
            }
        }
        file.close();
        if(x.size() != y.size() || y.size() != z.size()){
            std::cout << "ERROR.reader(): x y z size are not equal.\n";
            return false;
        }
        return true;
    } else{
        std::cout << "ERROR.reader(): unable to open file.\n";
        return false;
    }
}

/* Save data to vector.*/
bool convertor(std::list<double>& src, std::vector<double>& target)
{
    target.reserve(src.size());
    for(auto item : src){
        target.push_back(item);
    }
    src.clear();
    return true;
}

/* Calculate s array by x and y list.*/
bool calculator(std::vector<double>& x, std::vector<double>& y, std::vector<double>& s)
{
    if(x.size() != y.size() || x.size() <= 1 || y.size() <= 1){
        std::cout << "ERROR.calculator(): x.size() or y.size() is unvalid.\n";
        return false;
    }
    s.reserve(x.size());
    s.push_back(0);
    auto xiter = x.begin();
    auto yiter = y.begin();
    for(int i = 0; i < x.size() - 1; i++){
        double x0 = *xiter;
        double y0 = *yiter;
        ++xiter;
        ++yiter;
        double x1 = *xiter;
        double y1 = *yiter;
        s.push_back(s.back() + length(x0, y0, x1, y1));
    }
    return true;
}

int main(int argc, char **argv)
{
    //
    // In this example we demonstrate penalized spline fitting of noisy data
    //
    // We have:
    // * x - abscissas
    // * y - vector of experimental data, straight line with small noise
    //
    int base_function_num = 50;
    double lambdans = 1e-3;

    // step 01. read and convert the data
    std::list<double> x_list, y_list, z_list;
    std::vector<double> x_vec, y_vec, z_vec, s_vec;
    if(!reader("lane_pointcloud.txt", x_list, y_list, z_list)) return 0;
    convertor(x_list, x_vec);
    convertor(y_list, y_vec);
    convertor(z_list, z_vec);

    // step 02. use class
    std::vector<std::vector<double>> result;
    AlglibSplineFitting splinefitting;
    if(!splinefitting.fitting(x_vec, y_vec, z_vec, result, AlglibSplineFitting::ASF_NORMAL)){
        std::cout << "ERROR: alglib spline fitting failed.\n";
    }

    std::ofstream myfile("output.txt");
    if (myfile.is_open()) {
        for(int i = 0; i < result.at(0).size(); i++){
            double xi = result.at(0).at(i);
            double yi = result.at(1).at(i);
            double zi = result.at(2).at(i);
            myfile << 
                std::to_string(xi) << " " << 
                std::to_string(yi) << " " << 
                std::to_string(zi) << " " << 
                "\n";
        }
        myfile.close();
        std::cout << "File written successfully.\n";
    } else {
        std::cout << "Unable to open file.\n";
    }
    return 0;
}