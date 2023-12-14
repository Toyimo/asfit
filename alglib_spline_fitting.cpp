#include "alglib_spline_fitting.h"
#include "interpolation.h"

#include <iostream>
#include <algorithm>
#include <math.h>

using namespace alglib;

inline double length(const double& x0, const double& y0, const double& x1, const double& y1)
{ 
    return std::sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0)); 
}

bool AlglibSplineFitting::fitting(
    const std::vector<double>& xarray, 
    const std::vector<double>& yarray, 
    const std::vector<double>& zarray,
    std::vector<std::vector<double>>& result,
    ASFMode mode,
    int density)
{
    if(mode == ASF_PARAM){
        return fitting_param(xarray, yarray, zarray, result, density); 
    }else{
        return fitting_normal(xarray, yarray, zarray, result, density);
    }
}

bool AlglibSplineFitting::fitting_param(
    const std::vector<double>& xarray, 
    const std::vector<double>& yarray, 
    const std::vector<double>& zarray,
    std::vector<std::vector<double>>& result,
    int density)
{
    // step 01. calculate sarray
    std::vector<double> sarray;
    if(!calculator(xarray, yarray, sarray)) {
        return false;
    }

    // step 02. convert to real_1d_array
    real_1d_array x, y, z, s;
    int n = xarray.size();
    x.setcontent(n, xarray.data());
    y.setcontent(n, yarray.data());
    z.setcontent(n, zarray.data());
    s.setcontent(n, sarray.data());

    // step 03. begin spline 1d fit
    spline1dinterpolant spline_x, spline_y, spline_z;
    spline1dfitreport rep_x, rep_y, rep_z;
    spline1dfit(s, x, _base_function_num, _lambdans, spline_x, rep_x);
    spline1dfit(s, y, _base_function_num, _lambdans, spline_y, rep_y);
    spline1dfit(s, z, _base_function_num, _lambdans, spline_z, rep_z);

    // step 04. prepare parameters
    double smin = 0;
    double smax = sarray.back();
    int cnt = (int)(smax / density);
    double step = (smax - smin) / cnt;

    // step 05. calculate the spline with density
    result.resize(3, std::vector<double>(0.0));
    result.at(0).reserve(cnt + 1);
    result.at(1).reserve(cnt + 1);
    result.at(2).reserve(cnt + 1);
    for(int i = 0; i <= cnt; i++){
        double si = smin + i * step;
        result.at(0).push_back(spline1dcalc(spline_x, si));
        result.at(1).push_back(spline1dcalc(spline_y, si));
        result.at(2).push_back(spline1dcalc(spline_z, si));
    }
    return true;
}

bool AlglibSplineFitting::fitting_normal(
    const std::vector<double>& xarray, 
    const std::vector<double>& yarray, 
    const std::vector<double>& zarray,
    std::vector<std::vector<double>>& result,
    int density)
{
    // step 01. convert to real_1d_array
    real_1d_array x, y, z;
    int n = xarray.size();
    x.setcontent(n, xarray.data());
    y.setcontent(n, yarray.data());
    z.setcontent(n, zarray.data());

    // step 02. begin spline 1d fit
    spline1dinterpolant spline_y, spline_z;
    spline1dfitreport rep_y, rep_z;
    spline1dfit(x, y, _base_function_num, _lambdans, spline_y, rep_y);
    spline1dfit(x, z, _base_function_num, _lambdans, spline_z, rep_z);

    // step 03. prepare parameters
    double xmin = *std::min_element(xarray.begin(), xarray.end());
    double xmax = *std::max_element(xarray.begin(), xarray.end());
    int cnt = (int)((xmax - xmin) / density);
    if(cnt == 0){
        std::cout 
            << "ERROR.fittting_normal(): cnt is zero with xmin,xmax=" 
            << std::to_string(xmin) << "," << std::to_string(xmax)<< ".\n";
            return false;
    }
    double step = (xmax - xmin) / cnt;

    // step 04. calculate the spline with density
    result.resize(3, std::vector<double>(0.0));
    result.at(0).reserve(cnt + 1);
    result.at(1).reserve(cnt + 1);
    result.at(2).reserve(cnt + 1);
    for(int i = 0; i <= cnt; i++){
        double xi = xmin + i * step;
        result.at(0).push_back(xi);
        result.at(1).push_back(spline1dcalc(spline_y, xi));
        result.at(2).push_back(spline1dcalc(spline_z, xi));
    }
    return true;
}

bool AlglibSplineFitting::calculator(
    const std::vector<double>& x, 
    const std::vector<double>& y, 
    std::vector<double>& s)
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
    if(s.size() <= 0) return false;
    return true;
}