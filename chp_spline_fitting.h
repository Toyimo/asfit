// @Description: Generate Parameterized Spline Curve with Unordered Point
// @Time       : 2023/12/25 10:15
// @Author     : tongjx

#pragma once

#include <vector>
#include "utils/geometry.h"

class ConcaveHullParamSplineFitting
{
public:
    ConcaveHullParamSplineFitting(){}
    ~ConcaveHullParamSplineFitting(){}

public:
    /* Control the fitting error, 
       fit the point better when the value is smaller.*/
    double& lambdans(){ return _lambdans; }
    /* Control how much control points for the spline, 
       provides more degrees of shape curvature when the value is larger.*/
    double& base_function_num() {return _base_function_num; }
    /* Control the concave shape, 
       the shape is roupher when the value is larger.*/
    double& concave_lambdans(){ return _concave_lambdans; }
    
public:
    /**
     * FITTING
     * 
     * Description: 
     *    generate a spline [[x], [y], [z]] with given points [x], [y], [z]
     *    NOTE: 1. x.size() == y.size() == z.size() = n is necessary
     *          2. the coordinate system MUST be UTM
     * Parameters:
     *    @xarray:  x coordinates
     *    @yarray:  y coordinates
     *    @zarray:  z coordinates
     *    @result:  [[x], [y], [z]] spline with 3*n dimension
     *    @mode:    ASF_PARAM as default, which spline will be created: ASF_PARAM or ASF_NORMAL
     *    @density: 1.0m as default, generate points every 1.0 meter
     * Return:
     *    ture if fitting successs, otherwise return false
    */
    bool fitting(
        const std::vector<double>& xarray, 
        const std::vector<double>& yarray, 
        const std::vector<double>& zarray,
        std::vector<std::vector<double>>& result,
        double density = 1.0
    );

private:
    bool generate_concave_hull(const std::vector<double>& pcl_points, std::vector<asfit::Point>& concave_geom);
    bool generate_reference_line_with_concave_hull(std::vector<asfit::Point>& concave_geom, asfit::Polyline& reference_line);
    bool projection(const asfit::Polyline& reference_line, const std::vector<asfit::Point>& pcl_points, std::vector<asfit::Point>& projected_pcl_points);
    bool fitting_pcl_points(std::vector<asfit::Point>& projected_pcl_points, std::vector<std::vector<double>>& result, const double& density);

private:
    double _concave_lambdans = 5e-2;
    double _lambdans = 1e-4;
    double _base_function_num = 30;
};