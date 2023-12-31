// @Description: Spline Fitting with Penalized Regression
// @Time       : 2023/12/13 15:36
// @Author     : tongjx

#pragma once
#include <vector>

/** 
 * ALGLIBSPLINE FITTING
 * 
 * Description:
 *    Spline Fitting with Penalized Regression
 * Parameters:
 *    @lambdans: 1e-3 as default, understanding as offset error
 *    @base_function_num: 50 as default, base function number of spline
 *                        this parameter determines the fineness of the curve
*/
class AlglibSplineFitting
{
public:
    typedef enum {ASF_PARAM, ASF_NORMAL, ASF_CUSTOM_PARAM} ASFMode;

public:
    AlglibSplineFitting(){}
    ~AlglibSplineFitting(){}

public:
    /* Control the fitting error, 
       fit the point better when the value is smaller.*/
    double& lambdans(){ return _lambdans; }
    /* Control how much control points for the spline, 
       provides more degrees of shape curvature when the value is larger.*/
    double& base_function_num() {return _base_function_num; }

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
        ASFMode mode = ASF_PARAM,
        double density = 1.0
    );

    /**
     * FITTING
     * 
     * Description: 
     *    generate a spline [[x], [y], [z]] with given points [x], [y], [z]
     *    NOTE: 1. x.size() == y.size() == z.size() == s.size() = n is necessary
     *          2. the coordinate system MUST be UTM
     * Parameters:
     *    @xarray:  x coordinates
     *    @yarray:  y coordinates
     *    @zarray:  z coordinates
     *    @sarray:  prameter function s coordinate
     *    @result:  [[x], [y], [z]] spline with 3*n dimension
     *    @density: 1.0m as default, generate points every 1.0 meter
     * Return:
     *    ture if fitting successs, otherwise return false
    */    
    bool fitting(
        const std::vector<double>& xarray, 
        const std::vector<double>& yarray, 
        const std::vector<double>& zarray,
        const std::vector<double>& sarray,
        std::vector<std::vector<double>>& result,
        double density = 1.0
    );

private:
    bool fitting_param(
        const std::vector<double>& xarray, 
        const std::vector<double>& yarray, 
        const std::vector<double>& zarray,
        std::vector<std::vector<double>>& result,
        double density
    );
    bool fitting_normal(
        const std::vector<double>& xarray, 
        const std::vector<double>& yarray, 
        const std::vector<double>& zarray,
        std::vector<std::vector<double>>& result,
        double density
    );
    bool calculator(
        const std::vector<double>& x, 
        const std::vector<double>& y, 
        std::vector<double>& s
    );

private:
    double _lambdans = 1e-4;
    double _base_function_num = 30;
};