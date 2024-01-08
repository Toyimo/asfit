#include "chp_spline_fitting.h"
#include "alglib_spline_fitting.h"
#include "concavehull/concavehull.hpp"

#include <iostream>
#include <list>
#include <tuple>
#include <unordered_set>
#include <unordered_map>
#include <fstream>

/* Less Function */
struct PointLess
{
    bool operator()(const asfit::Point &a, const asfit::Point &b) const
    {
        return a.attributes.at("s") < b.attributes.at("s");
    }
};

/* Split ordered vector with tolerance. */
std::list<std::tuple<size_t, size_t>> split_vector_with_order_tolerance(
    const std::vector<size_t>& vec, 
    const size_t& tolerance)
{
    std::list<std::tuple<size_t, size_t>> res;
    if(tolerance >= vec.size()) return res;
    if(vec.size() < 1) return res;
    size_t i = 0, j = i + 1;
    size_t start = i;
    while(j < vec.size()){
        while(vec[j] - vec[i] <= tolerance){
            ++j;
            ++i;
            if(j >= vec.size()){
                break;
            }
        }
        res.push_back(std::make_tuple(start, j - 1));
        i = j;
        j = i + 1;
        start = i;
    }
    if (i != vec.size()) {
        res.push_back(std::make_tuple(start, vec.size() - 1));
    }
    return res;
}

bool ConcaveHullParamSplineFitting::fitting(
    const std::vector<double>& xarray, 
    const std::vector<double>& yarray, 
    const std::vector<double>& zarray,
    std::vector<std::vector<double>>& result,
    double density)
{
    // step 00. check value
    if(xarray.size() != yarray.size() || yarray.size() != zarray.size() || xarray.size() < 2){
        std::cout << "ERROR.chp_spline_fitting.cpp::fitting(): pcl array size is invalid.\n";
        return false;
    }

    std::vector<double> pointsets;
    std::vector<asfit::Point> concave_geom;
    std::vector<asfit::Point> pcl_points, projected_pcl_points;
    asfit::Polyline reference_line;

    // step 01. prepare the data
    pcl_points.reserve(xarray.size());
    pointsets.reserve(xarray.size() * 2);
    for(int i = 0; i < xarray.size(); i++){
        pointsets.push_back(xarray.at(i));
        pointsets.push_back(yarray.at(i));
        asfit::Point pt(xarray.at(i), yarray.at(i));
        pt.attributes["z"] = zarray.at(i);
        pcl_points.push_back(pt);
    }

    // step 02. generate concave hull geometry
    if(!generate_concave_hull(pointsets, concave_geom)){
        return false;
    }

    // step 03. generate reference line by concave hull geometry
    if(!generate_reference_line_with_concave_hull(concave_geom, reference_line)){
        return false;
    }

    // step 04. projecting points onto the reference line
    if(!projection(reference_line, pcl_points, projected_pcl_points)){
        return false;
    }

    // step 05. fitting the pcl points
    if(!fitting_pcl_points(projected_pcl_points, result, density)){
        return false;
    }

    // success
    return true;
}

bool ConcaveHullParamSplineFitting::generate_concave_hull(
    const std::vector<double>& pcl_points, 
    std::vector<asfit::Point>& concave_geom)
{
    std::vector<double> res = concavehull(pcl_points, _concave_lambdans);
    if(res.size() < 3){
        std::cout << "ERROR.chp_spline_fitting.cpp::generate_concave_hull(): concave hull size < 3.\n";
        return false;
    }
    concave_geom.reserve(int(res.size() / 2));
    for(int i = 0; i < res.size(); i += 2)
    {
        asfit::Point pt;
        pt.x = res.at(i);
        pt.y = res.at(i + 1);
        concave_geom.push_back(pt);
    }
    return true;
}

bool ConcaveHullParamSplineFitting::generate_reference_line_with_concave_hull(
    std::vector<asfit::Point>& concave_geom, 
    asfit::Polyline& reference_line)
{
    // radius to angle
    auto rad2angle = [](const double& rad){
        return rad * 180 / M_PI;
    };

    // calculate the angle for all point
    asfit::Point& second_to_last_pt = concave_geom.at(concave_geom.size() - 2);
    for(size_t i = 0; i < concave_geom.size() - 1; i++){
        if(i == 0){
            asfit::Point& p0 = second_to_last_pt;
            asfit::Point& p1 = concave_geom.at(i);
            asfit::Point& p2 = concave_geom.at(i + 1);
            p1.attributes["angle"] = rad2angle(p1.convex(p0, p2));
        }
        else{
            asfit::Point& p0 = concave_geom.at(i - 1);
            asfit::Point& p1 = concave_geom.at(i);
            asfit::Point& p2 = concave_geom.at(i + 1);
            p1.attributes["angle"] = rad2angle(p1.convex(p0, p2));
        }
    }

    // summing the convex angle with slidding window
    concave_geom.pop_back(); 
    size_t sws = size_t(concave_geom.size() * 0.2);
    sws = sws > 10 ? sws : 10;
    sws = sws < 100 ? sws : 100;
    size_t n = concave_geom.size();
    for(size_t i = 0; i < concave_geom.size(); i++){
        double delta = 0.0;
        double max_delta = std::numeric_limits<double>::min();
        asfit::Point& pt = concave_geom.at(i);
        for(int j = 1; j < sws; j++){
            delta += concave_geom.at((i + j) % n).attributes["angle"];
            if(std::fabs(std::fmod(delta, 360)) > max_delta){
                max_delta = delta;
            }
        }
        pt.attributes["max_delta"] = std::fabs(std::fmod(max_delta, 360));
    }

    std::ofstream opt_file("debug-concave.txt");
    if(opt_file.is_open()){
        for(auto& pt : concave_geom){
            opt_file << 
            std::to_string(pt.x) << " " << std::to_string(pt.y) << " "
            << std::to_string(pt.attributes["max_delta"])
            << std::endl;
        }
    }

    // process the SW group
    size_t index = 0;
    std::unordered_map<size_t, asfit::Point*> sws_pt_map; 
    std::vector<size_t> sws_indices;
    for(auto& pt : concave_geom){
        if(pt.attributes["max_delta"] > 160){
            sws_indices.push_back(index);
            sws_pt_map.insert({index, &pt});
        }
        ++index;
    }

    // get sliding window group
    std::list<std::tuple<size_t, size_t>> sw_group = split_vector_with_order_tolerance(
        sws_indices, std::ceil(sws / 2)
    );
    if(sw_group.size() < 2){
        std::cout << "ERROR.ConcaveHullParamSplineFitting::generate_reference_line_with_concave_hull: "
                  << "sliding window group size < 2." << std::endl;
        return false;
    }

    // get the represent window
    std::unordered_map<size_t, asfit::Point*> polar_points;
    for(auto& item : sw_group){
        size_t mid_id = std::floor(0.5 * (std::get<0>(item) + std::get<1>(item)));
        size_t index = sws_indices.at(mid_id);
        double max_angle = -1.0;
        long long target_id = -1;
        for(size_t i = index + 1; i < index + sws; i++){
            size_t id = i % n;
            double angle = std::fabs(concave_geom.at(id).attributes["angle"]);
            if(angle > 100){
                if(max_angle < angle){
                    max_angle = angle;
                    target_id = id;
                }
            }
        }
        if(target_id < 0){
            target_id = size_t(index + std::ceil(sws * 0.5)) % n;
        }
        polar_points.insert({target_id, &concave_geom.at(target_id)});
    }
    if(polar_points.size() < 2){
        std::cout << "ERROR.ConcaveHullParamSplineFitting::generate_reference_line_with_concave_hull: "
                  << "polar size < 2." << std::endl;
        return false;
    }

    // get the ideal polar point
    double max_distance = -1.0;
    std::vector<double> distance_array;
    std::unordered_map<size_t, std::tuple<size_t, size_t>> distance_array_index_to_pt_index;
    std::unordered_map<size_t, std::list<size_t>> pt_index_to_distance_array_index;
    std::vector<size_t> polar_points_keys;
    polar_points_keys.reserve(polar_points.size());
    for(auto& pair : polar_points){ 
        polar_points_keys.push_back(pair.first); 
        pt_index_to_distance_array_index.insert({pair.first, {}});
    }
    for(size_t i = 0; i < polar_points_keys.size(); i++){
        for(size_t j = i + 1; j < polar_points_keys.size(); j++){
            size_t& ii = polar_points_keys.at(i);
            size_t& jj = polar_points_keys.at(j);
            asfit::Point* pt0 = polar_points.at(polar_points_keys.at(i));
            asfit::Point* pt1 = polar_points.at(polar_points_keys.at(j));
            double len = pt0->get_length_to_pt(*pt1);
            if(len > max_distance){
                max_distance = len;
            }
            distance_array.push_back(len);
            size_t data_index = distance_array.size() - 1;
            pt_index_to_distance_array_index[ii].push_back(data_index);
            pt_index_to_distance_array_index[jj].push_back(data_index);
            distance_array_index_to_pt_index.insert({data_index, std::make_tuple(ii, jj)});
        }
    }
    double distance_threshold = max_distance * 0.2;
    std::unordered_set<size_t> erase_pt_indices;
    for(size_t i = 0; i < distance_array.size(); i++){
        if(distance_array.at(i) < distance_threshold){
            auto& item = distance_array_index_to_pt_index.at(i);
            erase_pt_indices.insert(std::get<0>(item));
            erase_pt_indices.insert(std::get<1>(item));
        }
    }
    for(auto& key : erase_pt_indices){
        auto& lists = pt_index_to_distance_array_index.at(key);
        for(auto& id : lists){
            distance_array.at(id) = -1.0;
        }
    }
    max_distance = -1.0;
    size_t target_distance_array_index = 0;
    for(size_t i = 0; i < distance_array.size(); i++){
        double& dis = distance_array.at(i);
        if(dis > 0 && dis > max_distance){
            max_distance = dis;
            target_distance_array_index = i;
        }
    }

    // get the reference line
    auto& link = distance_array_index_to_pt_index.at(target_distance_array_index);
    size_t id_0 = std::get<0>(link);
    size_t id_1 = std::get<1>(link);
    if(id_0 > id_1) std::swap(id_0, id_1);
    double s = 0.0;
    reference_line.data.reserve(id_1 - id_0 + 1);
    for(size_t i = id_0; i <= id_1; i++){
        asfit::Point pt = concave_geom.at(i);
        if(reference_line.data.size() != 0){
            asfit::Point& pt_0 = reference_line.data.back();
            double ds = pt.get_length_to_pt(pt_0);
            s += ds;
            pt.attributes["s"] = s;
        }else{
            pt.attributes["s"] = 0.0;
        }
        reference_line.data.push_back(pt);
    }

    // std::ofstream opt_file("debug-reference-line.txt");
    // if(opt_file.is_open()){
    //     std::vector<asfit::Point> out;
    //     reference_line.douglas_peuker(out, 0.3);
    //     for(auto& pt : out){
    //         opt_file << 
    //         std::to_string(pt.x) << " " << std::to_string(pt.y) << " "
    //         << std::to_string(pt.attributes["max_delta"])
    //         << std::endl;
    //     }
    // }

    std::vector<asfit::Point> out;
    reference_line.douglas_peuker(out, 0.3);
    reference_line.data.swap(out);

    if(reference_line.data.size() < 2){
        std::cout << "ERROR.chp_spline_fitting.cpp::generate_reference_line_with_concave_hull(): reference line size < 2.\n";
        return false;
    } else{ return true; }
}

bool ConcaveHullParamSplineFitting::projection(
    const asfit::Polyline& reference_line, 
    const std::vector<asfit::Point>& pcl_points, 
    std::vector<asfit::Point>& result)
{
    result.reserve(pcl_points.size());
    for(auto& pt : pcl_points){
        double s = 0.0;
        double min_dis = std::numeric_limits<double>::max();
        for(int i = 1; i < reference_line.data.size(); i++){
            const asfit::Point& pt0 = reference_line.data.at(i - 1);
            const asfit::Point& pt1 = reference_line.data.at(i);
            double s_pt0 = pt0.attributes.at("s");
            double ds = 0.0;
            double dis = pt.get_length_to_segment(pt0, pt1, ds);
            if(dis < min_dis){
                s = s_pt0 + ds;
                min_dis = dis;
            }
        }
        asfit::Point pcl_pt = pt;
        pcl_pt.attributes["s"] = s;
        result.push_back(pcl_pt);
    }
    std::sort(result.begin(), result.end(), PointLess());
    if(result.size() != pcl_points.size()){
        std::cout << "ERROR.chp_spline_fitting.cpp::projection(): failed.\n";
        return false;
    }
    else { return true; }
}


bool ConcaveHullParamSplineFitting::fitting_pcl_points(
    std::vector<asfit::Point>& projected_pcl_points,
    std::vector<std::vector<double>>& result,
    const double& density)
{
    std::vector<double> xarray, yarray, zarray, sarray;
    xarray.reserve(projected_pcl_points.size());
    yarray.reserve(projected_pcl_points.size());
    zarray.reserve(projected_pcl_points.size());
    sarray.reserve(projected_pcl_points.size());
    for(auto& pt : projected_pcl_points){
        xarray.push_back(pt.x);
        yarray.push_back(pt.y);
        zarray.push_back(pt.attributes.at("z"));
        sarray.push_back(pt.attributes.at("s"));
    }

    AlglibSplineFitting splinefitting;
    splinefitting.lambdans() = _lambdans;
    splinefitting.base_function_num() = _base_function_num;
    if (!splinefitting.fitting(xarray, yarray, zarray, sarray, result, density)){
        std::cout << "ERROR.AlglibSplineFitting(): alglib spline fitting failed.\n";
        return false;
    }else { return true; }
}