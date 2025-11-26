//
// Created by redwan on 4/2/21.
//
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include "matplotlibcpp.h"
#include <nlohmann/json.hpp>

#include "algo/MinTurnDecomposition.h"
#include "algo/HorizontalAreaDecomposition.h"
#include "algo/TriangularDecomposition.h"
#include "algo/SweepPathCalipers.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
namespace plt = matplotlibcpp;
using namespace nlohmann;

enum METHOD
{
    MINTURN = 1,
    HORIZONTAL,
    TRIANGULAR
};

inline void draw_polygon(const wykobi::polygon<double, 2> &poly, int h_scale = 1, int w_scale = 1, const string &color = "r")
{
    vector<double> x, y;
    std::cout << "[";
    for (const auto &i : poly)
    {
        x.push_back(i.x * w_scale);
        y.push_back(i.y * h_scale);
        printf("(%lf, %lf) ", i.x * w_scale, i.y * h_scale);
    }
    std::cout << "]" << std::endl;
    x.push_back(poly[0].x * w_scale);
    y.push_back(poly[0].y * h_scale);
    plt::plot(x, y, color);
    plt::axis("off");
}

inline void option_parser(const wykobi::polygon<double, 2> roi, const vector<double> &uav_batteries, const METHOD &method, const string &img_path)
{
    vector<wykobi::polygon<double, 2>> result;
    switch (method)
    {
    case MINTURN:
    {
        //            cout <<"[Method]: MINTURN" << endl;
        TaskPlanning::MinTurnDecomposition algo(uav_batteries);
        result = algo.solve(roi);
        break;
    }
    case HORIZONTAL:
    {
        //            cout <<"[Method]: HORIZONTAL" << endl;
        TaskPlanning::HorizontalAreaDecomposition algo(uav_batteries);
        result = algo.solve(roi);
        break;
    }
    case TRIANGULAR:
        //            cout <<"[Method]: TRIANGULAR" << endl;
        TaskPlanning::TriangularDecomposition algo(uav_batteries);
        result = algo.solve(roi);
        break;
    }

    //    assert(result.size() == uav_batteries.size() && " result does not comply");

    for (std::size_t i = 0; i < result.size(); ++i)
    {
        auto polygon = result[i];
        std::vector<wykobi::point2d<double>> convex_hull;
        wykobi::algorithm::convex_hull_melkman<wykobi::point2d<double>>(
            polygon.begin(),
            polygon.end(),
            std::back_inserter(convex_hull));
        wykobi::polygon<double, 2> convex_hull_polygon = wykobi::make_polygon<double>(convex_hull);
        result[i] = convex_hull_polygon;
    }
    // load image
    int width = 1, height = 1;

    if (!img_path.empty())
    {
        auto img = cv::imread(img_path, cv::IMREAD_UNCHANGED);
        cout << "[Imgfile] size " << img.size << endl;
        height = img.size[0];
        width = img.size[1];
        plt::imshow(img.data, height, width, 3);
        // actual area was 100x100
        height /= 100;
        width /= 100;
    }
    json outfile;
    auto poly_to_vec = [](const wykobi::polygon<double, 2> &poly)
    {
        json coord;
        vector<double> x, y;
        for (const auto &p : poly)
        {
            x.push_back(p.x);
            y.push_back(p.y);
        }
        coord["x"] = x;
        coord["y"] = y;
        return coord;
    };
    int count = 1;
    for (auto &poly : result)
    {
        draw_polygon(poly, height, width, "grey");
        outfile["roi_" + to_string(count++)] = poly_to_vec(poly);
    }

    if (method != TRIANGULAR)
    {
        // drawing sweep path
        SweepPathCalipers path(uav_batteries, roi);
        auto sweep_paths = path.solve(result, true);
        vector<string> colors{"--r", "--g", "--b"};
        int count = 0;
        for (auto &poly : sweep_paths)
        {
            draw_polygon(poly, height, width, colors[count]);
            outfile["path_" + to_string(count)] = poly_to_vec(poly);
            count++;
        }

        // computing path length
        float total_path_length = 0;
        for (auto &poly : sweep_paths)
        {
            float path_length = 0;
            for (std::size_t i = 0; i < poly.size() - 1; ++i)
            {
                arma::vec2 a{poly[i].x, poly[i].y};
                arma::vec2 b{poly[i + 1].x, poly[i + 1].y};
                path_length += arma::norm(a - b);
            }
            // std::size_t index = sweep_paths.size() - count--;
            total_path_length += path_length;
            //            printf("[PathLength] %d : %f m\n", index, path_length);
        }
        //        printf("[PathLength] total path length : %f m\n", total_path_length);
    }

    // write json file
    std::ofstream o("result.json");
    o << std::setw(4) << outfile << std::endl;
}
