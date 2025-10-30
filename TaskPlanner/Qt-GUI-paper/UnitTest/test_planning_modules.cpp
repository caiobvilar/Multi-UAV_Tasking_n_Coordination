//
// Created by redwan on 7/31/20.
//

#include "gtest/gtest.h"
#include <vector>
#include <algorithm>
#include "wykobi.hpp"
#include "wykobi_algorithm.hpp"
#include "nlohmann/json.hpp"
#include "Algorithm/MinTurnDecomposition.h"
#include "Algorithm/SweepPathCalipers.h"
#include <filesystem>
using namespace std;
using namespace nlohmann;
using namespace wykobi;
typedef wykobi::polygon<double, 2> POLY;
typedef std::vector<POLY> POLYGONS;

void area_decompose(const vector<double>& batteries, POLYGONS & decompose_area)
{
    TaskPlanning::MinTurnDecomposition algo(batteries);
    decompose_area = algo.solve(decompose_area[0]);
}

void path_planning(const vector<double>& footprints, POLYGONS & decompose_area, const POLY &target)
{
    SweepPathCalipers path(footprints, target);
    decompose_area = path.solve(decompose_area, false);
}

void test_path_planning(POLYGONS& polygons, json& j, const double &coverage, const POLY & ref)
{
    vector<double>footprints;
    from_json(j["footprints"], footprints);
    path_planning(footprints, polygons, ref);
    double total = 0;
    for (auto& path: polygons)
    {
        // covert path to convex polygon
        POLY poly;
        std::vector<wykobi::point2d<double>> hull;
        wykobi::algorithm::convex_hull_melkman<wykobi::point2d<double>>
                (
                        path.begin(),
                        path.end(),
                        std::back_inserter(hull)
                );
        poly = wykobi::make_polygon<double>(hull);
        // compute the area of the polyon
        total += abs(area(poly));
    }
    EXPECT_GE(round(total), coverage/1.3);
    EXPECT_LE(round(total), coverage);
}


void test_decomposition(POLYGONS& polygons, json& j, double& coverage, double offset = 2)
{
    vector<double>x, y, batteries;
    from_json(j["roi"]["x"], x);
    from_json(j["roi"]["y"], y);
    from_json(j["batteries"], batteries);

    ASSERT_GE(x.size(), batteries.size());
    POLY poly;
    for (int k = 0; k < x.size(); ++k) {
        poly.push_back(make_point(x[k], y[k]));
    }
    polygons.emplace_back(poly);
    area_decompose(batteries, polygons);
    ASSERT_TRUE(polygons.size() == batteries.size());
    vector<double>total_area(polygons.size());
    std::transform(polygons.begin(),polygons.end(), total_area.begin(),[&](const POLY& a){return abs(area(a));});
    coverage = std::accumulate(total_area.begin(), total_area.end(), 0);

    EXPECT_GE(coverage, round(abs(area(poly))- offset));
    // new sweep path requires target area as a reference
    // put this information at the back of polygon
    polygons.emplace_back(poly);

}


TEST(PlanningModuels, SweepPath)
{
//  get current directory from this file and add tests folder
    char * absolute_path = __FILE__;
    int size_ = strlen(absolute_path) - 25 ; // strlen(__FILE__NAME__) = 25

    auto convertToString = [](char* a, int size)
    {
        string s = "";
        for (int i = 0; i < size; i++) {
            s = s + a[i];
        }
        return s;
    };
    // remove the filename and get the directory name only
    auto path = convertToString(absolute_path, size_) + "tests";
    cout<<"[TEST_DIR]: "<<path<<"\n";

    for (const auto & entry : std::filesystem::directory_iterator(path))
    {
        // read a JSON file
        try {
            std::ifstream i(entry.path());
            json j; i >> j;
            POLYGONS polygons;
            double coverage;
            cout<< "[" << entry.path().filename() << "]: " <<endl;
            test_decomposition(polygons, j, coverage, 5);
            // the last element is the target reference
            // path planning requires target reference
            // remove ref from decomposed polygons
            int N = polygons.size() -1 ;
            auto ref = polygons.at(N);
            polygons.pop_back();
            test_path_planning(polygons, j , coverage, ref);
        }
        catch (...) {
//            cout <<"[Ignoring]: " << entry.path() <<endl;
        }
    }



}