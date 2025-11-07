#include <iostream>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <cassert>

#include "algo/TriangularDecomposition.h"
#include "matplotlibcpp.h"

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include "src/option_parser.cpp"
using namespace std;
using namespace nlohmann;
namespace plt = matplotlibcpp;
using namespace arma;
namespace po = boost::program_options;

int main(int argc, char *argv[])
{
    //    std::cout << "AreaConverageBenchmarking" << std::endl;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()("help,h", "Help screen")("j", po::value<string>(), "input json file")("i", po::value<string>(), "input image file")("o", po::value<string>(), "output image dir")("m", po::value<int>(), "method number 1: MINTURN 2: HORIZONTAL 3: TRIANGULAR");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        cout << desc << "\n";
        return 1;
    }

    assert(vm.count("j") && "JSON file missing");
    //    assert(vm.count("i") && "Image file missing");

    string jFileName = vm["j"].as<string>();
    string imgFileName;
    if (vm.count("i"))
        imgFileName = vm["i"].as<string>();

    std::ifstream jFile(jFileName);
    json jData;
    jFile >> jData;
    vector<double> roi_x, roi_y;
    from_json(jData["roi"]["x"], roi_x);
    from_json(jData["roi"]["y"], roi_y);
    assert(roi_x.size() == roi_y.size() && "input roi does have same number of rows");
    assert(roi_x.size() > 0 && "roi cannot be empty");
    // convert vector coord to wykobi polygon
    wykobi::polygon<double, 2> roi;
    for (std::size_t i = 0; i < roi_y.size(); ++i)
        roi.push_back(wykobi::make_point<double>(roi_x[i], roi_y[i]));
    // initialize the class and solve the decomposition problem
    vector<double> uav_batteries;
    from_json(jData["batteries"], uav_batteries);
    int method = vm["m"].as<int>();
    option_parser(roi, uav_batteries, static_cast<METHOD>(method), imgFileName);

    if (vm.count("o"))
    {
        boost::filesystem::path p(jFileName);
        string out_file = vm["o"].as<string>() + p.stem().string() + ".png";
        plt::save(out_file);
    }
    else
        plt::show();
    return 0;
}
