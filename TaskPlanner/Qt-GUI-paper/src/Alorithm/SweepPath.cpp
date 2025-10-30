//
// Created by robor on 5/24/2020.
//

#include "Algorithm/SweepPath.h"
#include "Algorithm/RotatingCalipers.h"
using namespace PathPlanning;
using namespace TaskPlanning;
SweepPath::SweepPath(const std::vector<double> &footprints):footprints_(const_cast<std::vector<double> &>(footprints)) {

}

std::vector<wykobi::polygon<double, 2>> SweepPath::solve(const std::vector<wykobi::polygon<double, 2>> &decomoseArea) {


    std::vector<wykobi::polygon<double, 2>> result;

    for(const auto& clip_boundry: decomoseArea)
    {
        wykobi::polygon<double,2> clipped_polygon = m_optimize(clip_boundry);
        result.push_back(clipped_polygon);

    }
    return result;
}

std::vector<double> SweepPath::m_get_yaw_angles(const wykobi::polygon<double, 2> &decomosePolygon) const {
    std::vector<double> result(decomosePolygon.size());
    for (int i = 0; i < decomosePolygon.size()-1; ++i) {
        double dy = decomosePolygon[i+1].y - decomosePolygon[i].y;
        double dx = decomosePolygon[i+1].x - decomosePolygon[i].x;
        double m = atan2(dy,dx);
        result[i] = m;
    }
    return result;


}

double SweepPath::m_path_stat(const wykobi::polygon<double, 2> &pathPolygon) const {
    double length = 0;
    double num_turns = pathPolygon.size();
    if(num_turns == 0)return std::numeric_limits<double>::max();
    for (int i = 0; i < pathPolygon.size()-1; ++i) {
        double dy = pathPolygon[i+1].y - pathPolygon[i].y;
        double dx = pathPolygon[i+1].x - pathPolygon[i].x;
        length += sqrt(dx*dx + dy*dy);
    }
//    printf("[optimizer]: (%ld, %ld) = %ld \n",length, num_turns, num_turns/length);
    return num_turns*length;
}

wykobi::polygon<double, 2>  SweepPath::m_optimize(const wykobi::polygon<double, 2> &decomosePolygon) {

    std::vector<double> angles = m_get_yaw_angles(decomosePolygon);
    wykobi::polygon<double, 2> result;
    double best_path_val = std::numeric_limits<double>::max();
    int N = decomosePolygon.size();
    int i = 0;
    for(auto &alpha:angles)
    {
        geom::rectangle target;

        geom::line ref(geom::point(decomosePolygon[i].x,decomosePolygon[i].y),
                       geom::point(decomosePolygon[(i+1)%N].x,decomosePolygon[(i+1)%N].y)
                );
        m_rotate_rectangle(target,ref);
        wykobi::polygon<double ,2> polygon = target.getSweepPath(0.001);
        wykobi::polygon<double,2> clipped_polygon;
        wykobi::algorithm::sutherland_hodgman_polygon_clipper<wykobi::point2d<double>>
                (decomosePolygon, polygon, clipped_polygon);
        double eval = m_path_stat(clipped_polygon);
        if(eval < best_path_val)
        {
            best_path_val = eval;
            result.clear();
            std::copy(clipped_polygon.begin(),clipped_polygon.end(),std::back_inserter(result));
//            // std::cout<< eval << std::endl;
        }

    }
    // std::cout<< best_path_val << std::endl;
    return result;

}
//
//void SweepPath::m_rotate_rectangle(geom::rectangle &bb, geom::line &ref) {
//    // convert rectangle to a matix format
//    arma::mat X(4,2);
//    for(int i = 0; i<4; ++i)
//    {
//        X(i,0) = bb.all[i]->getA().x;
//        X(i,1) = bb.all[i]->getA().y;
//    }
//    //rotate
//    double dy = ref.getB().y - ref.getA().y;
//    double dx = ref.getB().x - ref.getA().x;
//    double theta = atan2(dy,dx);
//    arma::vec x0(2);
//    x0(0) = ref.getA().x;
//    x0(1) =  ref.getA().y;
//
//    arma::mat R(2,2);
//    R(0, 0) = cos(theta);
//    R(0, 1) = sin(theta);
//    R(1, 0) = sin(theta);
//    R(1, 1) = -cos(theta);
//
//    arma::mat X_hat(4,2);
//    arma::vec center(2);
//    X_hat =  X*R;
//    arma::mat B = repmat(x0.t(), 4, 1);
//    X_hat +=   B;
//    center(0) = (X_hat(0,0) + X_hat(2,0))/2.0;
//    center(1) = (X_hat(0,1) + X_hat(2,1))/2.0;
//    arma::mat C = repmat(center.t(), 4, 1);
//    X = X_hat - C + B;
//    //FIXME rotate here
//    Point p0(X(0,0),X(0,1));
//    Point p1(X(1,0),X(1,1));
//    Point p2(X(2,0),X(2,1));
////    Point p3(X(3,0),X(3,1));
//    auto dist1 = RotatingCalipers::dist(p0, p1);
//    auto dist2 = RotatingCalipers::dist(p2, p1);
//    std::cout<<"[MinTurnDecomposition] H x W " <<dist1 <<" x " << dist2 <<"\n";
//    int init = (dist1<dist2)?0:1;
//    printf("init %d \n", init);
//
//    for(int i = init; i<4+init; ++i)
//        bb.all[i] = new geom::line(geom::point(X(i%4,0),X(i%4,1)),geom::point(X((i+1)%4,0),X((i+1)%4,1)));
////    // std::cout<<"[X:]\n"<<X <<std::endl;
//}
