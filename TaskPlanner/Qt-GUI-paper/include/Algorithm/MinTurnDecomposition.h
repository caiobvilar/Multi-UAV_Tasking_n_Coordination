//
// Created by robor on 7/14/2020.
//

#ifndef AREACOVERAGE_MINTURNDECOMPOSITION_H
#define AREACOVERAGE_MINTURNDECOMPOSITION_H

#include <iostream>
#include <wykobi.hpp>
#include <wykobi_algorithm.hpp>
#include "Algorithm/RotatingCalipers.h"
#include <armadillo>
#include <functional>


namespace TaskPlanning
{
    class MinTurnDecomposition {
    public:
        MinTurnDecomposition(const vector<double> &batteries);
        virtual std::vector<wykobi::polygon<double, 2>> solve(const wykobi::polygon<double,2>& target);

        double binary_search(double cap, std::function<double(double)>&f);

        virtual ~MinTurnDecomposition();

        static std::tuple<arma::mat22, arma::mat22> MinRectMat(const wykobi::polygon<double, 2> &target);





    protected:
        std::vector<double> batteries_, capabilities_;
    protected:
        std::vector<double>m_get_capabilities(double area );
        wykobi::polygon<double, 2> m_get_boundinbox(const arma::mat22& AD, const arma::mat22& EF);
        wykobi::polygon<double, 2> m_cut_polygon(const wykobi::polygon<double, 2>& divider, const wykobi::polygon<double, 2>& target);

        void m_decompose_area(const wykobi::polygon<double, 2> &target, const arma::mat22 &AD, const arma::mat22 &BC,
                              arma::mat22 &EF, arma::mat22 &REF, vector<wykobi::polygon<double, 2>> &result);
    };

}

#endif //AREACOVERAGE_MINTURNDECOMPOSITION_H
