//
// Created by robor on 6/20/2020.
//

#ifndef AREACOVERAGE_HORIZONTALAREADECOMPOSITION_H
#define AREACOVERAGE_HORIZONTALAREADECOMPOSITION_H

#include "MinTurnDecomposition.h"

namespace TaskPlanning
{
    class HorizontalAreaDecomposition: public MinTurnDecomposition {

    public:
        HorizontalAreaDecomposition(const vector<double> &batteries);

        vector<wykobi::polygon<double, 2>> solve(const wykobi::polygon<double, 2> &target) override;


    };
}

#endif //AREACOVERAGE_HORIZONTALAREADECOMPOSITION_H
