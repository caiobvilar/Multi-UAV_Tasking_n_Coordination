//
// Created by redwan on 8/1/20.
//

#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <wykobi.hpp>
#include <wykobi_algorithm.hpp>

typedef  wykobi::point2d<double> POINT;
typedef wykobi::polygon<double, 2> POLY;
typedef std::vector<POLY> POLYGONS;
using namespace wykobi;
typedef std::pair<POINT, POINT> LINE;

// logger
#include <thread>
#include <QtCore>
#include <QDateTime>
#include <QFile>

// Mintime
#include <unordered_map>
#include <armadillo>
#include <iterator>

// sq
#include <functional>
