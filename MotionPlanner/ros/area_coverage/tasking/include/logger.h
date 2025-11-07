//
// Created by robor on 5/29/2020.
//

#ifndef AREACOVERAGE_LOGGER_H
#define AREACOVERAGE_LOGGER_H

#include "pch.h"

class logger
{
public:
    logger(std::shared_ptr<solver> &solver) : logger_thread_(), m_solver_(solver)
    {

        QDir current;
        current.mkdir("output");
    }
    ~logger()
    {
        if (logger_thread_.joinable())
        {
            logger_thread_.join();
        }
    }
    void write_json(const QJsonObject &data) const
    {
        // write json file
        QJsonDocument JsonDocument;
        JsonDocument.setObject(data);
        QString filepath;
#ifdef LOGGER_ONCE
        filepath = "output/motionPlan.json";
        QFile file2(filepath);
        file2.remove();
#else
        // set a filename from timestamp
        const QDateTime now = QDateTime::currentDateTime();
        const QString timestamp = now.toString(QLatin1String("yyyyMMdd-hhmmsszzz"));
        // write json file to a file
        filepath = "output/result_" + timestamp + ".json";
#endif

        QFile file(filepath);
        file.open(QFile::WriteOnly | QFile::Text | QFile::Truncate);
        file.write(JsonDocument.toJson());
        file.close();
    }

    void polygon_to_json(const std::vector<wykobi::polygon<double, 2>> &polygons, QJsonObject &data, const QString &field)
    {
        QJsonObject temp;
        int j = 0;
        for (auto &poly : polygons)
        {
            QJsonArray x, y;
            for (std::size_t i = 0; i < poly.size(); ++i)
            {
                x.append(poly[i].x);
                y.append(poly[i].y);
            }
            QJsonObject val;
            val["x"] = x;
            val["y"] = y;
            temp[QString::number(j++)] = val;
        }
        data[field] = temp;
    }

    template <typename T>
    void point_to_json(const std::vector<T> &initial_positions, QJsonObject &data, const QString &field)
    {
        QJsonArray x, y;
        for (auto &pos : initial_positions)
        {
            x.append(pos.x);
            y.append(pos.y);
        }
        QJsonObject val;
        val["x"] = x;
        val["y"] = y;
        data[field] = val;
    }

    template <typename T>
    void vector_to_json(const std::vector<T> &variable, QJsonObject &data, const QString &field)
    {

        QJsonArray temp;
        for (auto &val : variable)
            temp.append(val);
        data[field] = temp;
    }

    void execute()
    {
        std::cout << "[logger] logging problem ...\n";
        auto solver_ = m_solver_.lock();
        do
        {
            std::unique_lock<std::mutex> lk(solver_->m);

            solver_->cv.wait(lk, [=]
                             { return solver_->logging; });
            // todo write logs

            if (!solver_->STOP)
            {
                std::cout << "[logger] writing log \n";
                QJsonObject data;
                point_to_json(solver_->initial_positions_, data, "initial_positions");
                std::vector<POINT> roi;
                std::copy(solver_->roi_.begin(), solver_->roi_.end(), std::back_inserter(roi));
                point_to_json(roi, data, "roi");

                vector_to_json(solver_->veolcity_limits_, data, "max_velocities");
                vector_to_json(solver_->uav_batteries_, data, "batteries");
                vector_to_json(solver_->sensor_footprints_, data, "footprints");
                polygon_to_json(solver_->decompose_areas, data, "areas");
                polygon_to_json(solver_->sweep_paths, data, "paths");
                solver_->publish_task();
                write_json(data);
                solver_->logging = false;
            }
            lk.unlock();

        } while (!solver_->STOP);
        std::cout << "[logger] terminated \n";
    }

    void Start()
    {
        logger_thread_ = std::thread(&logger::execute, this);
    }

private:
    std::thread logger_thread_;
    std::weak_ptr<solver> m_solver_;
};

#endif // AREACOVERAGE_LOGGER_H
