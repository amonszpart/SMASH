//
// Created by bontius on 03/01/16.
//

#ifndef TRACKVIDEO_COMMON_ENERGYPLOTTER_H
#define TRACKVIDEO_COMMON_ENERGYPLOTTER_H

#include <string>
#include "gnuPlotter.h"

class EnergyPlotter
{
    public:
        EnergyPlotter() = default;
        static EnergyPlotter& getInstance()
        {
            static EnergyPlotter instance;
            return instance;
        }

        void addValue( const float value, const std::string& windowName, const std::string& dataName = "" )
        {
            objPlot[ windowName ].addValue( value, dataName.empty()?windowName:dataName );
        }
        void addPoint( const float value, const float x, const std::string& windowName, const std::string& dataName = "" )
        {
            objPlot[ windowName ].addPoint( io::Gnuplotter::PointT(x,value), dataName.empty()?windowName:dataName );
        }

        void plot(std::string const mode = "points", io::Gnuplotter::Options const * const options = nullptr) {
            for (auto const pair : objPlot) {
                pair.second.plot(pair.first, mode, options);
            }
        }
    protected:
        std::map< std::string, io::Gnuplotter > objPlot;
    public:
        EnergyPlotter(EnergyPlotter const&)   = delete;
        void operator=(EnergyPlotter const&)  = delete;
};

#endif //TRACKVIDEO_COMMON_ENERGYPLOTTER_H
