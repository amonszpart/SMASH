#ifndef PA_GNUPLOTTER_H
#define PA_GNUPLOTTER_H

#include "Eigen/Core"
#include "Eigen/StdVector"
#include "tracking/common/io/os.h"
#include <map>
#include <fstream>
#include <iostream>

namespace io {
    class Gnuplotter {
        public:
            typedef Eigen::Vector2f PointT;
            struct Options {
                    inline void setXBounds(PointT const &xBounds) { _xBounds = xBounds; _hasXBounds = true; }
                    inline void setYBounds(PointT const &yBounds) { _yBounds = yBounds; _hasYBounds = true; }

                    inline PointT const& getYBounds() const { return _yBounds; }
                    inline PointT const& getXBounds() const { return _xBounds; }

                    inline bool hasXBounds() const { return _hasXBounds; }
                    inline bool hasYBounds() const { return _hasYBounds; }
                protected:
                    PointT _xBounds, _yBounds;
                    bool _hasXBounds, _hasYBounds;
            }; //...struct Options

        protected:
            typedef std::vector<PointT,Eigen::aligned_allocator<PointT> > SecondT;
        public:
            typedef std::map< std::string,
                              SecondT,
                              std::less<std::string>,
                              Eigen::aligned_allocator< std::pair<std::string,SecondT> >  > StringVector2fMap;
            inline void   addPoint(const PointT     &point, const std::string& tag) { _plots[tag].push_back(point); }
            inline void   addValue(const float      &value, const std::string& tag) { _plots[tag].push_back( PointT( _plots[tag].size(), value) ); }
            inline size_t getSize (const std::string&tag ) const { fflush(stdout); if ( _plots.find(tag) == _plots.end() ) return 0; else return _plots.at(tag).size(); }
            void plot( const std::string& title, std::string const& mode = "points", Options const* const options = nullptr ) const;
        protected:
            StringVector2fMap _plots;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    }; //...cls GnuPlotter

    inline void Gnuplotter::plot( const std::string& title, std::string const& mode, Options const* const options ) const
    {
        std::string folderName("gv_tmp");
        io::my_mkdir(folderName.c_str());
        std::vector< std::pair<std::string, std::string> > fNamesTitles;
        for ( auto const& labelPlotPair : _plots )
        {
            std::string fName(folderName + kFileSep + title + labelPlotPair.first + ".gv");
            std::ofstream f( fName );
            for ( auto const& entry : labelPlotPair.second )
            {
                f << entry(0) << " " << entry(1) << std::endl;
            }
            f.close();
            fNamesTitles.push_back( std::make_pair(fName,labelPlotPair.first) );
        }
        std::stringstream ss;
        ss << "gnuplot -p -e \"set term x11 title '" << title << "'; plot ";
        int id = 0;
        for ( auto const& fName : fNamesTitles )
        {
            ss << "'" + fName.first + "' w " + mode + " title '" << fName.second + "'";
            if ( size_t(id + 1) != fNamesTitles.size() )
                ss << ",";
            ++id;
        }
        ss << ";";
        ss << "set title '" << title << "';";
        if (options) {
            if (options->hasXBounds())
                ss << "set xrange " << options->getXBounds()(0) << " < " << options->getXBounds()(1) << ";";
            if (options->hasYBounds())
                ss << "set yrange " << options->getYBounds()(0) << " < " << options->getYBounds()(1) << ";";
        }
        ss << "\"";
        std::string command = ss.str();
        //std::cout << command << std::endl;
        int ret = system( command.c_str() );
        if (ret != 0)
            std::cout << "[" << __func__ << "] systemRet: " << ret << std::endl;
    } //...plot()

}

#endif // PA_GNUPLOTTER_H
