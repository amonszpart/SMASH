#include "tracking/common/io/convert.h"

#include "tracking/common/io/broxIO.h"           // readTracks
#include "tracking/common/util/impl/parse.hpp"   // console
#include "tracking/common/track.h"
#include "tracking/common/io/sequentialReader.h"
#include "tracking/common/io/rgbIo.h"
#include "CTensor.h"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>
#include <ios>
#include <sys/stat.h>

int tracking::convertPPM( int argc, const char** argv )
{
    if ( argc < 4 )
    {
        std::cerr << "usage: --convert-ppm in.ppm out.ppm" << std::endl;
        return 1;
    }

    std::ifstream file( argv[2], std::ios::in | std::ios::binary );
    if ( !file.is_open() )
    {
        std::cerr << "Could not open " << argv[2] << std::endl;
        return 1;
    }

    int width{0}, height{0}, maxVal;
    int infoId( 0 ); // 0: P6, 1: width 2: height, 3: maxValue
    std::string line("");
    char memblock[1];
    while ( !file.eof() )
    {
        file.read( memblock, 1 );
        if ( memblock[0] != '\n')
        {
            line += memblock[0];
        }
        else
        {
            file.read(memblock,1); // seek behind next character
            std::cout << line << std::endl;
            if ( line[0] != '#' )
            {
                switch ( infoId )
                {
                    case 0: ++infoId; break;
                    case 1: width = atoi(line.c_str()); ++infoId; break;
                    case 2: height = atoi(line.c_str()); ++infoId; break;
                    case 3: maxVal = atoi(line.c_str()); ++infoId; break;
                    default: std::cerr << "infoId " << infoId << " should not be reached" << std::endl; break;
                }
            }
            line.clear();
            if ( infoId == 4 )
            {
                CTensor<float> img;
                img.setSize( width, height, 3 );
                char* data = new char[ width * height * 3];
                file.read( reinterpret_cast<char*>(data), width * height * 3 );
                //file.read( reinterpret_cast<char*>(img.data()), width * height * 3 );

                int offset = 0;
                for ( int y = 0; y != height; ++y )
                    for ( int x = 0; x != width; ++x )
                    {
                        img(x,y,1) = static_cast<float>(data[offset++]);
                        img(x,y,2) = static_cast<float>(data[offset++]);
                        img(x,y,0) = static_cast<float>(data[offset++]);
                    }
                img.writeToPPM( argv[3] );
                std::cout << "wrote to " << argv[3] << std::endl;
                break;
            }
        }
    }

    //fprintf(fp, "P6\n %s\n %d\n %d\n %d\n", comment, width, height, maxColorComponentValue);
    /* write image data bytes to the file */
    //std::cout << height << " * " << width << " * 3 = " << height * width * 3  << " vs. " << sizeof(data) << std::endl;
    //fwrite( data, sizeof(data), 1, fp );
    //fclose(fp);

    return EXIT_SUCCESS;
}

int tracking::convertTracksToBinary(int argc, const char** argv) {
    std::string fPath;
    if (!console::parse_arg(argc,argv,"--dat",fPath)) {
        std::cerr << "[" << __func__ << "]: " << "you should specify the input tracks by --dat" << std::endl;
        return 1;
    }
    std::string ofPath = fPath.substr( 0, fPath.rfind(".") ) + ".bin";

    int frameIdOffset(0);
    if (console::parse_arg(argc,argv,"--frame-offset",frameIdOffset))
        std::cout << "[" << __func__ << "]: " << "offsetting frames by " << frameIdOffset << std::endl;

    BroxIO::convertDatToBin( fPath, ofPath, frameIdOffset );

    return 0;
} //...convertTracksToBinary()

// --mask-tracks --dat truckVideo0Results100/truckVideo0Tracks101.bin --mask mask.png --out truckVideo0Results100/masked.bin
// --mask-tracks --dat truckVideo0Results/truckVideo0Tracks460.bin --mask mask.png --out truckVideo0Results/masked.bin
// --mask-tracks --dat truckVideo0Results/truckVideo0Tracks460.bin --mask mask_0000.png --frame-span 0,40 --out truckVideo0Results/masked_0_40.bin
// --mask-tracks --dat truckVideo0Results/truckVideo0Tracks460.bin --mask mask_0160.png --frame-span 160,200 --out truckVideo0Results/masked_160_200.bin
// --mask-tracks --dat truckVideo0Results/truckVideo0Tracks460.bin --mask mask_0160.png --frame-span 200 --out truckVideo0Results/masked_200_300.bin
int tracking::maskTracks( int argc, const char** argv )
{
    std::cout << "Usage: " << "--dat inputTracks.dat/bin --mask mask.png --out maskedTracks.bin" << std::endl;

    std::string fPath;
    if ( !console::parse_arg(argc,argv,"--dat",fPath) )
    {
        std::cerr << "[" << __func__ << "]: " << "you should specify the input tracks by --dat" << std::endl;
        return 1;
    }
    std::string maskPath;
    if ( !console::parse_arg(argc,argv,"--mask",maskPath) )
    {
        std::cerr << "[" << __func__ << "]: " << "you should specify the mask --mask" << std::endl;
        return 1;
    }
    std::string ofPath;
    if ( !console::parse_arg(argc,argv,"--out",ofPath) )
    {
        std::cerr << "[" << __func__ << "]: " << "you should specify the output tracks by --out" << std::endl;
        return 1;
    }
    FrameIdsT frameSpan;
    {
        std::vector<unsigned> tmpFrameSpan;
        if ( console::parse_x_arguments(argc,argv,"--frame-span",tmpFrameSpan) < 0 )
        {
            std::cerr << "[" << __func__ << "]: " << "you should specify the span of frames to apply the mask to by --frame-span start,stop" << std::endl;
            return 1;
        }
        else
            frameSpan.insert( frameSpan.end(), tmpFrameSpan.begin(), tmpFrameSpan.end() );
    }
    size_t maxTracks(0);
    console::parse_arg(argc,argv,"--max-tracks",maxTracks);
    std::cout << "out Track count limit: " << maxTracks << std::endl;
    if ( frameSpan.size() > 2 )
    {
        std::cerr << "span can only be one or two numbers" << std::endl;
        return 1;
    }
    else if ( frameSpan.size() == 1 )
    {
        frameSpan.push_back( frameSpan.at(0)+1 );
    }

    //std::cout << "reading " << fPath << std::endl;
    //Tracks* tracks( new Tracks );
    //BroxIO::readTracks( fPath, *tracks );
    //std::cout << "reading " << fPath << " finished" << std::endl;
    SequentialReader reader( fPath, SequentialReader::BINARY );

    cv::Mat mask;
    mask = cv::imread( maskPath, cv::IMREAD_GRAYSCALE );
    cv::imshow( "mask", mask );
    cv::waitKey();
    std::cout << "mask.cn: " << mask.channels() << std::endl;
    cv::Mat out( cv::Mat::zeros(mask.rows, mask.cols, CV_8UC1) );

    Tracks2D outTracks;
    outTracks.setSequenceLength( reader.getSequenceLength() );
    //std::vector<Track::FrameId> frameSpan = { 0, 40 };

    int trackId(0);
    Track2D track;
    while ( reader.readNext(track) )
    {
        bool doAdd( false );
        for ( FrameId frameId = frameSpan[0]; frameId != frameSpan[1] && !doAdd; ++frameId )
        {
            if ( track.hasPoint(frameId) )
            {
                auto const& pnt( track.getPoint(frameId) );
                if ( mask.at<uchar>(pnt.getPoint()(1),pnt.getPoint()(0)) > 128 )
                {
                    out.at<uchar>(pnt.getPoint()(1),pnt.getPoint()(0)) = 255;

                    doAdd = true;
//                    std::cout << "out.back: " << outTracks.back() << std::endl;
//                    std::cout << "track: " << track << std::endl;
                }
            }
        }

        if ( doAdd )
        {
            //outTracks.addTrack( track );
            outTracks.addTrack( track );
//            std::cout << "keys[" << outTracks.getTrackCount() << "]: " << std::endl;
//            for ( auto const& outTrack : outTracks )
//                std::cout << outTrack << std::endl;
//            std::cout << std::endl;
        }
        ++trackId;
        if ( !(trackId % 10000) )
            std::cout << "[" << __func__ << "]: " << trackId << ", added: " << outTracks.getTrackCount() << std::endl;
        if ( maxTracks && outTracks.getTrackCount() > maxTracks )
            break;
    }

    cv::imshow("out", out );
    cv::waitKey();

    std::cout << "writing tracks" << std::endl;
    BroxIO::writeTracks( outTracks, ofPath );

    return 0;
}

// cx: 60.000000, cy: 60.000000, fx: 131.250000, fy: 131.250000
int tracking::convertPng2Ppm( int argc, const char** argv )
{
    int index = console::find_argument( argc,argv,"--png2ppm" );
    if ( index < 1 || index+2 >= argc)
    {
        std::cerr << "usage: --png2ppm in.png out.ppm" << std::endl;
        return EXIT_FAILURE;
    }
    std::string inPath;
    console::parse_var( argv[index+1], inPath );
    std::string outPath;
    console::parse_var( argv[index+2], outPath );

    cv::Mat img = cv::imread(inPath, cv::IMREAD_COLOR);
    tracking::writeRgb( img, outPath );
    std::cout << "[" << __func__ << "]: " << "wrote to " << outPath << std::endl;
    return EXIT_SUCCESS;
}

// /media/Data2/Data/funcAcq/data/20151023_1823_proc$ ../runTracking.sh list.bmf 0 72 1
// --cars --dir /home/bontius/workspace/funcAcq/data/cars1/ --bmf cars1.bmf --dat cars1Results/cars1Tracks19.dat
// --dir /home/bontius/workspace/funcAcq/data/truckVideo0_101/ --bmf truckVideo0.bmf --dat truckVideo0Results100/truckVideo0Tracks101.dat
int tracking::convertDmap( int argc, const char** argv ) {
    std::string path;
    console::parse_arg(argc,argv,"--convert-dmap",path);
    std::ifstream f( path.c_str(), std::ios::binary );
    int width, height;
    f.read( reinterpret_cast<char*>(&width), sizeof(int) );
    f.read( reinterpret_cast<char*>(&height), sizeof(int) );
    std::cout << "w:" << width << ", h:" << height << std::endl;
    f.close();

    return EXIT_SUCCESS;
}

int tracking::maskPpm( int argc, const char** argv ) {
    int index = console::find_argument( argc,argv,"--mask-ppm" );
    if ( index < 1 || index+4 >= argc)
    {
        std::cerr << "usage: --mask-ppm color_%05d.ppm mask.png starFrame endFrame step" << std::endl;
        return EXIT_FAILURE;
    }
    std::string pattern( argv[index+1]);
    std::string maskPath( argv[index+2]);
    size_t startFrame = std::atoi(argv[index+3]);
    size_t endFrame   = std::atoi(argv[index+4]);
    size_t step       = 0;
    if ( argc >= index+5 )
        console::parse_var(argv[index+5],step);
    std::cout << "pattern: " << pattern << ", mask: " << maskPath << ", frames: " << startFrame << ":" << step << ":" << endFrame << std::endl;
    cv::Mat mask = cv::imread(maskPath, cv::IMREAD_GRAYSCALE);
    cv::imshow("mask",mask);
    cv::waitKey();
    mkdir("sample",0755);

    size_t id(0lu);
    for ( size_t frameId = startFrame; frameId <= endFrame; frameId += step, ++id )
    {
        char path[255];
        char oPath[255];

        sprintf(path, pattern.c_str(), frameId);
        cv::Mat img = cv::imread(path, cv::IMREAD_UNCHANGED);

        sprintf(oPath,"sample/masked_%05lu.ppm",id);

        cv::Mat masked;
        img.copyTo(masked,mask);
        cv::imshow("masked",masked);
        cv::waitKey(40);

        tracking::writeRgb(masked, oPath);
        std::cout << "[" << __func__ << "]: " << "wrote to " << path << std::endl;
        sprintf(path,"depth_%05lu.pgm",frameId);

        sprintf(oPath,"sample/depth_%05lu.pgm",id);
        std::string cmd = "cp " + std::string(path) + " " + std::string(oPath);
        std::cout << cmd << std::endl;
        int ret = system(cmd.c_str());
        if ( ret )
            std::cout << "ret: " << ret << std::endl;
    }
    return EXIT_SUCCESS;
}