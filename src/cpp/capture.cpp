#include <librealsense/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include<vector>
using namespace std;
using namespace cv;

int const TILE_W  = 640;
int const TILE_H = 480;
int const FRAMERATE = 60;
bool  _loop = true;
String const WINDOW_DEPTH = "Depth window";
String const WINDOW_RGB = "RGB window";
rs::context ctx;
rs::device & dev = *ctx.get_device(0);

bool initialize_streaming( )
{
       bool success = false;
       if (ctx.get_device_count() == 0) throw runtime_error("No device detected. Is it plugged in?");
       if(ctx.get_device_count()>0){
           dev.enable_stream( rs::stream::color, TILE_W, TILE_H, rs::format::rgb8, FRAMERATE );
           dev.enable_stream( rs::stream::depth, TILE_W, TILE_H, rs::format::z16, FRAMERATE );
           dev.set_option(rs::option::color_enable_auto_exposure, 1);
           dev.set_option(rs::option::color_enable_auto_white_balance, 1);
           vector<rs::stream> supported_streams;
//           // Compute field of view for each enabled stream
//           for (auto & stream : supported_streams)
//           {
//               if (!dev.is_stream_enabled(stream)) continue;
//               auto intrin = dev.get_stream_intrinsics(stream);
//               cout << "Capturing " << stream << " at " << intrin.width << " x " << intrin.height;
//               cout << setprecision(1) << fixed << ", fov = " << intrin.hfov() << " x " << intrin.vfov() << ", distortion = " << intrin.model() << endl;
//           }
           dev.start( );
           success = true;
}
       return success;
}

bool display_next_frame( )
{
//       // Create depth image
//       Mat depth16( TILE_H, TILE_W, CV_16U,   (uchar *)dev.get_frame_data( rs::stream::depth ) );

//       // Create color image
       Mat rgb( TILE_H, TILE_W, CV_8UC3,  (uchar *)dev.get_frame_data( rs::stream::color ) );

//       // < 800
//       Mat depth8u = depth16;
//       depth8u.convertTo( depth8u, CV_8UC1, 1.0);
//       cout<<dev.get_depth_scale()<<endl;
//       imshow( WINDOW_DEPTH, depth8u );
       cvtColor( rgb, rgb, cv::COLOR_BGR2RGB );
       imshow( WINDOW_RGB, rgb );
    Mat img( TILE_H, TILE_W, CV_8UC1);
    const uint16_t *ptr = (const uint16_t*) (dev.get_frame_data(rs::stream::depth));
    for (int i = 0; i < img.size().area(); i++) {
        const double depth = 1.0 * ptr[i];
        img.data[i] = (unsigned char)((1.0 - (depth - 550) / (2500 - 550)) * 255);
    }

    cv::imshow("realsense", img);


       int k = 0;
       if((k = waitKey(1)) == 27)
           return false;
       else
           return true;
}
int main() try
{
//    Mat colorImage  = Mat::zeros(Size(TILE_H, TILE_W), CV_8UC3);
//    Mat depthImage = Mat::zeros(Size(TILE_H, TILE_W), CV_16UC1);
//    Mat colorImageFlag  = Mat::zeros(Size(TILE_H, TILE_W), CV_8UC1);
//    Mat mask  = Mat::zeros(Size(TILE_H, TILE_W), CV_8UC3);
    rs::log_to_console( rs::log_severity::warn );

    if( !initialize_streaming( ) )
    {
          std::cout << "Unable to locate a camera" << std::endl;
          rs::log_to_console( rs::log_severity::fatal );
          return EXIT_FAILURE;
    }

    while( _loop )
    {
          if( dev.is_streaming( ) )
                 dev.wait_for_frames( );

          _loop = display_next_frame( );
    }
    dev.stop( );
    destroyAllWindows( );

    return EXIT_SUCCESS;

}
catch (const rs::error & e)
{
    cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << endl;
    return EXIT_FAILURE;
}
catch (const exception & e)
{
    cerr << e.what() << endl;
    return EXIT_FAILURE;
}

