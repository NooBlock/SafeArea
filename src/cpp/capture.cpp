#include <librealsense/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include"../hpp/pointcloud.hpp"
#include<vector>
#include<omp.h>

using namespace std;
using namespace cv;

int const TILE_W  = 640;
int const TILE_H = 480;
int const FRAMERATE = 60;
bool  _loop = true;
String const WINDOW_DEPTH = "Depth window";
String const WINDOW_RGB = "RGB window";
String const WINDOW_PC = "PointCloud window";
String const WINDOW_COLOR_ALIGHED_TO_DEPTH = "Color_alighed_to_Depth";
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
           dev.start( );

           success = true;
}
       return success;
}

bool exit_(){
    int k = 0;
    if((k = waitKey(1)) == 27)
        return false;
    else
        return true;
}
void change_color(const Mat& depth){
      Mat depthcolor( TILE_H, TILE_W, CV_8UC3);
      for (int i = 0; i < TILE_H; i++) {
//#pragma omp parallel for num_threads(4)
          for(int j = 0;j<TILE_W;j++){
              int p = (int)depth.at<uchar>(i,j);
          if (0<p && p< 32){
              depthcolor.at<Vec3b>(i,j) [0] = 0;
              depthcolor.at<Vec3b>(i,j) [1] = 255;
              depthcolor.at<Vec3b>(i,j) [2] = 255;
          }else if(33<p && p < 64){
              depthcolor.at<Vec3b>(i,j) [0] = 0;
              depthcolor.at<Vec3b>(i,j) [1] = 25;
              depthcolor.at<Vec3b>(i,j) [2] = 25;
          }else if(65<p && p< 96){
              depthcolor.at<Vec3b>(i,j) [0] = 0;
              depthcolor.at<Vec3b>(i,j) [1] = 55;
              depthcolor.at<Vec3b>(i,j) [2] = 255;
          }else if(97<p && p < 128){
              depthcolor.at<Vec3b>(i,j) [0] = 0;
              depthcolor.at<Vec3b>(i,j) [1] = 0;
              depthcolor.at<Vec3b>(i,j) [2] = 255;
          }else if(129<p && p < 160){
              depthcolor.at<Vec3b>(i,j) [0] = 255;
              depthcolor.at<Vec3b>(i,j) [1] = 0;
              depthcolor.at<Vec3b>(i,j) [2] = 255;
         }else if(161<p && p < 192){
              depthcolor.at<Vec3b>(i,j) [0] = 0;
              depthcolor.at<Vec3b>(i,j) [1] = 255;
              depthcolor.at<Vec3b>(i,j) [2] = 0;
         }else if(193<p && p < 224){
              depthcolor.at<Vec3b>(i,j) [0] = 100;
              depthcolor.at<Vec3b>(i,j) [1] = 255;
              depthcolor.at<Vec3b>(i,j) [2] = 0;
         }else{
              depthcolor.at<Vec3b>(i,j) [0] = 10;
              depthcolor.at<Vec3b>(i,j) [1] = 0;
              depthcolor.at<Vec3b>(i,j) [2] = 255;
         }
     }
      cv::imshow("depthcolor", depthcolor);
    }
}
//void projection(const Mat& depth){
//    Mat depth_projection( TILE_H, TILE_W, CV_8UC3);
//}
bool display_next_frame( )
{
//    rs::intrinsics depth_intrin = dev.get_stream_intrinsics(rs::stream::depth);
//    rs::extrinsics depth_to_color = dev.get_extrinsics(rs::stream::depth, rs::stream::color);
//    rs::intrinsics color_intrin = dev.get_stream_intrinsics(rs::stream::color);
//    float scale = dev.get_depth_scale();
    const uint16_t * depth_image = (const uint16_t *)dev.get_frame_data(rs::stream::depth);
    const uint8_t * color_image = (const uint8_t *)dev.get_frame_data(rs::stream::color);
    const uchar * color_to_depth_frame = (const uchar*)(dev.get_frame_data(rs::stream::color_aligned_to_depth));

    // create color image and color aligned to depth image
    Mat rgb( TILE_H, TILE_W, CV_8UC3,  (uchar *)color_image );
    Mat color_aligned_to_depth(TILE_H, TILE_W, CV_8UC3, (uchar *)color_to_depth_frame);
    Mat depthImg(TILE_H, TILE_W, CV_16UC1, (uint16_t  *)depth_image);

    cvtColor( rgb, rgb, cv::COLOR_BGR2RGB );
     cvtColor( color_aligned_to_depth, color_aligned_to_depth, cv::COLOR_BGR2RGB );
     imshow( WINDOW_RGB, rgb );
     imshow(WINDOW_COLOR_ALIGHED_TO_DEPTH, color_aligned_to_depth);
    Mat img( TILE_H, TILE_W, CV_8UC1);
    const uint16_t *ptr = (const uint16_t*) depth_image;
    for (int i = 0; i < img.size().area(); i++) {
        const double depth = 1.0 * ptr[i];
        img.data[i] = (unsigned char)((1.0 - (depth - 550) / (2500 - 550)) * 255);
//        cout<<img.data[i]<<endl;
    }
    cv::imshow(WINDOW_DEPTH, img);
//    change_color(img);

    return exit_();
}

int main() try
{

    rs::log_to_console( rs::log_severity::warn );
    if( !initialize_streaming( ) )
    {
          std::cout << "Unable to locate a camera" << std::endl;
          rs::log_to_console( rs::log_severity::fatal );
          return EXIT_FAILURE;
    }
// -   GLFWwindow *win = point_cloud_init();
//    point_cloud(dev);
//    glEnd();
//    glfwTerminate();
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

