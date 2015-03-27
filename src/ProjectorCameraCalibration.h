#pragma once

#include "Calibration.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"

using namespace ofxCv;

namespace cml
{  

  class ProjectorCameraCalibration : public Calibration
  {
    public:

      ProjectorCameraCalibration();
      ~ProjectorCameraCalibration(); 

      void init( ofPixels& pix, string cam_calib_filename, string cam_name = "camera", string proj_name = "projector" );
      void update( ofPixels& pix ); 
      void calibrate();
      void render();

      void save_all( string folder );
      void reset();

    private:

      int w, h, chan;

      string cam_name, proj_name;
      ofxCv::Calibration calib_cam, calib_proj;

      //settings
      cml::Calibration::Config cfg_proj;
      float offset_x_3x3, offset_y_3x3;
      float offset_x_3x4, offset_y_3x4;
      float offset_x_3x5, offset_y_3x5;
      float offset_x_3x6, offset_y_3x6;

      vector<ofImage> imgs;
      cv::Mat1d R, T;
      cv::Mat1d proj_intrinsics;
      cv::Mat1d proj_distortion;

      ofPixels previous, diff;
      cv::Mat camMat;
      float diffMean; 

      void allocate( ofPixels& pix ); 
      void save_stereo_RT( string folder );
      void load_settings(); //TODO load from settings.yml

      void calibrate_projector_intrinsics( 
          vector< vector<cv::Point3f> >& projected_points3d_on_board, 
          vector< vector<cv::Point2f> >& projector_pattern );

      void calibrate_projector_camera( 
          vector< vector<cv::Point3f> >& projected_points3d_on_board, 
          vector< vector<cv::Point2f> >& projector_pattern, 
          vector< vector<cv::Point2f> >& projected_points );

      void find_homographies( 
          vector<cv::Point2f>& printed_points, 
          vector<cv::Point2f>& printed_pattern, 
          vector<cv::Mat1d>& homographies );

      void projected_points_on_board( 
          vector< vector<cv::Point2f> >& projected_points, 
          vector< cv::Mat1d >& homographies, 
          vector< vector<cv::Point3f> >& projected_points3d_on_board );

      bool find_printed_points( 
          vector<cv::Point2f>& printed_points );

      bool find_projected_points( 
          vector< vector<cv::Point2f> >& projected_points );

      void make_printed_pattern( 
          vector<cv::Point2f>& printed_pattern );

      void make_projector_pattern( 
          vector< vector<cv::Point2f> >& projector_pattern ); 

      bool find_printed_chessboards(
          const cv::Mat& img, 
          vector<cv::Point2f>& corners);

      void find_chessboard_roi(
          int width, int height, 
          cv::Mat& frame, 
          vector<cv::Point2f>& corners );


      //functions from 
      //https://github.com/rgbdemo/nestk/blob/master/ntk/camera/calibration.h

      //chessboard pattern only
      void find_chessboard_corners(
          int pattern_width, int pattern_height,
          vector<Point2f>& corners,
          const cv::Mat& image,
          float scale_factor); 

      void make_pattern(
          vector< vector<Point3f> >& output,
          int pattern_width,
          int pattern_height,
          float square_size,
          int nb_images);

      double computeCalibrationError(
          const cv::Mat& F,
          const vector< vector<cv::Point2f> >& rgb_corners,
          const vector< vector<cv::Point2f> >& depth_corners); 

      static bool compX(const cv::Point2f& a, const cv::Point2f& b) { return a.x < b.x; };
      static bool compY(const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; };

  };
};

