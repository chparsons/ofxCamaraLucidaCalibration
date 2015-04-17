/*
 * Authors: 
 * Mariano Tepper <mtepper@dc.uba.ar>
 * Christian Parsons <http://chparsons.com.ar/>
 */

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

      void init( 
          ofPixels& pix, 
          string cam_calib_file, 
          string pattern_settings_file,
          string cam_name = "camera", 
          string proj_name = "projector" );

      void update( ofPixels& pix ); 
      bool capture( ofPixels& pix );
      void calibrate();
      void render( int x, int y );
      void render_chessboard( int x = ofGetScreenWidth(), int y = 0, int brightness = 255 );

      void save_all( string folder );
      void save_images( string folder );
      void load_images( string folder );
      void reset();

      string log_config();

      cv::Size cam_size()
      {
        return calib_cam.getDistortedIntrinsics().getImageSize();
      };

      cv::Size proj_size()
      {
        return cfg_proj.image_size;
      };

    private:

      int width, height, chan;

      string cam_name, proj_name;
      string cam_calib_file, pattern_settings_file;

      ofxCv::Calibration calib_cam, calib_proj;
      ofxCv::Intrinsics proj_distorted_intrinsics;

      //settings
      cml::Calibration::Config cfg_proj;
      float offset_x_3x3, offset_y_3x3;
      float offset_x_3x4, offset_y_3x4;
      float offset_x_3x5, offset_y_3x5;
      float offset_x_3x6, offset_y_3x6;

      vector<ofImage> imgs;
      vector<cv::Point2f> captured_printed_points;
      vector<cv::Point2f> captured_projected_points;
      int projector_pattern_size();

      //calib output
      cml::Calibration::Extrinsics extrinsics;
      cv::Mat1d proj_intrinsics;
      cv::Mat1d proj_distortion;

      //ofPixels previous, diff;
      //cv::Mat camMat;
      //float diffMean; 
      //void allocate( ofPixels& pix ); 

      bool update_captured_points( ofImage& img );
      void save_stereo_RT( string folder );
      bool load_settings( string pattern_settings_file );

      bool calibrate_projector_intrinsics( 
          vector< vector<cv::Point3f> >& projected_points3d_on_board, 
          vector< vector<cv::Point2f> >& projector_pattern );

      void calibrate_projector_camera( 
          vector< vector<cv::Point3f> >& projected_points3d_on_board, 
          vector< vector<cv::Point2f> >& projector_pattern, 
          vector< vector<cv::Point2f> >& projected_points );

      void make_printed_pattern( 
          vector<cv::Point2f>& printed_pattern );

      void make_projector_pattern( 
          vector< vector<cv::Point2f> >& projector_pattern );

      bool find_homographies( 
          vector< vector<cv::Point2f> >& printed_points,
          vector<cv::Mat1d>& homographies ); 

      bool find_printed_points( 
          vector< vector<cv::Point2f> >& printed_points ); 

      bool find_projected_points( 
          vector< vector<cv::Point2f> >& projected_points );

      void find_printed_chessboards(
          const cv::Mat& img, 
          vector<cv::Point2f>& corners);

      void find_projected_chessboards(
          const cv::Mat& img, 
          vector<cv::Point2f>& corners);

      bool projected_points_on_board( 
          vector< vector<cv::Point2f> >& projected_points, 
          vector< cv::Mat1d >& homographies, 
          vector< vector<cv::Point3f> >& projected_points3d_on_board ); 

      //void find_chessboards(
          //const cv::Mat& img, 
          //vector<cv::Point2f>& corners,
          //bool add_printed_corners = true,
          //bool add_projector_corners = true,
          //int x = -1, int y = -1 ); 

      bool find_chessboard_roi(
          int width, int height, 
          cv::Mat& frame, 
          cv::Mat& dst_image,
          vector<cv::Point2f>& corners);

      //functions from 
      //https://github.com/rgbdemo/nestk/blob/master/ntk/camera/calibration.h

      //chessboard pattern type only
      void find_chessboard_corners(
          int pattern_width, int pattern_height,
          vector<cv::Point2f>& corners,
          const cv::Mat& src_image,
          cv::Mat& dst_image,
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

