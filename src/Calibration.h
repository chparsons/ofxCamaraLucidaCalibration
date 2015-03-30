#pragma once

#include "ofxOpenCv.h"
#include "ofxCv.h"

using namespace ofxCv;

namespace cml
{ 

  class Calibration
  {
    public:

      struct Config
      {
        cv::Size image_size;
        int pattern_width, pattern_height;
        float pattern_square_size;
        float pattern_square_size_pixels;
        ofxCv::CalibrationPattern pattern_type = CHESSBOARD; //CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID
      };

      Calibration();
      ~Calibration(); 

      void toggle_capture();

    protected: 

      bool _capture;
      float curTime, lastTime;
      float diffThreshold; // maximum amount of movement
      float timeThreshold; // minimum time between snapshots
      //int startCleaning; // start cleaning outliers after this many samples 

      bool update_cam( cv::Mat& camMat, ofPixels& pix, ofPixels& previous, ofPixels& diff, float* diffMean );

      void init_calib( ofxCv::Calibration& calibration, Calibration::Config cfg ); 
      void render_calib( ofxCv::Calibration& calibration, int x, int y=0 );
      void debug_calib( ofxCv::Calibration& calibration, string name, int x, int y=0 ); 

      //see ofxCalibration::save
      void save_intrinsics( ofxCv::Calibration& calibration, string name, string folder, string format = "ofxcv" ); 

      //cv::FileStorage load_settings();

      void render_mat( cv::Mat& img, int x, int y )
      {
        ofImage _img;
        ofxCv::toOf( img, _img );
        _img.draw(x,y);
      };

      bool flt_eq(float lhs, float rhs, float epsilon = std::numeric_limits<float>::epsilon())
      {
        return std::abs(lhs - rhs) <= epsilon;
      };

  };
};

