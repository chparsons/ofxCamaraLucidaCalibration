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

      struct Extrinsics
      {
        cv::Mat1d R, T;
        cv::Mat E, F;
        double error;
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

      void capture_failed()
      {
        capture_status = -capture_time_status();
      };

      void capture_success()
      {
        capture_status = capture_time_status();
      };

      void render_capture_status()
      {
        if (capture_status == 0) //idle
          return;

        float duration = 10;
        float t = ofClamp(duration-(capture_time_status()-abs(capture_status)),0,duration) / duration;
        if (t <= 0.)
        {
          capture_status = 0; //idle
          return;
        }

        ofFloatColor col;
        if (capture_status < 0) //failed
          col = ofFloatColor::red;
        else 
          col = ofFloatColor::green; //success 
        col.a = t;

        ofPushStyle();
        ofSetColor( col );
        ofRect(0,0,ofGetWidth(),ofGetHeight());
        ofPopStyle();
      };

      //see ofxCalibration::save
      void save_intrinsics( ofxCv::Calibration& calibration, string name, string folder, string format = "ofxcv" ); 

      //cv::FileStorage load_settings();

      void render_mat( cv::Mat& img, int x, int y )
      {
        ofImage _img;
        ofxCv::toOf( img, _img );
        _img.draw(x,y);
      };

      void render_points( vector<cv::Point2f>& points, int x, int y, float scale, float size = 5. )
      {
        ofPushStyle();
        ofNoFill();
        ofSetColor( ofColor::red );
        for ( int i = 0; i < points.size(); i++ )
          ofCircle( x + (points[i].x * scale), y + (points[i].y * scale), size );
        ofPopStyle();
      };

      bool flt_eq(float lhs, float rhs, float epsilon = std::numeric_limits<float>::epsilon())
      {
        return std::abs(lhs - rhs) <= epsilon;
      };

    private:

      float capture_status; //0 idle, -capture_time_status() failed, capture_time_status() success

      float capture_time_status()
      {
        return ofGetFrameNum();
        //return ofGetElapsedTimef();
      };

  };
};

