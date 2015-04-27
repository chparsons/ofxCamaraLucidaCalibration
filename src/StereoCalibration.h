#pragma once

#include "Calibration.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"

using namespace ofxCv;

namespace cml
{
  class StereoCalibration 
    : public Calibration
  {
    public:

      StereoCalibration() : Calibration() {};
      ~StereoCalibration(){}; 

      void init( 
          string pattern_settings_file,
          string name0, ofPixels& pix0, 
          string name1, ofPixels& pix1,
          string calib0_file = "", 
          string calib1_file = "" )
      {
        this->name0 = name0;
        this->name1 = name1;
        this->calib0_file = calib0_file;
        this->calib1_file = calib1_file;

        //cml::Calibration
        diffThreshold = 6.; //for manual capture
        timeThreshold = 1;

        w = pix0.getWidth();
        h = pix0.getHeight();
        chan = pix0.getNumChannels();

        load_settings(pattern_settings_file);
        init_calib( calib0, cfg );
        init_calib( calib1, cfg );

        preload_calibration( calib0, calib0_file, name0 );
        preload_calibration( calib1, calib1_file, name1 );

        allocate( pix0, pix1 );
      };

      void update( ofPixels& pix0, ofPixels& pix1 )
      {
        curTime = ofGetElapsedTimef(); 

        if ( !_capture || curTime - lastTime < timeThreshold )
          return;

        bool _updated0 = update_cam( camMat0, pix0, previous0, diff0, &diffMean0 );
        bool _updated1 = update_cam( camMat1, pix1, previous1, diff1, &diffMean1 );

        if ( !_updated0 || !_updated1 )
          return; 

        if ( !find_board( calib0, camMat0 ) || !find_board( calib1, camMat1 ) )
        {
          ofLogWarning() << "did not found the chessboard on any camera";
          capture_failed();
          return;
        }

        //double check
        if ( !calib0.add( camMat0 )
            || !calib1.add( camMat1 ) )
        {
          ofLogError() << "!@#$%ˆ&*";
          capture_failed();
          return;
        }

        //only calibrate if it was not preloaded
        if ( calib0_file.empty() )
          calib0.calibrate();

        if ( calib1_file.empty() )
          calib1.calibrate();

        undistort( calib0, pix0, undistorted0 );
        undistort( calib1, pix1, undistorted1 );

        lastTime = curTime;

        capture_success();
      };

      void render()
      {
        ofDrawBitmapStringHighlight("movement "+name0+": " + ofToString(diffMean0), 0, 20, ofColor::cyan, ofColor::black);        

        ofDrawBitmapStringHighlight("movement "+name1+": " + ofToString(diffMean1), w, 20, ofColor::cyan, ofColor::black); 

        render_calib(calib0, 0);
        render_calib(calib1, w);

        undistorted0.draw( 0, h );
        undistorted1.draw( w, h );

        debug_calib(calib0, name0, 0);
        debug_calib(calib1, name1, w);

        //we are not calibrating a camera if it has a preloaded calibration, so we wont have reproj errors per view
        if ( calib0_file.empty() ) 
          debug_reproj_errors_per_view( calib0, 0, 100 );
        if ( calib1_file.empty() ) 
          debug_reproj_errors_per_view( calib1, w, 100 );

        render_capture_status();
      };

      void save_all( string folder )
      {
        save_intrinsics( name0, folder, "ofxcv" );
        save_intrinsics( name0, folder, "aruco" );

        save_intrinsics( name1, folder, "ofxcv" );
        save_intrinsics( name1, folder, "aruco" );

        save_extrinsics( name0, name1, folder );
        save_extrinsics( name1, name0, folder );
      };

      void save_intrinsics( string name, string folder, string format = "ofxcv" )
      {
        if (name == name0) 
          cml::Calibration::save_intrinsics( calib0, name0, folder, format );
        else if (name == name1) 
          cml::Calibration::save_intrinsics( calib1, name1, folder, format );
      }; 

      void save_extrinsics( string src_name, string dst_name, string folder  )
      {
        ofxCv::Calibration& src_calib = src_name == name0 ? calib0 : calib1;
        ofxCv::Calibration& dst_calib = dst_name == name0 ? calib0 : calib1;
        save_extrinsics( src_name, src_calib, dst_name, dst_calib, folder );
      }; 

      void reset()
      {
        calib0.reset();
        calib1.reset();
      }; 

      void removeLast()
      {
        if (calib0.imagePoints.size() > 0)
        {
          calib0.imagePoints.pop_back();
          calib1.imagePoints.pop_back();
        }
      };

    private:

      int w, h, chan; 

      ofxCv::Calibration calib0, calib1;
      string calib0_file, calib1_file;

      string name0, name1;
      ofImage undistorted0, undistorted1;
      ofPixels previous0, previous1;
      ofPixels diff0, diff1;
      //ofPixels pix0, pix1;
      cv::Mat camMat0, camMat1;
      float diffMean0, diffMean1; 

      cml::Calibration::Config cfg;
      cml::Calibration::Extrinsics extrinsics;

      //see ofxCv::getTransformation
      bool calibrate_extrinsics( ofxCv::Calibration& src_calib, ofxCv::Calibration& dst_calib, cml::Calibration::Extrinsics& extrinsics ) 
      {

        if ( !src_calib.isReady() || !dst_calib.isReady() ) 
        {
          ofLogError() << "calibrate_extrinsics() requires both Calibration objects to have just been calibrated";
          return false;
        }

        if ( src_calib.imagePoints.size() != dst_calib.imagePoints.size() ) 
        {
          ofLogError() << "calibrate_extrinsics() requires both Calibration objects to be trained simultaneously on the same board";
          return false;
        }

        ofxCv::Intrinsics src_distorted_intrinsics = src_calib.getDistortedIntrinsics();
        ofxCv::Intrinsics dst_distorted_intrinsics = dst_calib.getDistortedIntrinsics();

        cv::Size image_size = src_distorted_intrinsics.getImageSize();

        cv::Mat srcCameraMatrix = src_distorted_intrinsics.getCameraMatrix();
        cv::Mat srcDistCoeffs = src_calib.getDistCoeffs();

        cv::Mat dstCameraMatrix = dst_distorted_intrinsics.getCameraMatrix();
        cv::Mat dstDistCoeffs = dst_calib.getDistCoeffs();

        vector<Point3f> points = ofxCv::Calibration::createObjectPoints( src_calib.getPatternSize(), src_calib.getSquareSize(), cfg.pattern_type );
        vector< vector<Point3f> > objectPoints;
        objectPoints.resize( src_calib.imagePoints.size(), points );

        // uses CALIB_FIX_INTRINSIC by default
        cv::stereoCalibrate(
            objectPoints,
            src_calib.imagePoints, 
            dst_calib.imagePoints,
            srcCameraMatrix, 
            srcDistCoeffs,
            dstCameraMatrix,
            dstDistCoeffs,
            image_size, 
            //output
            extrinsics.R, extrinsics.T,
            extrinsics.E, extrinsics.F );

        return true;
      };

      bool find_board( ofxCv::Calibration& calibration, cv::Mat& camMat )
      {
        vector<Point2f> pointBuf;
        return calibration.findBoard(camMat, pointBuf);
      };

      void allocate( ofPixels& pix0, ofPixels& pix1 )
      {
        //pix0.allocate( w, h, chan );
        //pix1.allocate( w, h, chan );

        ofxCv::imitate(undistorted0, pix0);
        ofxCv::imitate(previous0, pix0);
        ofxCv::imitate(diff0, pix0);

        ofxCv::imitate(undistorted1, pix1);
        ofxCv::imitate(previous1, pix1);
        ofxCv::imitate(diff1, pix1);

        //undistorted.allocate( w, h, OF_IMAGE_COLOR );
        //previous.allocate( w, h, chan );
        //diff.allocate( w, h, chan );
      };

      void save_extrinsics( string src_name, ofxCv::Calibration& src_calib, string dst_name, ofxCv::Calibration& dst_calib, string folder )
      {

        if ( !calibrate_extrinsics( src_calib, dst_calib, extrinsics ) ) 
        {
          ofLogWarning("cml::StereoCalibration") << "calibrate extrinsics failed";
          return;
        }

        string filename = folder + "/extrinsics_" + src_name + "_to_" + dst_name + ".yml";
        bool absolute = false;

        cv::FileStorage fs( ofToDataPath(filename, absolute), cv::FileStorage::WRITE); 

        fs << "R" << extrinsics.R;
        fs << "T" << extrinsics.T;
        fs << "E" << extrinsics.E;
        fs << "F" << extrinsics.F;

        ofLogNotice("cml::StereoCalibration") << "save extrinsics: RT from [" << src_name << "] to [" << dst_name << "] to file " << filename;
      };

      void preload_calibration( ofxCv::Calibration& calib, string calib_file, string name )
      {
        if ( calib_file.empty() )
          return; 

        calib.load( calib_file, false ); 

        ofLogNotice("cml::StereoCalibration")
          << "preloaded [" << name 
          << "] calibration file " << calib_file;
      };

      bool load_settings( string pattern_settings_file )
      {
        cv::FileStorage settings( ofToDataPath(pattern_settings_file), cv::FileStorage::READ );

        if ( !settings.isOpened() )
        {
          ofLogFatalError() << "could not open pattern settings file: " << ofToDataPath(pattern_settings_file);
          ofExit();
          return false;
        }

        cfg.pattern_width = settings["pattern_width"];;
        cfg.pattern_height = settings["pattern_height"];
        cfg.pattern_square_size = settings["pattern_square_size"]; //cm
        cfg.pattern_type = CHESSBOARD;

      };

  };
};

