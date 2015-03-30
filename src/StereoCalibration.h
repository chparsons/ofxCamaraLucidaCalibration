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

      void init( string name0, ofPixels& pix0, string name1, ofPixels& pix1 )
      {
        this->name0 = name0;
        this->name1 = name1;

        diffThreshold = 6.; //2.5;
        timeThreshold = 1;
        //startCleaning = 10;
        _capture = false;
        lastTime = 0;

        w = pix0.getWidth();
        h = pix0.getHeight();
        chan = pix0.getNumChannels();

        cml::Calibration::Config cfg;
        cfg.pattern_width = 7;
        cfg.pattern_height = 10;
        cfg.pattern_square_size = 2.5;
        cfg.pattern_type = CHESSBOARD;

        init_calib( calib0, cfg );
        init_calib( calib1, cfg );

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
          return;
        }

        calibrate( calib0, camMat0, pix0, undistorted0 );
        calibrate( calib1, camMat1, pix1, undistorted1 );

        lastTime = curTime;
      };

      void render()
      {
        //drawHighlightString("movement "+name0+": " + ofToString(diffMean0), 0, 20, cyanPrint);
        ofDrawBitmapStringHighlight("movement "+name0+": " + ofToString(diffMean0), 0, 20, ofColor::cyan, ofColor::white);
        
        //drawHighlightString("movement "+name1+": " + ofToString(diffMean1), w, 20, cyanPrint);
        ofDrawBitmapStringHighlight("movement "+name1+": " + ofToString(diffMean1), w, 20, ofColor::cyan, ofColor::white);

        debug_calib(calib0, name0, 0);
        debug_calib(calib1, name1, w);

        render_calib(calib0, 0);
        render_calib(calib1, w);

        undistorted0.draw( 0, h );
        undistorted1.draw( w, h );
      };

      void save_all( string folder )
      {
        save_intrinsics( name0, folder );
        save_intrinsics( name0, folder, "aruco" );
        save_intrinsics( name1, folder );
        save_intrinsics( name1, folder, "aruco" );
        save_stereo_RT( name0, name1, folder );
        save_stereo_RT( name1, name0, folder );
      };

      void save_intrinsics( string name, string folder, string format = "ofxcv" )
      {
        if (name == name0) 
          cml::Calibration::save_intrinsics( calib0, name0, folder, format );
        else if (name == name1) 
          cml::Calibration::save_intrinsics( calib1, name1, folder, format );
      }; 

      void save_stereo_RT( string src_name, string dst_name, string folder  )
      {
        ofxCv::Calibration& src_calib = src_name == name0 ? calib0 : calib1;
        ofxCv::Calibration& dst_calib = dst_name == name0 ? calib0 : calib1;
        save_stereo_RT( src_name, src_calib, dst_name, dst_calib, folder );
      }; 

      void reset()
      {
        calib0.reset();
        calib1.reset();
      }; 

    private:

      int w, h, chan;

      ofxCv::Calibration calib0, calib1;
      ofImage undistorted0, undistorted1;
      ofPixels previous0, previous1;
      ofPixels diff0, diff1;
      //ofPixels pix0, pix1;
      cv::Mat camMat0, camMat1;
      float diffMean0, diffMean1; 

      bool calibrate( ofxCv::Calibration& calibration, cv::Mat& camMat, ofPixels& pix, ofImage& undistorted )
      {
        if ( !calibration.add( camMat ) )
        {
          ofLogError() << "update calib: this should not happen, we already found the chessboard !@#$%ˆ&*:)";
          return false;
        }

        //cout << "re-calibrating " << name << endl;

        calibration.calibrate();

        //if ( calibration.size() > startCleaning ) 
          //calibration.clean();
        //calibration.save("calib_"+name+".yml");

        //let users save
        //save_intrinsics( calibration, name, "aruco" );

        if ( calibration.size() > 0 ) 
        {
          calibration.undistort( toCv(pix), toCv(undistorted) );
          undistorted.update();
        }
      };

      bool find_board( ofxCv::Calibration& calibration, cv::Mat& camMat )
      {
        vector<Point2f> pointBuf;
        return calibration.findBoard(camMat, pointBuf);
      };

      string name0, name1;

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

      void save_stereo_RT( string src_name, ofxCv::Calibration& src_calib, string dst_name, ofxCv::Calibration& dst_calib, string folder )
      {
        Mat R,T;

        if ( ! src_calib.getTransformation( dst_calib, R, T ) ) 
        {
          ofLogWarning("cml::StereoCalibration") << "save stereo RT failed on ofxCalibration::getTransformation";
          return;
        }

        string filename = folder + "/calib_RT_" + src_name + "_to_" + dst_name + ".yml";
        bool absolute = false;

        cv::FileStorage fs( ofToDataPath(filename, absolute), cv::FileStorage::WRITE); 

        fs << "R" << R;
        fs << "T" << T;

        ofLogNotice("cml::StereoCalibration") << "save stereo RT from [" << src_name << "] to [" << dst_name << "] to file " << filename;
      };

  };
};

