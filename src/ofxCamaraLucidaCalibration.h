/*
 * Camara Lucida
 * www.camara-lucida.com.ar
 *
 * Copyright (C) 2015  Christian Parsons
 * www.chparsons.com.ar
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "ofxOpenCv.h"
#include "ofxCv.h"

using namespace ofxCv;

namespace cml
{
  class StereoCalibration
  {
    public:

      StereoCalibration(){};
      ~StereoCalibration(){}; 

      //void calibrate_kinect_rgb(std::vector< std::vector<Point2f> >& stereo_corners);
      //void calibrate_kinect_depth(std::vector< std::vector<Point2f> >& stereo_corners, std::vector< DepthCalibrationPoint >& depth_values);

      //void calibrate_kinect_stereo(const std::vector< std::vector<Point2f> >& undistorted_rgb_corners, const std::vector< std::vector<Point2f> >& undistorted_depth_corners);

      //// Taken from http://www.ros.org/wiki/kinect_calibration/technical
      //void estimate_depth_function(const std::vector<DepthCalibrationPoint>& points);

      void init( string name0, ofPixels& pix0, string name1, ofPixels& pix1 )
      {
        this->name0 = name0;
        this->name1 = name1;

        diffThreshold = 6.; //2.5;
        timeThreshold = 1;
        //startCleaning = 10;
        capture = false;
        lastTime = 0;

        w = pix0.getWidth();
        h = pix0.getHeight();
        chan = pix0.getNumChannels();

        xCount = 7;
        yCount = 10;
        squareSize = 2.5;
        patternType = CHESSBOARD; //CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID

        init_calib(calib0);
        init_calib(calib1);

        allocate( pix0, pix1 );
      };

      void update( ofPixels& pix0, ofPixels& pix1 )
      {
        curTime = ofGetElapsedTimef(); 

        bool _updated0 = update_camMat( camMat0, pix0, previous0, diff0, &diffMean0 );
        bool _updated1 = update_camMat( camMat1, pix1, previous1, diff1, &diffMean1 );

        if ( !_updated0 || !_updated1 )
          return;

        if ( !capture || curTime - lastTime < timeThreshold )
          return;

        if ( !find_board( calib0, camMat0 ) || !find_board( calib1, camMat1 ) )
        {
          ofLogWarning() << "did not found the chessboard on any camera";
          return;
        }

        update_calib( calib0, camMat0, pix0, undistorted0 );
        update_calib( calib1, camMat1, pix1, undistorted1 );

        lastTime = curTime;
      };

      void render()
      {
        drawHighlightString("movement "+name0+": " + ofToString(diffMean0), 0, 20, cyanPrint);
        drawHighlightString("movement "+name1+": " + ofToString(diffMean1), w, 20, cyanPrint);

        debug_calib(calib0, name0, 0);
        debug_calib(calib1, name1, w);

        render_calib(calib0, 0);
        render_calib(calib1, w);

        undistorted0.draw( 0, h );
        undistorted1.draw( w, h );

        //drawHighlightString("press: spacebar to capture / s to save / r to reset", 10, ofGetHeight()-30, magentaPrint);
      };

      void save_intrinsics( string name, string format = "" )
      {
        if (name == name0) save_intrinsics( calib0, name0, format );
        else if (name == name1) save_intrinsics( calib1, name1, format );
      }; 

      void save_stereo_RT( int src_id, int dst_id  )
      {
        string src_name = src_id == 0 ? name0 : name1;
        string dst_name = dst_id == 0 ? name0 : name1;
        ofxCv::Calibration& src_calib = src_id == 0 ? calib0 : calib1;
        ofxCv::Calibration& dst_calib = dst_id == 0 ? calib0 : calib1;
        save_stereo_RT( src_name, src_calib, dst_name, dst_calib );
      }; 

      void reset()
      {
        calib0.reset();
        calib1.reset();
      };

      void toggle_capture()
      {
        capture = !capture;
      };

    private:

      ofxCv::Calibration calib0, calib1;
      ofImage undistorted0, undistorted1;
      ofPixels previous0, previous1;
      ofPixels diff0, diff1;
      //ofPixels pix0, pix1;
      cv::Mat camMat0, camMat1;
      float diffMean0, diffMean1; 

      bool update_camMat( cv::Mat& camMat, ofPixels& pix, ofPixels& previous, ofPixels& diff, float* diffMean )
      {
        camMat = toCv(pix);
        Mat prevMat = toCv(previous);
        Mat diffMat = toCv(diff);

        //cout << "camMat " << camMat.cols << " x " << camMat.rows << endl;

        ofxCv::absdiff(prevMat,camMat,diffMat);	
        camMat.copyTo(prevMat);

        *diffMean = ofxCv::mean( Mat( ofxCv::mean(diffMat) ) )[0];

        return *diffMean < diffThreshold;
      };

      bool update_calib( ofxCv::Calibration& calibration, cv::Mat& camMat, ofPixels& pix, ofImage& undistorted )
      {
        if ( !calibration.add( camMat ) )
        {
          ofLogError() << "update calib: this should not happen, we already found the chessboard !@#$%ˆ&*";
          return false;
        }

        //cout << "re-calibrating " << name << endl;

        calibration.calibrate();

        //if ( calibration.size() > startCleaning ) 
          //calibration.clean();
        //calibration.save("calib_"+name+".yml");

        //let user save
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


      //calib settings
      int xCount, yCount;
      float squareSize;
      ofxCv::CalibrationPattern patternType = CHESSBOARD; 

      float curTime, lastTime;
      float diffThreshold; // maximum amount of movement
      float timeThreshold; // minimum time between snapshots
      //int startCleaning; // start cleaning outliers after this many samples

      bool capture;

      string name0, name1;

      int w, h, chan;

      cv::FileStorage load_settings()
      {
        cv::FileStorage sett(ofToDataPath("settings.yml"), cv::FileStorage::READ);
        if ( ! sett.isOpened() )
          throw "calib setting not loaded";
        return sett;
      };

      void init_calib( ofxCv::Calibration& calibration )
      { 
        calibration.setPatternSize(xCount, yCount);
        calibration.setSquareSize(squareSize);
        calibration.setPatternType(patternType);
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

      void render_calib( ofxCv::Calibration& calibration, int x )
      {
        if ( ! calibration.isReady() )
          return;
        ofPushMatrix();
        ofTranslate( x, 0, 0 );
        calibration.draw(calibration.size()-1);
        ofPopMatrix();
      };

      void debug_calib( ofxCv::Calibration& calibration, string name, int x )
      {
        drawHighlightString(name, x, 40, yellowPrint, ofColor(0));

        stringstream intrinsics;
        intrinsics << "fov: " << toOf(calibration.getDistortedIntrinsics().getFov()) << " distCoeffs: " << calibration.getDistCoeffs();

        drawHighlightString(intrinsics.str(), x, 60, yellowPrint, ofColor(0));

        drawHighlightString("reproj error: " + ofToString(calibration.getReprojectionError()) + " from " + ofToString(calibration.size()), x, 80, magentaPrint);

        for ( int i = 0; i < calibration.size(); i++ )
        {
          drawHighlightString(ofToString(i) + ": " + ofToString(calibration.getReprojectionError(i)), x, 100 + 16 * i, magentaPrint);
        }
      }; 

      //see ofxCalibration::save
      void save_intrinsics( ofxCv::Calibration& calibration, string name, string format = "" )
      {
        if ( ! calibration.isReady() )
        {
          ofLog(OF_LOG_ERROR, "save_intrinsics for " + name + " failed, because your calibration isn't ready yet!");
        }

        string format_ext = format == "aruco" ? ".aruco" : "";
        string filename = "calib_"+ name + format_ext +".yml";
        bool absolute = false;

        cv::FileStorage fs( ofToDataPath(filename, absolute), cv::FileStorage::WRITE );

        const ofxCv::Intrinsics& distorted_intrinsics = calibration.getDistortedIntrinsics();

        cv::Size imageSize = distorted_intrinsics.getImageSize();
        cv::Size sensorSize = distorted_intrinsics.getSensorSize();
        Mat cameraMatrix = distorted_intrinsics.getCameraMatrix();

        if (format == "aruco")
        {
          fs << "camera_matrix" << cameraMatrix;
          fs << "image_width" << imageSize.width;
          fs << "image_height" << imageSize.height;
          fs << "sensor_width" << sensorSize.width;
          fs << "sensor_height" << sensorSize.height;
          fs << "distortion_coefficients" << calibration.getDistCoeffs();
          fs << "reprojection_error" << calibration.getReprojectionError();
        }
        //ofxCv format
        else
        {
          fs << "cameraMatrix" << cameraMatrix;
          fs << "imageSize_width" << imageSize.width;
          fs << "imageSize_height" << imageSize.height;
          fs << "sensorSize_width" << sensorSize.width;
          fs << "sensorSize_height" << sensorSize.height;
          fs << "distCoeffs" << calibration.getDistCoeffs();
          fs << "reprojectionError" << calibration.getReprojectionError();
        }

        ofLog() << "save intrinsics calib to file " << filename << ", format: " << format;
      };

      void save_stereo_RT( string src_name, ofxCv::Calibration& src_calib, string dst_name, ofxCv::Calibration& dst_calib )
      {
        Mat R,T;

        if ( ! src_calib.getTransformation( dst_calib, R, T ) ) return;

        string filename = "calib_RT_" + src_name + "_to_" + dst_name + ".yml";
        bool absolute = false;

        cv::FileStorage fs( ofToDataPath(filename, absolute), cv::FileStorage::WRITE); 

        fs << "R" << R;
        fs << "T" << T;

        ofLog() << "save stereo RT from [" << src_name << "] to [" << dst_name << "] to file " << filename;
      };

  };
};

