#include "Calibration.h"

using namespace ofxCv;

namespace cml
{ 

  Calibration::Calibration()
  {
    _capture = false;
    capture_status = 0;
    diffThreshold = 2.5;
    timeThreshold = 3;
    lastTime = 0;
  };

  Calibration::~Calibration(){}; 

  void Calibration::toggle_capture()
  {
    _capture = !_capture;
  };

  void Calibration::undistort( ofxCv::Calibration& calib, ofPixels& pix, ofImage& undistorted )
  {
    if ( !calib.isReady() )
      return;

    calib.undistort( toCv(pix), toCv(undistorted) );
    undistorted.update();

    //cv::Mat undistorted_mat;
    //cv::Mat distorted_intrinsics = calib.getDistortedIntrinsics().getCameraMatrix();
    //cv::Mat distortion = calib.getDistCoeffs();
    //cv::undistort( toCv(pix), undistorted_mat, distorted_intrinsics, distortion );
    //ofImage undistorted;
    //ofxCv::imitate(undistorted, pix);
    //toOf(undistorted_mat, undistorted);
    //undistorted.update();
  };

  bool Calibration::update_cam( cv::Mat& camMat, ofPixels& pix, ofPixels& previous, ofPixels& diff, float* diffMean )
  {
    camMat = toCv(pix);
    cv::Mat prevMat = toCv(previous);
    cv::Mat diffMat = toCv(diff);

    //cout << "camMat " << camMat.cols << " x " << camMat.rows << endl;

    ofxCv::absdiff(prevMat,camMat,diffMat);	
    camMat.copyTo(prevMat);

    *diffMean = ofxCv::mean( cv::Mat( ofxCv::mean(diffMat) ) )[0];

    return *diffMean < diffThreshold;
  };

  void Calibration::init_calib( ofxCv::Calibration& calibration, Calibration::Config cfg  )
  { 
    calibration.setPatternSize(cfg.pattern_width, cfg.pattern_height);
    calibration.setSquareSize(cfg.pattern_square_size);
    calibration.setPatternType(cfg.pattern_type);
  }; 

  void Calibration::render_calib( ofxCv::Calibration& calibration, int x, int y )
  {
    if ( ! calibration.isReady() || calibration.size() == 0 )
      return;
    ofPushMatrix();
    ofTranslate( x, y, 0 );
    calibration.draw(calibration.size()-1);
    ofPopMatrix();
  };

  void Calibration::debug_calib( ofxCv::Calibration& calibration, string name, int x, int y )
  {
    ofDrawBitmapStringHighlight( name, x, y+40, ofColor::yellow, ofColor::black );

    stringstream intrinsics;
    intrinsics << "fov: " << toOf(calibration.getDistortedIntrinsics().getFov()) << " distCoeffs: " << calibration.getDistCoeffs();

    ofDrawBitmapStringHighlight( intrinsics.str(), x, y+60, ofColor::yellow, ofColor::black );

    ofDrawBitmapStringHighlight( "reproj error: " + ofToString(calibration.getReprojectionError()) + " from " + ofToString(calibration.size()), x, y+80, ofColor::magenta, ofColor::white );
  }; 

  void Calibration::debug_reproj_errors_per_view( ofxCv::Calibration& calibration, int x, int y )
  {
    for ( int i = 0; i < calibration.size(); i++ )
    {
      ofDrawBitmapStringHighlight(ofToString(i) + ": " + ofToString(calibration.getReprojectionError(i)), x, y + 16 * i, ofColor::magenta, ofColor::white);
    }
  };

  //see ofxCalibration::save
  void Calibration::save_intrinsics( ofxCv::Calibration& calibration, string name, string folder, string format )
  {
    save_intrinsics( calibration, name, folder, calibration.getReprojectionError(), format );
  };

  void Calibration::save_intrinsics( ofxCv::Calibration& calibration, string name, string folder, float reprojection_error, string format )
  {
    if ( ! calibration.isReady() )
    {
      ofLogError("cml::Calibration") << "save_intrinsics for " << name << " failed, because your calibration isn't ready yet!";
      return;
    }

    string filename = folder + "/intrinsics_" + name + "." + format + ".yml";
    bool absolute = false;

    cv::FileStorage fs( ofToDataPath(filename, absolute), cv::FileStorage::WRITE );

    const ofxCv::Intrinsics& distorted_intrinsics = calibration.getDistortedIntrinsics();
    cv::Size imageSize = distorted_intrinsics.getImageSize();
    cv::Size sensorSize = distorted_intrinsics.getSensorSize();
    cv::Mat cameraMatrix = distorted_intrinsics.getCameraMatrix();

    const ofxCv::Intrinsics& undistorted_intrinsics = calibration.getUndistortedIntrinsics();
    cv::Size undistorted_imageSize = undistorted_intrinsics.getImageSize();
    cv::Size undistorted_sensorSize = undistorted_intrinsics.getSensorSize();
    cv::Mat undistorted_cameraMatrix = undistorted_intrinsics.getCameraMatrix();

    if (format == "aruco")
    {
      fs << "camera_matrix" << cameraMatrix;
      fs << "image_width" << imageSize.width;
      fs << "image_height" << imageSize.height;
      fs << "sensor_width" << sensorSize.width;
      fs << "sensor_height" << sensorSize.height;
      fs << "distortion_coefficients" << calibration.getDistCoeffs();
      fs << "reprojection_error" << reprojection_error;

      //let's add the undistorted intrinsics
      fs << "undistorted_camera_matrix" << undistorted_cameraMatrix;
      fs << "undistorted_image_width" << undistorted_imageSize.width;
      fs << "undistorted_image_height" << undistorted_imageSize.height;
      fs << "undistorted_sensor_width" << undistorted_sensorSize.width;
      fs << "undistorted_sensor_height" << undistorted_sensorSize.height;
    }

    else if (format == "ofxcv")
    {
      fs << "cameraMatrix" << cameraMatrix;
      fs << "imageSize_width" << imageSize.width;
      fs << "imageSize_height" << imageSize.height;
      fs << "sensorSize_width" << sensorSize.width;
      fs << "sensorSize_height" << sensorSize.height;
      fs << "distCoeffs" << calibration.getDistCoeffs();
      fs << "reprojectionError" << reprojection_error;

      //let's add the undistorted intrinsics
      fs << "undistorted_cameraMatrix" << undistorted_cameraMatrix;
      fs << "undistorted_imageSize_width" << undistorted_imageSize.width;
      fs << "undistorted_imageSize_height" << undistorted_imageSize.height;
      fs << "undistorted_sensorSize_width" << undistorted_sensorSize.width;
      fs << "undistorted_sensorSize_height" << undistorted_sensorSize.height;
    }

    ofLogNotice("cml::Calibration") << "save intrinsics calib to file " << filename << ", format: " << format;
  }; 

};

