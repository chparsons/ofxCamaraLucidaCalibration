#include "Calibration.h"

using namespace ofxCv;

namespace cml
{ 

  //public:

  Calibration::Calibration()
  {
    _capture = false;
  };

  Calibration::~Calibration(){}; 

  void Calibration::toggle_capture()
  {
    _capture = !_capture;
  };

  //protected: 

  bool Calibration::update_cam( cv::Mat& camMat, ofPixels& pix, ofPixels& previous, ofPixels& diff, float* diffMean )
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

    for ( int i = 0; i < calibration.size(); i++ )
    {
      ofDrawBitmapStringHighlight(ofToString(i) + ": " + ofToString(calibration.getReprojectionError(i)), x, y+100 + 16 * i, ofColor::magenta, ofColor::white);
    }
  }; 

  //see ofxCalibration::save
  void Calibration::save_intrinsics( ofxCv::Calibration& calibration, string name, string folder, string format )
  {
    if ( ! calibration.isReady() )
    {
      ofLogError("cml::Calibration") << "save_intrinsics for " << name << " failed, because your calibration isn't ready yet!";
    }

    string filename = folder + "/calib_" + name + "." + format + ".yml";
    bool absolute = false;

    cv::FileStorage fs( ofToDataPath(filename, absolute), cv::FileStorage::WRITE );

    const ofxCv::Intrinsics& distorted_intrinsics = calibration.getDistortedIntrinsics();

    cv::Size imageSize = distorted_intrinsics.getImageSize();
    cv::Size sensorSize = distorted_intrinsics.getSensorSize();
    cv::Mat cameraMatrix = distorted_intrinsics.getCameraMatrix();

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

    else if (format == "ofxcv")
    {
      fs << "cameraMatrix" << cameraMatrix;
      fs << "imageSize_width" << imageSize.width;
      fs << "imageSize_height" << imageSize.height;
      fs << "sensorSize_width" << sensorSize.width;
      fs << "sensorSize_height" << sensorSize.height;
      fs << "distCoeffs" << calibration.getDistCoeffs();
      fs << "reprojectionError" << calibration.getReprojectionError();
    }

    ofLogNotice("cml::Calibration") << "save intrinsics calib to file " << filename << ", format: " << format;
  }; 

  //cv::FileStorage Calibration::load_settings()
  //{
    //cv::FileStorage sett(ofToDataPath("settings.yml"), cv::FileStorage::READ);
    //if ( ! sett.isOpened() )
      //throw "calib setting not loaded";
    //return sett;
  //};

};

