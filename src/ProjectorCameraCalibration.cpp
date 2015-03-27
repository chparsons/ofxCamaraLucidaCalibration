
#include "Calibration.h"
#include "ProjectorCameraCalibration.h"

using namespace ofxCv;

namespace cml
{

  //public:

  ProjectorCameraCalibration::ProjectorCameraCalibration() : Calibration() {};
  ProjectorCameraCalibration::~ProjectorCameraCalibration(){};

  void ProjectorCameraCalibration::init( 
      ofPixels& pix, 
      string cam_calib_filename, 
      string cam_name, 
      string proj_name )
  {
    this->cam_name = cam_name;
    this->proj_name = proj_name;

    diffThreshold = 6.; //2.5;
    timeThreshold = 1;
    lastTime = 0;

    capture = false;

    w = pix.getWidth();
    h = pix.getHeight();
    chan = pix.getNumChannels();

    load_settings(); //TODO load from settings.yml

    init_calib( calib_proj, cfg_proj );

    bool absolute = false;
    calib_cam.load( cam_calib_filename, absolute );

    allocate( pix ); 
  };

  void ProjectorCameraCalibration::update( ofPixels& pix )
  {
    curTime = ofGetElapsedTimef(); 

    if ( !capture || curTime - lastTime < timeThreshold )
      return;

    bool _updated = update_cam( camMat, pix, previous, diff, &diffMean );

    if ( !_updated )
      return;

    ofImage undistorted;
    ofxCv::imitate(undistorted, pix);
    calib_cam.undistort( toCv(pix), toCv(undistorted) );

    //TODO save to disk?
    imgs.push_back( undistorted );

    lastTime = curTime;
  };

  void ProjectorCameraCalibration::calibrate()
  {
    ofLogNotice("cml::ProjectorCameraCalibration") << "calibrate";

    vector<cv::Mat1d> homographies;
    vector<cv::Point2f> printed_pattern;
    vector<cv::Point2f> printed_points;
    vector< vector<cv::Point2f> > projector_pattern;
    vector< vector<cv::Point2f> > projected_points;
    vector< vector<cv::Point3f> > projected_points3d_on_board;


    //1) build 4 printed and projector patterns
    //(i.e. 2d points projected onto the image plane)
    make_printed_pattern( printed_pattern );
    make_projector_pattern( projector_pattern );

    //2) find 4 printed patterns points on camera (progressive white roi)
    bool found_printed = find_printed_points( printed_points );
    if (!found_printed) return;

    //3) find homography btw:
    //--printed points on undistorted camera (step 2)
    //--printed pattern points (step 1)
    find_homographies( printed_points, printed_pattern, homographies );

    //4) find projector pattern points on camera
    bool found_projected = find_projected_points( projected_points );
    //if (!found_projected) return;

    if ( homographies.size() != projected_points.size() )
    {
      ofLogError("cml::ProjectorCameraCalibration") << "error: homographies.size() = " << homographies.size() << " and projected_points.size() = " << projected_points.size() << " should be equal";
      return;
    }

    //5) transform projector pattern points on camera (step 4) to lie on the board plane => perspective transform using homography (step 3)
    //6) make 3d projector points on the board plane: flip Y and add Z
    projected_points_on_board( projected_points, homographies, projected_points3d_on_board );

    //7) calibrate projector intrinsics with: 
    //--projector pattern points on board from camera view (step 6) 
    //--projector pattern points
    calibrate_projector_intrinsics( projected_points3d_on_board, projector_pattern );

    //8) stereo calibrate with:
    //--projector pattern points on board from camera view (step 6) 
    //--projector pattern points
    //--projector pattern points on camera (step 4)
    calibrate_projector_camera( projected_points3d_on_board, projector_pattern, projected_points );

  };

  void ProjectorCameraCalibration::render()
  {
    ofDrawBitmapStringHighlight("movement: " + ofToString(diffMean), 0, 20, ofColor::cyan, ofColor::white);

    debug_calib(calib_cam, cam_name, 0, 0 );
    debug_calib(calib_proj, proj_name, 0, 480 );

    //render_calib(calib_cam, 0);

    if (imgs.size() > 0)
      imgs[imgs.size()-1].draw( 0, h );
  };

  void ProjectorCameraCalibration::save_all( string folder )
  {
    //cml::Calibration::save_intrinsics( calib_cam, cam_name ); //already calibrated
    cml::Calibration::save_intrinsics( calib_proj, proj_name, folder );
    save_stereo_RT( folder );
  };

  void ProjectorCameraCalibration::reset()
  {
    imgs.clear();
  };


  //private: 

  void ProjectorCameraCalibration::allocate( ofPixels& pix )
  {
    ofxCv::imitate(previous, pix);
    ofxCv::imitate(diff, pix);
  };

  //TODO load from settings.yml
  void ProjectorCameraCalibration::load_settings()
  {
    cfg_proj.image_size = cv::Size(1024, 768);
    cfg_proj.pattern_width = 7;
    cfg_proj.pattern_height = 10;
    cfg_proj.pattern_square_size = 0.04;
    cfg_proj.pattern_square_size_pixels = 40;

    //xy positions in meters of each 3xN printed patterns
    offset_x_3x3 = 0.66;
    offset_y_3x3 = 0.;
    offset_x_3x4 = 0.;
    offset_y_3x4 = 0.;
    offset_x_3x5 = 0.;
    offset_y_3x5 = 0.33;
    offset_x_3x6 = 0.66;
    offset_y_3x6 = 0.29;
  };

  void ProjectorCameraCalibration::save_stereo_RT( string folder )
  {
    string src_name = proj_name;
    string dst_name = cam_name;

    string filename = folder + "/calib_RT_" + src_name + "_to_" + dst_name + ".yml";
    bool absolute = false;

    cv::FileStorage fs( ofToDataPath(filename, absolute), cv::FileStorage::WRITE); 

    fs << "R" << R;
    fs << "T" << T;

    ofLogNotice("cml::ProjectorCameraCalibration") << "save stereo RT from [" << src_name << "] to [" << dst_name << "] to file " << filename;
  }; 

  //calibration

  void ProjectorCameraCalibration::calibrate_projector_intrinsics( 
      vector< vector<cv::Point3f> >& projected_points3d_on_board, 
      vector< vector<cv::Point2f> >& projector_pattern )
  {
    int flags = CV_CALIB_ZERO_TANGENT_DIST + CV_CALIB_FIX_K1 + CV_CALIB_FIX_K2 + CV_CALIB_FIX_K3;
    vector<cv::Mat> rvecs, tvecs;

    double error = cv::calibrateCamera(
        projected_points3d_on_board, 
        projector_pattern, 
        cfg_proj.image_size,
        //output
        proj_intrinsics, 
        proj_distortion,
        rvecs, tvecs, flags);

    ofLogNotice("ProjectorCameraCalibration") << "calibrate projector intrinsics:" 
      << "\n projector intrinsics: \n"
      << proj_intrinsics
      << "\n projector distortion: \n"
      << proj_distortion
      << "\n error int: \n"
      << error;

    //TODO update calib_proj from
    //proj_intrinsics
    //proj_distortion
  };

  void ProjectorCameraCalibration::calibrate_projector_camera( 
      vector< vector<cv::Point3f> >& projected_points3d_on_board, 
      vector< vector<cv::Point2f> >& projector_pattern, 
      vector< vector<cv::Point2f> >& projected_points )
  {

    cv::Mat E(3,3,CV_64F), F(3,3,CV_64F);
    cv::Mat zero_dist (proj_distortion.size(), proj_distortion.type());
    zero_dist = cv::Scalar(0);

    //const ofxCv::Intrinsics& 
    cv::Mat cam_distorted_intrinsics = calib_cam.getDistortedIntrinsics().getCameraMatrix();

    cv::stereoCalibrate(
        projected_points3d_on_board,
        projector_pattern, //i.e. projector pattern projected onto the projector image plane
        projected_points, //i.e. projector pattern projected onto the camera image plane
        //from proj intrinsics calib
        proj_intrinsics, 
        proj_distortion,
        //from cam intrinsics loaded
        cam_distorted_intrinsics, 
        zero_dist, //calib_cam.getDistCoeffs() ???
        cfg_proj.image_size,
        //output
        R, T, E, F,
        cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 50, 1e-6),
        cv::CALIB_FIX_INTRINSIC);

    double error = computeCalibrationError(F, projector_pattern, projected_points);

    ofLogNotice("ProjectorCameraCalibration") << "calibrate projector camera:" 
      << "\n average pixel reprojection error: " << error
      << "\n R: \n" << R
      << "\n T: \n" << T;

    //T = rgb_R * T - rgb_T;
    //R = rgb_R * R;
  };

  void ProjectorCameraCalibration::find_homographies( 
      vector<cv::Point2f>& printed_points, 
      vector<cv::Point2f>& printed_pattern, 
      vector<cv::Mat1d>& homographies )
  {
    for (int i = 0; i < imgs.size(); i++ )
    {
      cv::Mat1d homography = cv::findHomography( cv::Mat(printed_points), cv::Mat(printed_pattern), CV_RANSAC, 0.01 );
      homographies.push_back( homography );
    }
  };

  void ProjectorCameraCalibration::projected_points_on_board( 
      vector< vector<cv::Point2f> >& projected_points, 
      vector< cv::Mat1d >& homographies, 
      vector< vector<cv::Point3f> >& projected_points3d_on_board )
  { 

    vector< vector<cv::Point2f> > proj_pts2d;
    for (int i = 0; i < imgs.size(); i++)
    {
      cv::Mat1d homography = homographies[i];
      cv::Mat proj_pts_mat; 
      cv::perspectiveTransform(cv::Mat(projected_points[i]), proj_pts_mat, homography);
      proj_pts2d.push_back( proj_pts_mat );
    }

    projected_points3d_on_board.resize( proj_pts2d.size() );
    //projected_points3d_on_board( proj_pts2d.size() );

    for ( int i = 0; i < proj_pts2d.size(); i++ )
    {
      projected_points3d_on_board[i].resize( proj_pts2d[i].size() );

      for ( int k = 0; k < proj_pts2d[i].size(); k++ )
      {
        projected_points3d_on_board[i][k].x = proj_pts2d[i][k].x;
        projected_points3d_on_board[i][k].y = -proj_pts2d[i][k].y;
        projected_points3d_on_board[i][k].z = 0.;
      }
    }
  };

  bool ProjectorCameraCalibration::find_printed_points(
      vector<cv::Point2f>& printed_points )
  {
    for ( int i = 0; i < imgs.size(); i++ )
    {
      //ofImage& img = imgs[i];
      cv::Mat img = toCv(imgs[i]);
      bool found = find_printed_chessboards( img, printed_points );
      if (!found) 
      {
        ofLogWarning("cml::ProjectorCameraCalibration") << "chessboard patterns not found on image " << ofToString(i);
        return false;
      }
    }
    return true;
  };

  bool ProjectorCameraCalibration::find_projected_points( 
      vector< vector<cv::Point2f> >& projected_points )
  {
    projected_points.resize( imgs.size() );
    int projw = cfg_proj.pattern_width;
    int projh = cfg_proj.pattern_height;

    bool allfound = true;
    for ( int i = 0; i < imgs.size(); i++ )
    {
      //ofImage& img = imgs[i];
      cv::Mat img = toCv(imgs[i]); 

      vector<cv::Point2f> proj_corners;
      find_chessboard_corners( projw, projh, proj_corners, img, 1 );
      bool found = proj_corners.size() == projw * projh;

      if (found)
      {
        projected_points[i] = proj_corners; 
      }

      else
      {
        ofLogWarning("cml::ProjectorCameraCalibration") << "calibrate: projected pattern not found on image " << ofToString(i);
        projected_points[i].resize(0);
        allfound = false;
      }
    }

    return allfound;
  };

  void ProjectorCameraCalibration::make_printed_pattern(
      vector<cv::Point2f>& printed_pattern )
  {
    vector< vector<cv::Point3f> > _3x3, _3x4, _3x5, _3x6; 

    make_pattern(_3x3, 3,3, cfg_proj.pattern_square_size, 1);
    make_pattern(_3x4, 3,4, cfg_proj.pattern_square_size, 1);
    make_pattern(_3x5, 3,5, cfg_proj.pattern_square_size, 1);
    make_pattern(_3x6, 3,6, cfg_proj.pattern_square_size, 1);

    printed_pattern.resize( _3x3[0].size() + _3x4[0].size() + _3x5[0].size() + _3x6[0].size() );

    int i = 0;

    for (vector<cv::Point3f>::iterator it = _3x6[0].begin(); it != _3x6[0].end(); it++) 
    {
      printed_pattern[i].x = it->x + offset_x_3x6;
      printed_pattern[i].y = it->y + offset_y_3x6;
      i++;
    }

    for (vector<cv::Point3f>::iterator it = _3x5[0].begin(); it != _3x5[0].end(); it++) 
    {
      printed_pattern[i].x = it->x + offset_x_3x5;
      printed_pattern[i].y = it->y + offset_y_3x5;
      i++;
    }

    for (vector<cv::Point3f>::iterator it = _3x4[0].begin(); it != _3x4[0].end(); it++) 
    {
      printed_pattern[i].x = it->x + offset_x_3x4;
      printed_pattern[i].y = it->y + offset_y_3x4;
      i++;
    }

    for (vector<cv::Point3f>::iterator it = _3x3[0].begin(); it != _3x3[0].end(); it++) 
    {
      printed_pattern[i].x = it->x + offset_x_3x3;
      printed_pattern[i].y = it->y + offset_y_3x3;
      i++;
    }

  };

  void ProjectorCameraCalibration::make_projector_pattern( 
      vector< vector<cv::Point2f> >& projector_pattern )
  {
    float offset_x = (cfg_proj.image_size.width - (cfg_proj.pattern_width-1) * cfg_proj.pattern_square_size_pixels) / 2;

    float offset_y = (cfg_proj.image_size.height - (cfg_proj.pattern_height-1) * cfg_proj.pattern_square_size_pixels) / 2;

    int nimgs = imgs.size();

    vector< vector<cv::Point3f> > corners3d;

    make_pattern( corners3d, cfg_proj.pattern_width, cfg_proj.pattern_height, cfg_proj.pattern_square_size_pixels, nimgs );

    projector_pattern.resize( nimgs );
    for (int i = 0; i < nimgs; i++) 
    {
      int npts = corners3d[i].size();
      projector_pattern[i].resize( npts );
      for (int j = 0; j < npts; j++) 
      {
        projector_pattern[i][j].x = corners3d[i][ npts-1-j ].x + offset_x;			
        projector_pattern[i][j].y = corners3d[i][ npts-1-j ].y + offset_y;			
      }
    }

  };

  bool ProjectorCameraCalibration::find_printed_chessboards(
      const cv::Mat& img, 
      vector<cv::Point2f>& corners)
  {
    cv::Mat copy = img.clone();
    vector<cv::Point2f> pts_proj, pts_3x6, pts_3x5, pts_3x4, pts_3x3; 

    find_chessboard_roi( cfg_proj.pattern_width, cfg_proj.pattern_height, copy, pts_proj ); //just for fun
    find_chessboard_roi( 3,6, copy, pts_3x6);
    find_chessboard_roi( 3,5, copy, pts_3x5);
    find_chessboard_roi( 3,4, copy, pts_3x4);
    find_chessboard_roi( 3,3, copy, pts_3x3);

    bool found = pts_3x6.size() != 0 && pts_3x5.size() != 0 && pts_3x4.size() != 0 && pts_3x3.size() != 0;

    if (!found) return false;

    corners.insert(corners.end(), pts_3x6.begin(), pts_3x6.end());
    corners.insert(corners.end(), pts_3x5.begin(), pts_3x5.end());
    corners.insert(corners.end(), pts_3x4.begin(), pts_3x4.end());
    corners.insert(corners.end(), pts_3x3.begin(), pts_3x3.end());

    return true;
  };

  void ProjectorCameraCalibration::find_chessboard_roi(
      int width, int height, 
      cv::Mat& frame, 
      vector<cv::Point2f>& corners )
  {

    float minX, minY, maxX, maxY;
    float offset = 12;

    find_chessboard_corners(width, height, corners, frame, 1);

    if (corners.size() == 0)
      return;

    minX = std::min_element(corners.begin(), corners.end(), compX)->x;
    maxX = std::max_element(corners.begin(), corners.end(), compX)->x;
    minY = std::min_element(corners.begin(), corners.end(), compY)->y;
    maxY = std::max_element(corners.begin(), corners.end(), compY)->y;

    minX = std::max(minX-offset, 0.f);
    maxX = std::min(maxX+offset, (float)cfg_proj.image_size.width);
    minY = std::max(minY-offset, 0.f);
    maxY = std::min(maxY+offset, (float)cfg_proj.image_size.height);
    //Make a rectangle
    cv::Rect roi(minX, minY, maxX-minX, maxY-minY);
    //Point a cv::Mat header at it (no allocation is done)
    cv::Mat image_roi = frame(roi);
    image_roi = cv::Scalar(255, 255, 255);
  };


  //functions from 
  //https://github.com/rgbdemo/nestk/blob/master/ntk/camera/calibration.h

  void ProjectorCameraCalibration::make_pattern(
      vector< vector<Point3f> >& output,
      int pattern_width,
      int pattern_height,
      float square_size,
      int nb_images)
  {
    const int nb_corners = pattern_width * pattern_height;

    output.resize(nb_images);
    for(int i = 0; i < nb_images; ++i)
    {
      output[i].resize(nb_corners);
      for(int j = 0; j < pattern_height; ++j)
        for(int k = 0; k < pattern_width; ++k)
        {
          output[i][j*pattern_width+k] = Point3f(k*square_size, j*square_size, 0);
        }
    }
  }; 

  //chessboard pattern only
  void ProjectorCameraCalibration::find_chessboard_corners(
      int pattern_width, int pattern_height,
      vector<Point2f>& corners,
      const cv::Mat& image,
      float scale_factor)
  {
    cv::Size pattern_size (pattern_width, pattern_height);

    cv::Mat scaled_image;

    if (flt_eq(scale_factor, 1))
    {
      scaled_image = image.clone();
    }
    else
    {
      cv::resize(
          image, scaled_image,
          cv::Size(image.cols*scale_factor, image.rows*scale_factor),
          scale_factor, scale_factor, cv::INTER_CUBIC);
    }

    int flags = CV_CALIB_CB_NORMALIZE_IMAGE|CV_CALIB_CB_ADAPTIVE_THRESH;

    bool ok = cv::findChessboardCorners(
        scaled_image,
        pattern_size,
        corners,
        flags);

    if (!ok)
    {
      flags = CV_CALIB_CB_NORMALIZE_IMAGE;
      ok = findChessboardCorners(
          scaled_image,
          pattern_size,
          corners,
          flags);
    }

    if (!ok)
    {
      flags = CV_CALIB_CB_ADAPTIVE_THRESH;
      ok = findChessboardCorners(
          scaled_image,
          pattern_size,
          corners,
          flags);
    }

    if (!ok)
    {
      flags = 0;
      ok = findChessboardCorners(
          scaled_image,
          pattern_size,
          corners,
          flags);
    }

    //cv::Mat draw_image = scaled_image;

    cv::Mat gray_image;
    cvtColor(image, gray_image, CV_BGR2GRAY);
    if (ok)
    {
      cornerSubPix(
          gray_image, corners, cv::Size(5,5), cv::Size(-1,-1),
          cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
    }

    if (ok)
    {
      for (int i = 0; i < corners.size(); ++i)
      {
        corners[i].x /= scale_factor;
        corners[i].y /= scale_factor;
      }
    }

    if (!ok)
    {
      corners.clear();
      return;
    }
  }; 

  double ProjectorCameraCalibration::computeCalibrationError(
      const cv::Mat& F,
      const vector< vector<Point2f> >& rgb_corners,
      const vector< vector<Point2f> >& depth_corners)
  {
    vector<cv::Point2f> points_in_rgb;
    for (int i = 0; i < rgb_corners.size(); ++i)
      for (int j = 0; j < rgb_corners[i].size(); ++j)
        points_in_rgb.push_back(rgb_corners[i][j]);

    vector<cv::Point2f> points_in_depth;
    for (int i = 0; i < depth_corners.size(); ++i)
      for (int j = 0; j < depth_corners[i].size(); ++j)
        points_in_depth.push_back(depth_corners[i][j]);

    vector<Vec3f> lines_in_depth;
    vector<Vec3f> lines_in_rgb;

    cv::computeCorrespondEpilines(cv::Mat(points_in_rgb), 1, F, lines_in_depth);
    cv::computeCorrespondEpilines(cv::Mat(points_in_depth), 2, F, lines_in_rgb);

    double avgErr = 0;
    for(int i = 0; i < points_in_rgb.size(); ++i)
    {
      double err = fabs(points_in_rgb[i].x*lines_in_rgb[i][0] +
          points_in_rgb[i].y*lines_in_rgb[i][1] + lines_in_rgb[i][2]);
      avgErr += err;
    }

    for(int i = 0; i < points_in_depth.size(); ++i)
    {
      double err = fabs(points_in_depth[i].x*lines_in_depth[i][0] +
          points_in_depth[i].y*lines_in_depth[i][1] + lines_in_depth[i][2]);
      avgErr += err;
    }

    return avgErr / (points_in_rgb.size() + points_in_depth.size());
  }; 

};

