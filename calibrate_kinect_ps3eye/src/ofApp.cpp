#include "ofApp.h"


void ofApp::setup()
{
  ofSetFrameRate(30);
  ofSetVerticalSync(true);
  ofSetLogLevel(OF_LOG_NOTICE);

  w = 640;
  h = 480;
  chan = 3;

  ps3.initGrabber(w, h);
  ps3.setAutogain(true);
  ps3.setAutoWhiteBalance(true);
  //ps3.setGain(uint8_t val);
  //ps3.setExposure(uint8_t val);
  //ps3.setSharpness(uint8_t val);
  //ps3.setContrast(uint8_t val);
  //ps3.setBrightness(uint8_t val);
  //ps3.setHue(uint8_t val);
  //ps3.setRedBalance(uint8_t val);
  //ps3.setBlueBalance(uint8_t val);

  kinect.setRegistration(true);
  // ir, rgb, texture
  kinect.init(true, true, true);
  kinect.open();

  kinect.update(); 
  ps3.update();

  pix_kinect_rgb = kinect.getIRPixelsRef(); //copy
  pix_ps3eye = ps3.getPixelsRef(); //copy
  pix_ps3eye.setNumChannels( chan );

  calibration.init( "kinect", pix_kinect_rgb, "ps3eye", pix_ps3eye );
}


void ofApp::exit() 
{
	kinect.close();
  ps3.close();
}


void ofApp::update()
{
  ofSetWindowTitle(ofToString(ofGetFrameRate(),2));

  kinect.update(); 
  ps3.update();

  if ( !kinect.isFrameNew() && !ps3.isFrameNew() ) return;

  if ( kinect.isFrameNew() )
    pix_kinect_rgb = kinect.getIRPixelsRef(); //copy

  if ( ps3.isFrameNew() )
  {
    ps3_tex.loadData(ps3.getPixelsRef());
    pix_ps3eye = ps3.getPixelsRef(); //copy
    pix_ps3eye.setNumChannels( chan );
  }

  calibration.update( pix_kinect_rgb, pix_ps3eye );
}


void ofApp::draw()
{
  ofBackground(100);
  ofSetColor(255); 

  //kinect.draw( 0, 0, w, h );
  kinect.drawIR( 0, 0, w, h );
  //kinect.drawDepth( 0, h, w, h );

  //ps3.draw( 0, w );
  ps3_tex.draw( w, 0 );

  calibration.render();
}


void ofApp::keyPressed(int key)
{
  if ( key == ' ' ) 
    calibration.toggle_capture(); 
}


void ofApp::keyReleased(int key)
{
  if ( key == ' ' ) 
    calibration.toggle_capture();

  else if ( key == 's' )
  {
    calibration.save_intrinsics( "kinect" );
    calibration.save_intrinsics( "ps3eye", "aruco" );
  }

  else if ( key == 'r' )
    calibration.reset();
}


void ofApp::mouseMoved(int x, int y )
{

}


void ofApp::mouseDragged(int x, int y, int button)
{

}


void ofApp::mousePressed(int x, int y, int button)
{

}


void ofApp::mouseReleased(int x, int y, int button)
{

}


void ofApp::windowResized(int w, int h)
{

}


void ofApp::gotMessage(ofMessage msg)
{

}


void ofApp::dragEvent(ofDragInfo dragInfo)
{ 

}

