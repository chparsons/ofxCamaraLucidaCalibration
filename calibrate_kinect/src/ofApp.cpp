#include "ofApp.h"


void ofApp::setup()
{
  ofSetFrameRate(30);
  ofSetVerticalSync(true);
  ofSetLogLevel(OF_LOG_NOTICE); 

  kinect.setRegistration(true);
  // ir, rgb, texture
  kinect.init(true, true, true);
  kinect.open(); 

  //kinect.setDualRgbIR(true);

  w = kinect.getWidth();
  h = kinect.getHeight();

  kinect.update(); 
  pix_ir = kinect.getIRPixelsRef(); //copy
  pix_rgb = kinect.getPixelsRef(); //copy

  calibration.init( "kinect_ir", pix_ir, "kinect_rgb", pix_rgb );
}


void ofApp::exit() 
{
	kinect.close();
}


void ofApp::update()
{
  ofSetWindowTitle(ofToString(ofGetFrameRate(),2));

  kinect.update(); 

  //if ( !kinect.isFrameNew() )
    //return;

  pix_ir = kinect.getIRPixelsRef(); //copy
  pix_rgb = kinect.getPixelsRef(); //copy

  calibration.update( pix_ir, pix_rgb );
}


void ofApp::draw()
{
  ofBackground(100);
  ofSetColor(255); 

  kinect.drawIR( 0, 0, w, h );
  kinect.draw( w, 0, w, h );
  //kinect.drawDepth( 0, h, w, h );

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
    calibration.save_intrinsics( "kinect_ir" );
    calibration.save_intrinsics( "kinect_rgb" );
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

