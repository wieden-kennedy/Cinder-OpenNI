/*
* 
* Copyright (c) 2013, Wieden+Kennedy
* 
* Stephen Schieberl
* Michael Latzoni
*
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or 
* without modification, are permitted provided that the following 
* conditions are met:
* 
* Redistributions of source code must retain the above copyright 
* notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright 
* notice, this list of conditions and the following disclaimer in 
* the documentation and/or other materials provided with the 
* distribution.
* 
* Neither the name of the Ban the Rewind nor the names of its 
* contributors may be used to endorse or promote products 
* derived from this software without specific prior written 
* permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
*/

#include "cinder/app/AppBasic.h"
#include "cinder/Camera.h"
#include "Cinder-OpenNI.h"

/* 
* This application demonstrates how to display NiTE hands.
*/
class HandApp : public ci::app::AppBasic 
{
public:
	void						draw();
	void						keyDown( ci::app::KeyEvent event );
	void						prepareSettings( ci::app::AppBasic::Settings* settings );
	void						setup();
	void						update();
private:
	ci::CameraPersp				mCamera;

	ci::Channel16u				mChannel;
	OpenNI::DeviceRef			mDevice;
	OpenNI::DeviceManagerRef	mDeviceManager;
	std::vector<ci::Vec3f>		mHands;
	ci::gl::TextureRef			mTexture;
	void						onHand( nite::HandTrackerFrameRef frame, const OpenNI::DeviceOptions& deviceOptions );

	void						screenShot();
};

#include "cinder/gl/Texture.h"
#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"

using namespace ci;
using namespace ci::app;
using namespace std;

void HandApp::draw()
{
	gl::setViewport( getWindowBounds() );
	gl::clear();
	gl::setMatricesWindow( getWindowSize() );
	
	if ( mChannel ) {
		if ( mTexture ) {
			mTexture->update( Surface32f( mChannel ) );
		} else {
			mTexture = gl::Texture::create( mChannel );
		}
		gl::draw( mTexture, mTexture->getBounds(), getWindowBounds() );
	}

	gl::setMatrices( mCamera );
	for ( vector<Vec3f>::const_iterator iter = mHands.begin(); iter != mHands.end(); ++iter ) {
		gl::pushMatrices();
		gl::translate( *iter );
		for ( float i = 0.0f; i < 50.0f; i += 5.0f ) {
			gl::drawStrokedCircle( Vec2f::zero(), 10.0f + i, 32 );
		}
		gl::popMatrices();
	}
}

void HandApp::keyDown( KeyEvent event )
{
	switch ( event.getCode() ) {
	case KeyEvent::KEY_q:
		quit();
		break;
	case KeyEvent::KEY_f:
		setFullScreen( !isFullScreen() );
		break;
	case KeyEvent::KEY_s:
		screenShot();
		break;
	}
}

void HandApp::onHand( nite::HandTrackerFrameRef frame, const OpenNI::DeviceOptions& deviceOptions )
{
	mChannel							= OpenNI::toChannel16u( frame.getDepthFrame() );
	vector<nite::GestureData> gestures	= OpenNI::toVector( frame.getGestures() );
	vector<nite::HandData> hands		= OpenNI::toVector( frame.getHands() );

	// Use a wave gesture to trigger hand tracking
	for ( vector<nite::GestureData>::const_iterator iter = gestures.begin(); iter != gestures.end(); ++iter ) {
		if ( iter->isComplete() && iter->getType() == nite::GestureType::GESTURE_WAVE ) {
			nite::HandId id;
			mDevice->getHandTracker().startHandTracking( iter->getCurrentPosition(), &id );
		}
	}

	mHands.clear();
	for ( vector<nite::HandData>::const_iterator iter = hands.begin(); iter != hands.end(); ++iter ) {
		if ( iter->isLost() ) {
			mDevice->getHandTracker().stopHandTracking( iter->getId() );
		} else if ( iter->isTracking() ) {
			Vec3f position	= OpenNI::toVec3f( iter->getPosition() );
			position.x		= -position.x;
			mHands.push_back( position );
		}
	}
}

void HandApp::prepareSettings( Settings* settings )
{
	settings->setFrameRate( 60.0f );
	settings->setWindowSize( 800, 600 );
}

void HandApp::screenShot()
{
	writeImage( getAppPath() / fs::path( "frame" + toString( getElapsedFrames() ) + ".png" ), copyWindowSurface() );
}

void HandApp::setup()
{
	gl::color( ColorAf::white() );

	mDeviceManager		= OpenNI::DeviceManager::create();

	Vec2i windowSize	= toPixels( getWindowSize() );
	mCamera				= CameraPersp( windowSize.x, windowSize.y, 45.0f, 1.0f, 1000.0f );
	mCamera.lookAt( Vec3f::zero(), Vec3f::zAxis(), Vec3f::yAxis() );

	try {
		mDevice = mDeviceManager->createDevice( OpenNI::DeviceOptions().enableHandTracking().enableInfrared() );
	} catch ( OpenNI::ExcDeviceNotFound ex ) {
		console() << ex.what() << endl;
		quit();
		return;
	} catch ( OpenNI::ExcDeviceNotAvailable ex ) {
		console() << ex.what() << endl;
		quit();
		return;
	}
	
	mDevice->connectHandEventHandler( &HandApp::onHand, this );
	mDevice->start();
	mDevice->getHandTracker().startGestureDetection( nite::GestureType::GESTURE_WAVE );
}

void HandApp::update()
{
	mDeviceManager->update();
}

CINDER_APP_BASIC( HandApp, RendererGl )
