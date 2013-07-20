/*
* 
* Copyright (c) 2013, Ban the Rewind
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
#include "cinder/gl/Texture.h"
#include "Cinder-OpenNI.h"

/* 
* This application demonstrates how to display OpenNI's 
* depth stream.
*/
class BasicApp : public ci::app::AppBasic 
{
public:
	void					draw();
	void					keyDown( ci::app::KeyEvent event );
	void					prepareSettings( ci::app::AppBasic::Settings* settings );
	void					setup();
private:
	OpenNI::DeviceManager	mDeviceManager;
	OpenNI::DeviceRef		mDevice;
	ci::Channel16u			mChannel;
	void					onDepth( openni::VideoFrameRef frame );

	void					screenShot();
};

#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"

using namespace ci;
using namespace ci::app;
using namespace std;

void BasicApp::draw()
{
	gl::setViewport( getWindowBounds() );
	gl::clear( Colorf::black() );

	if ( mChannel ) {
		gl::TextureRef texture = gl::Texture::create( Channel8u( mChannel ) );
		gl::draw( texture, texture->getBounds(), getWindowBounds() );
	}
}

void BasicApp::keyDown( KeyEvent event )
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

void BasicApp::onDepth( openni::VideoFrameRef frame )
{
	mChannel = OpenNI::toChannel16u( frame );
}

void BasicApp::prepareSettings( Settings* settings )
{
	settings->setFrameRate( 60.0f );
	settings->setWindowSize( 800, 600 );
}

void BasicApp::screenShot()
{
	writeImage( getAppPath() / fs::path( "frame" + toString( getElapsedFrames() ) + ".png" ), copyWindowSurface() );
}

void BasicApp::setup()
{
	// The device manager automatically initializes OpenNI and NiTE.
	// It's a good idea to check that initialization is complete
	// before accessing devices.
	if ( mDeviceManager.isInitialized() ) {

		// Catching this exception prevents our application
		// from crashing when no devices are connected.
		try {
			mDevice = mDeviceManager.createDevice();
		} catch ( OpenNI::ExcDeviceNotAvailable ex ) {
			console() << ex.what() << endl;
		}
	
		// If we've successfully accessed a device, start and add a 
		// callback for the depth stream.
		if ( mDevice ) {
			mDevice->connectDepthEventHandler( &BasicApp::onDepth, this );
			mDevice->start();
		}
	}
}

CINDER_APP_BASIC( BasicApp, RendererGl )
