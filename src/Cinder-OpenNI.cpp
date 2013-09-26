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

#include "Cinder-OpenNI.h"

#include "cinder/app/App.h"

namespace OpenNI
{
using namespace ci;
using namespace ci::app;
using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////

bool success( nite::Status status )
{
	switch ( status ) {
	case nite::Status::STATUS_BAD_USER_ID:
		console() << "Bad user ID" << endl;
		return false;
	case nite::Status::STATUS_ERROR:
		console() << "A NiTE error occurred" << endl;
		return false;
	case nite::Status::STATUS_OUT_OF_FLOW:
		console() << "NiTE out of flow error" << endl;
		return false;
	default:
		return true;
	}
}

bool success( openni::Status status )
{
	if ( status != openni::STATUS_OK ) {
		console() << openni::OpenNI::getExtendedError() << endl;
		return false;
	}
	return true;
}

AxisAlignedBox3f toAxisAlignedBox3f( const nite::Point3f& aMin, const nite::Point3f& aMax )
{
	return AxisAlignedBox3f( toVec3f( aMin ), toVec3f( aMax ) );
}

Planef toPlanef( const nite::Point3f& point, const nite::Point3f& normal )
{
	return Planef( toVec3f( point ), toVec3f( normal ) );
}

Quatf toQuatf( const nite::Quaternion& q )
{
	return Quatf( q.w, q.x, q.y, q.z );
}

Vec3f toVec3f( const nite::Point3f& v )
{
	return Vec3f( v.x, v.y, v.z );
}

Channel8u toChannel8u( const openni::VideoFrameRef& f )
{
	return Channel8u( f.getWidth(), f.getHeight(), f.getStrideInBytes(), 1, (uint8_t*)f.getData() );
}
	
Channel16u toChannel16u( const openni::VideoFrameRef& f )
{
	return Channel16u( f.getWidth(), f.getHeight(), f.getStrideInBytes(), 1, (uint16_t*)f.getData() );
}

Surface8u toSurface8u( const openni::VideoFrameRef& f )
{
	return Surface8u( (uint8_t*)f.getData(), f.getWidth(), f.getHeight(), f.getStrideInBytes(), SurfaceChannelOrder::RGB );
}
	
Surface16u toSurface16u( const openni::VideoFrameRef& f )
{
	return Surface16u( (uint16_t*)f.getData(), f.getWidth(), f.getHeight(), f.getStrideInBytes(), SurfaceChannelOrder::RGB );
}

//////////////////////////////////////////////////////////////////////////////////////////////

DeviceOptions::DeviceOptions()
{
	enableColor( false );
	enableDepth( true );
	enableHandTracking( false );
	enableInfrared( true );
	enableUserTracking( false );

	setColorFrameRate( 30 );
	setDepthFrameRate( 30 );
	setInfraredFrameRate( 30 );

	setColorPixelFormat( openni::PixelFormat::PIXEL_FORMAT_RGB888 );
	setDepthPixelFormat( openni::PixelFormat::PIXEL_FORMAT_DEPTH_100_UM );
	setInfraredPixelFormat( openni::PixelFormat::PIXEL_FORMAT_GRAY16 );

	setColorSize( Vec2i( 640, 480 ) );
	setDepthSize( Vec2i( 320, 240 ) );
	setInfraredSize( Vec2i( 320, 240 ) );

	setUri( "" );
}

bool DeviceOptions::isColorEnabled() const
{
	return mEnabledColor;
}

bool DeviceOptions::isDepthEnabled() const
{
	return mEnabledDepth;
}

bool DeviceOptions::isHandTrackingEnabled() const
{
	return mEnabledHandTracking;
}

bool DeviceOptions::isInfraredEnabled() const
{
	return mEnabledInfrared;
}

bool DeviceOptions::isUserTrackingEnabled() const
{
	return mEnabledUserTracking;
}

int32_t DeviceOptions::getColorFrameRate() const
{
	return mFrameRateColor;
}

int32_t DeviceOptions::getDepthFrameRate() const
{
	return mFrameRateDepth;
}
 
int32_t DeviceOptions::getInfraredFrameRate() const
{
	return mFrameRateInfrared;
}

openni::PixelFormat	DeviceOptions::getColorPixelFormat() const
{
	return mPixelFormatColor;
}

openni::PixelFormat	DeviceOptions::getDepthPixelFormat() const
{
	return mPixelFormatDepth;
}

openni::PixelFormat	DeviceOptions::getInfraredPixelFormat() const
{
	return mPixelFormatInfrared;
}

const Vec2i& DeviceOptions::getColorSize() const
{
	return mSizeColor;
}

const Vec2i& DeviceOptions::getDepthSize() const
{
	return mSizeDepth;
}

const Vec2i& DeviceOptions::getInfraredSize() const
{
	return mSizeInfrared;
}

const string& DeviceOptions::getUri() const
{
	return mUri;
}

DeviceOptions& DeviceOptions::enableColor( bool enable )
{
	mEnabledColor = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableDepth( bool enable )
{
	mEnabledDepth = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableHandTracking( bool enable )
{
	mEnabledHandTracking = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableInfrared( bool enable )
{
	mEnabledInfrared = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableUserTracking( bool enable )
{
	mEnabledUserTracking = enable;
	return *this;
}

DeviceOptions& DeviceOptions::setColorFrameRate( int32_t frameRate )
{
	mFrameRateColor = frameRate;
	return *this;
}

DeviceOptions& DeviceOptions::setDepthFrameRate( int32_t frameRate )
{
	mFrameRateDepth = frameRate;
	return *this;
}

DeviceOptions& DeviceOptions::setInfraredFrameRate( int32_t frameRate )
{
	mFrameRateInfrared = frameRate;
	return *this;
}

DeviceOptions& DeviceOptions::setColorPixelFormat( openni::PixelFormat format )
{
	mPixelFormatColor = format;
	return *this;
}

DeviceOptions& DeviceOptions::setDepthPixelFormat( openni::PixelFormat format )
{
	mPixelFormatDepth = format;
	return *this;
}

DeviceOptions& DeviceOptions::setInfraredPixelFormat( openni::PixelFormat format )
{
	mPixelFormatInfrared = format;
	return *this;
}

DeviceOptions& DeviceOptions::setColorSize( const Vec2i& size )
{
	mSizeColor = size;
	return *this;
}

DeviceOptions& DeviceOptions::setDepthSize( const Vec2i& size )
{
	mSizeDepth = size;
	return *this;
}

DeviceOptions& DeviceOptions::setInfraredSize( const Vec2i& size )
{
	mSizeInfrared = size;
	return *this;
}

void DeviceOptions::setUri( const std::string& uri )
{
	mUri = uri;
}

//////////////////////////////////////////////////////////////////////////////////////////////

Listener::Listener( DeviceOptions& deviceOptions )
	: mDeviceOptions( deviceOptions ), mNewFrame( false )
{
}

HandTrackerListener::HandTrackerListener( HandTrackerListener::EventHandler eventHandler, DeviceOptions& deviceOptions )
	: Listener( deviceOptions ), nite::HandTracker::NewFrameListener(), mEventHandler( eventHandler )
{
}

void HandTrackerListener::onNewFrame( nite::HandTracker& tracker )
{
	lock_guard<recursive_mutex> lock( mMutex );
	if ( !mNewFrame ) {
		success( tracker.readFrame( &mFrame ) );
		mNewFrame = true;
	}
}

void HandTrackerListener::update()
{
	if ( mNewFrame && mFrame.isValid() ) {
		mEventHandler( mFrame, mDeviceOptions );
		mNewFrame = false;
	}
}
	
UserTrackerListener::UserTrackerListener( UserTrackerListener::EventHandler eventHandler, DeviceOptions& deviceOptions )
	: Listener( deviceOptions ), nite::UserTracker::NewFrameListener(), mEventHandler( eventHandler )
{
}
	
void UserTrackerListener::onNewFrame( nite::UserTracker& tracker )
{
	lock_guard<recursive_mutex> lock( mMutex );
	if ( !mNewFrame ) {
		success( tracker.readFrame( &mFrame ) );
		mNewFrame = true;
	}
}

void UserTrackerListener::update()
{
	if ( mNewFrame && mFrame.isValid() ) {
		mEventHandler( mFrame, mDeviceOptions );
		mNewFrame = false;
	}
}

VideoStreamListener::VideoStreamListener( VideoStreamListener::EventHandler eventHandler, DeviceOptions& deviceOptions )
	: Listener( deviceOptions ), openni::VideoStream::NewFrameListener(), mEventHandler( eventHandler )
{
}

void VideoStreamListener::onNewFrame( openni::VideoStream& stream ) 
{
	lock_guard<recursive_mutex> lock( mMutex );
	if ( !mNewFrame ) {
		success( stream.readFrame( &mFrame ) );
		mNewFrame = true;
	}
}

void VideoStreamListener::update()
{
	if ( mNewFrame && mFrame.isValid() ) {
		mEventHandler( mFrame, mDeviceOptions );
		mNewFrame = false;
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////

Device::Device( const DeviceOptions& deviceOptions )
	: mConnected( false ), mDeviceOptions( deviceOptions ), mDeviceState( openni::DeviceState::DEVICE_STATE_NOT_READY ), 
	mListenerColor( 0 ), mListenerDepth( 0 ), mListenerHand( 0 ), mListenerInfrared( 0 ), mListenerUser( 0 )
{
	if ( success( mDevice.open( mDeviceOptions.getUri().c_str() ) ) ) {
		if ( mDevice.getSensorInfo( openni::SENSOR_COLOR ) == 0 ) {
			mDeviceOptions.enableColor( false );
		}
		if ( mDevice.getSensorInfo( openni::SENSOR_DEPTH ) == 0 ) {
			mDeviceOptions.enableDepth( false );
			mDeviceOptions.enableHandTracking( false );
			mDeviceOptions.enableUserTracking( false );
		}
		if ( mDevice.getSensorInfo( openni::SENSOR_IR ) == 0 ) {
			mDeviceOptions.enableInfrared( false );
		}

		if ( mDeviceOptions.isColorEnabled() ) {
			if ( success( mStreamColor.create( mDevice, openni::SENSOR_COLOR ) ) ) {
				openni::VideoMode mode;
				mode.setFps( mDeviceOptions.getColorFrameRate() );
				mode.setPixelFormat( mDeviceOptions.getColorPixelFormat() );
				mode.setResolution( mDeviceOptions.getColorSize().x, mDeviceOptions.getColorSize().y );
			}
		}

		if ( mDeviceOptions.isDepthEnabled() ) {
			if ( success( mStreamDepth.create( mDevice, openni::SENSOR_DEPTH ) ) ) {
				openni::VideoMode mode;
				mode.setFps( mDeviceOptions.getDepthFrameRate() );
				mode.setPixelFormat( mDeviceOptions.getDepthPixelFormat() );
				mode.setResolution( mDeviceOptions.getDepthSize().x, mDeviceOptions.getDepthSize().y );
			}
		}

		if ( mDeviceOptions.isHandTrackingEnabled() ) {
			success( mTrackerHand.create( &mDevice ) );
		}

		if ( mDeviceOptions.isInfraredEnabled() ) {
			if ( success( mStreamInfrared.create( mDevice, openni::SENSOR_IR ) ) ) {
				openni::VideoMode mode;
				mode.setFps( mDeviceOptions.getInfraredFrameRate() );
				mode.setPixelFormat( mDeviceOptions.getInfraredPixelFormat() );
				mode.setResolution( mDeviceOptions.getInfraredSize().x, mDeviceOptions.getInfraredSize().y );
			}
		}

		if ( mDeviceOptions.isUserTrackingEnabled() ) {
			success( mTrackerUser.create( &mDevice ) );
		}

		mConnected = true;
	}
}

Device::~Device()
{
	if ( mListenerColor != 0 ) {
		mStreamColor.removeNewFrameListener( mListenerColor );
		delete mListenerColor;
	}
	if ( mListenerDepth != 0 ) {
		mStreamDepth.removeNewFrameListener( mListenerDepth );
		delete mListenerDepth;
	}
	if ( mListenerHand != 0 ) {
		mTrackerHand.removeNewFrameListener( mListenerHand );
		delete mListenerHand;
	}
	if ( mListenerInfrared != 0 ) {
		mStreamInfrared.removeNewFrameListener( mListenerInfrared );
		delete mListenerInfrared;
	}
	if ( mListenerUser != 0 ) {
		mTrackerUser.removeNewFrameListener( mListenerUser );
		delete mListenerUser;
	}
		
	stop();

	if ( mStreamColor.isValid() ) {
		mStreamColor.destroy();
	}
	if ( mStreamDepth.isValid() ) {
		mStreamDepth.destroy();
	}
	if ( mTrackerHand.isValid() ) {
		mTrackerHand.destroy();
	}
	if ( mStreamInfrared.isValid() ) {
		mStreamInfrared.destroy();
	}
	if ( mTrackerUser.isValid() ) {
		mTrackerUser.destroy();
	}

	mDevice.close();
}

void Device::start()
{
	if ( mDevice.isValid() ) {
		if ( mDeviceOptions.isColorEnabled() && mStreamColor.isValid() ) {
			success( mStreamColor.start() );
		}
		if ( mDeviceOptions.isDepthEnabled() && mStreamDepth.isValid() ) {
			success( mStreamDepth.start() );
		}
		if ( mDeviceOptions.isInfraredEnabled() && mStreamInfrared.isValid() ) {
			success( mStreamInfrared.start() );
		}
	}
}

void Device::stop()
{
	if ( mStreamColor.isValid() ) {
		mStreamColor.stop();
	}
	if ( mStreamDepth.isValid() ) {
		mStreamDepth.stop();
	}
	if ( mStreamInfrared.isValid() ) {
		mStreamInfrared.stop();
	}
}
	
const openni::Device& Device::getDevice() const
{
	return mDevice;
}

const openni::DeviceInfo& Device::getDeviceInfo() const
{
	return mDeviceInfo;
}

const DeviceOptions& Device::getDeviceOptions() const
{
	return mDeviceOptions;
}

openni::DeviceState Device::getDeviceState() const
{
	return mDeviceState;
}

openni::VideoStream& Device::getColorStream()
{
	return mStreamColor;
}

const openni::VideoStream& Device::getColorStream() const
{
	return mStreamColor;
}

openni::VideoStream& Device::getDepthStream()
{
	return mStreamDepth;
}

const openni::VideoStream& Device::getDepthStream() const
{
	return mStreamDepth;
}

nite::HandTracker& Device::getHandTracker()
{
	return mTrackerHand;
}

const nite::HandTracker& Device::getHandTracker() const
{
	return mTrackerHand;
}

openni::VideoStream& Device::getInfraredStream()
{
	return mStreamInfrared;
}

const openni::VideoStream& Device::getInfraredStream() const
{
	return mStreamInfrared;
}

nite::UserTracker& Device::getUserTracker()
{
	return mTrackerUser;
}

const nite::UserTracker& Device::getUserTracker() const
{
	return mTrackerUser;
}

bool Device::isConnected() const
{
	return mConnected;
}

void Device::connectColorEventHandler( const VideoStreamListener::EventHandler& eventHandler )
{
	if ( mDeviceOptions.isColorEnabled() && mStreamColor.isValid() ) {
		mListenerColor = new VideoStreamListener( eventHandler, mDeviceOptions );
		mStreamColor.addNewFrameListener( mListenerColor );
	}
}

void Device::connectDepthEventHandler( const VideoStreamListener::EventHandler& eventHandler )
{
	if ( mDeviceOptions.isDepthEnabled() && mStreamDepth.isValid() ) {
		mListenerDepth = new VideoStreamListener( eventHandler, mDeviceOptions );
		mStreamDepth.addNewFrameListener( mListenerDepth );
	}
}

void Device::connectHandEventHandler( const HandTrackerListener::EventHandler& eventHandler )
{
	if ( mDeviceOptions.isHandTrackingEnabled() && mTrackerHand.isValid() ) {
		mListenerHand = new HandTrackerListener( eventHandler, mDeviceOptions );
		mTrackerHand.addNewFrameListener( mListenerHand );
	}
}

void Device::connectInfraredEventHandler( const VideoStreamListener::EventHandler& eventHandler )
{
	if ( mDeviceOptions.isInfraredEnabled() && mStreamInfrared.isValid() ) {
		mListenerInfrared = new VideoStreamListener( eventHandler, mDeviceOptions );
		mStreamInfrared.addNewFrameListener( mListenerInfrared );
	}
}

void Device::connectUserEventHandler( const UserTrackerListener::EventHandler& eventHandler )
{
	if ( mDeviceOptions.isUserTrackingEnabled() && mTrackerUser.isValid() ) {
		mListenerUser = new UserTrackerListener( eventHandler, mDeviceOptions );
		mTrackerUser.addNewFrameListener( mListenerUser );
	}
}

void Device::update()
{
	if ( mListenerColor != 0 ) {
		mListenerColor->update();
	}
	if ( mListenerDepth != 0 ) {
		mListenerDepth->update();
	}
	if ( mListenerHand != 0 ) {
		mListenerHand->update();
	}
	if ( mListenerInfrared != 0 ) {
		mListenerInfrared->update();
	}
	if ( mListenerUser != 0 ) {
		mListenerUser->update();
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////
	
DeviceManagerRef DeviceManager::create()
{
	return DeviceManagerRef( new DeviceManager() );
}

DeviceManager::DeviceManager()
	: mDeviceCount( 0 ), mInitialized( false )
{
	initialize();

	app::App::get()->getSignalUpdate().connect( bind( &DeviceManager::update, this ) );
}

DeviceManager::~DeviceManager()
{
	mDevices.clear();
	if ( mInitialized ) {
		openni::OpenNI::removeDeviceConnectedListener( this );
		openni::OpenNI::removeDeviceDisconnectedListener( this );
		openni::OpenNI::removeDeviceStateChangedListener( this );

		openni::OpenNI::shutdown();
		nite::NiTE::shutdown();
			
		mInitialized = false;
	}
}

void DeviceManager::initialize()
{
	if ( !success( openni::OpenNI::initialize() ) ) {
		return;
	}
	if ( !success( nite::NiTE::initialize() ) ) {
		return;
	}
	if ( !success( openni::OpenNI::addDeviceConnectedListener( this ) ) ) {
		return;
	}
	if ( !success( openni::OpenNI::addDeviceDisconnectedListener( this ) ) ) {
		return;
	}
	if ( !success( openni::OpenNI::addDeviceStateChangedListener( this ) ) ) {
		return;
	}
	enumerateDevices();
	mInitialized = true;
}

bool DeviceManager::isInitialized() const
{
	return mInitialized;
}

DeviceRef DeviceManager::createDevice( const DeviceOptions& deviceOptions )
{
	DeviceOptions options = deviceOptions;
	string uri = options.getUri();
	if ( !uri.empty() ) {
		try {
			DeviceRef device = getDevice( uri );
			return device;
		} catch ( ExcDeviceNotFound ex ) {
		}

		int32_t count = mDeviceInfoArray.getSize();
		for ( int32_t i = 0; i < count; ++i ) {
			if ( mDeviceInfoArray[ i ].getUri() == uri ) {
				mDevices.push_back( new Device( deviceOptions ) );
				return &mDevices[ mDevices.size() - 1 ];
			}
		}

		throw ExcDeviceNotFound( uri );
	}

	int32_t count = mDeviceInfoArray.getSize();
	for ( int32_t i = 0; i < count; ++i ) {
		uri = mDeviceInfoArray[ i ].getUri();
		try {
			DeviceRef device = getDevice( uri );
		} catch ( ExcDeviceNotFound ex ) {
			options.setUri( uri );
			mDevices.push_back( new Device( options ) );
			return &mDevices[ mDevices.size() - 1 ];
		}
	}

	throw ExcDeviceNotAvailable();
}

size_t DeviceManager::getDeviceCount() const
{
	return mDeviceCount;
}

const std::vector<openni::DeviceInfo>& DeviceManager::getDeviceInfoList() const
{
	return mDeviceInfoList;
}

void DeviceManager::update()
{
	for ( DeviceList::iterator iter = mDevices.begin(); iter != mDevices.end(); ++iter ) {
		iter->update();
	}
}

void DeviceManager::enumerateDevices()
{
	openni::OpenNI::enumerateDevices( &mDeviceInfoArray );
	mDeviceInfoList = toVector( mDeviceInfoArray );
	mDeviceCount	= mDeviceInfoList.size();
}

DeviceRef DeviceManager::getDevice( const std::string& uri )
{
	size_t i = 0;
	for ( DeviceList::iterator iter = mDevices.begin(); iter != mDevices.end(); ++iter, ++i ) {
		if ( iter->getDeviceOptions().getUri() == uri ) {
			return &mDevices[ i ];
		}
	}
	throw ExcDeviceNotFound( uri );	}

void DeviceManager::onDeviceStateChanged( const openni::DeviceInfo* info, openni::DeviceState state ) 
{
	try {
		console() << "State changed" << endl;
		DeviceRef device		= getDevice( info->getUri() );
		device->mDeviceState	= state;
	} catch ( ExcDeviceNotFound ex ) {
	}
}

void DeviceManager::onDeviceConnected( const openni::DeviceInfo* info )
{
	try {
		console() << "Connected" << endl;
		DeviceRef device	= getDevice( info->getUri() );
		device->mConnected	= true;
	} catch ( ExcDeviceNotFound ex ) {
	}
	enumerateDevices();
}

void DeviceManager::onDeviceDisconnected( const openni::DeviceInfo* info )
{
	try {
		console() << "Disconnected" << endl;
		DeviceRef device	= getDevice( info->getUri() );
		device->mConnected	= false;
		device->stop();
	} catch ( ExcDeviceNotFound ex ) {
	}
	enumerateDevices();
}

//////////////////////////////////////////////////////////////////////////////////////////////
	
ExcDeviceNotAvailable::ExcDeviceNotAvailable() throw()
{
	sprintf( mMessage, "No free devices available" );
}

ExcDeviceNotFound::ExcDeviceNotFound( const string &uri ) throw()
{
	sprintf( mMessage, "Could not find device: %s", uri.c_str() );
}
}
