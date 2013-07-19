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

#include "Cinder-OpenNI.h"

#include "cinder/app/App.h"

namespace OpenNI
{
	using namespace ci;
	using namespace ci::app;
	using namespace std;

	//////////////////////////////////////////////////////////////////////////////////////////////

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

	template<typename T>
	vector<T> toVector( const nite::Array<T>& a )
	{
		vector<T> v;
		v.insert( v.end(), &a[ 0 ], &a[ a.getSize() / sizeof( T ) ]);
		return v;
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
		enableInfrared( false );
		enableUserTracking( false );

		setColorSize( Vec2i( 320, 240 ) );
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

	float DeviceOptions::getColorFrameRate() const
	{
		return mFrameRateColor;
	}

	float DeviceOptions::getDepthFrameRate() const
	{
		return mFrameRateDepth;
	}
 
	float DeviceOptions::getInfraredFrameRate() const
	{
		return mFrameRateInfrared;
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

	HandTrackerListener::HandTrackerListener( HandTrackerListener::EventHandler eventHandler )
		: nite::HandTracker::NewFrameListener(), mEventHandler( eventHandler )
	{
	}
	
	UserTrackerListener::UserTrackerListener( UserTrackerListener::EventHandler eventHandler )
		: nite::UserTracker::NewFrameListener(), mEventHandler( eventHandler )
	{
	}

	VideoStreamListener::VideoStreamListener( VideoStreamListener::EventHandler eventHandler )
		: openni::VideoStream::NewFrameListener(), mEventHandler( eventHandler )
	{
	}

	void HandTrackerListener::onNewFrame( nite::HandTracker& tracker )
	{
		tracker.readFrame( &mFrame );
		mEventHandler( mFrame );
	}
	
	void UserTrackerListener::onNewFrame( nite::UserTracker& tracker )
	{
		tracker.readFrame( &mFrame );
		mEventHandler( mFrame );
	}

	void VideoStreamListener::onNewFrame( openni::VideoStream& stream ) 
	{
		stream.readFrame( &mFrame );
		mEventHandler( mFrame );
	}

	//////////////////////////////////////////////////////////////////////////////////////////////

	Device::Device( const DeviceOptions& deviceOptions )
	{
		mConnected			= false;
		mDeviceOptions		= deviceOptions;
		mDeviceState		= openni::DeviceState::DEVICE_STATE_NOT_READY;

		mListenerColor		= 0;
		mListenerDepth		= 0;
		mListenerHand		= 0;
		mListenerInfrared	= 0;
		mListenerUser		= 0;
	}

	Device::~Device()
	{
		if ( mListenerColor != 0 ) {
			if ( mStreamColor.isValid() ) {
				mStreamColor.removeNewFrameListener( mListenerColor );
			}
			delete mListenerColor;
		}
		if ( mListenerDepth != 0 ) {
			if ( mStreamDepth.isValid() ) {
				mStreamDepth.removeNewFrameListener( mListenerDepth );
			}
			delete mListenerDepth;
		}
		if ( mListenerHand != 0 ) {
			if ( mTrackerHand.isValid() ) {
				mTrackerHand.removeNewFrameListener( mListenerHand );
			}
			delete mListenerHand;
		}
		if ( mListenerInfrared != 0 ) {
			if ( mStreamInfrared.isValid() ) {
				mStreamInfrared.removeNewFrameListener( mListenerInfrared );
			}
			delete mListenerInfrared;
		}
		if ( mListenerUser != 0 ) {
			if ( mTrackerUser.isValid() ) {
				mTrackerUser.removeNewFrameListener( mListenerUser );
			}
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
		mDevice.open( mDeviceOptions.getUri().c_str() );

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
			if ( mStreamColor.create( mDevice, openni::SENSOR_COLOR ) != openni::STATUS_OK ) {
				openni::VideoMode mode;
				mode.setFps( mDeviceOptions.getColorFrameRate() );
				mode.setPixelFormat( openni::PixelFormat::PIXEL_FORMAT_RGB888 );
				mode.setResolution( mDeviceOptions.getColorSize().x, mDeviceOptions.getColorSize().y );
				mStreamColor.setVideoMode( mode );
				mStreamColor.start();
				mDeviceOptions.enableColor( false );
			}
		}

		if ( mDeviceOptions.isDepthEnabled() ) {
			if ( mStreamDepth.create( mDevice, openni::SENSOR_DEPTH ) != openni::STATUS_OK ) {
				openni::VideoMode mode;
				mode.setFps( mDeviceOptions.getDepthFrameRate() );
				mode.setPixelFormat( openni::PixelFormat::PIXEL_FORMAT_DEPTH_1_MM );
				mode.setResolution( mDeviceOptions.getDepthSize().x, mDeviceOptions.getDepthSize().y );
				mStreamDepth.setVideoMode( mode );
				mStreamDepth.start();
				mDeviceOptions.enableDepth( false );
			}
		}

		if ( mDeviceOptions.isHandTrackingEnabled() ) {
			if ( mTrackerHand.create( &mDevice ) != nite::STATUS_OK ) {
				mDeviceOptions.enableHandTracking( false );
			}
		}

		if ( mDeviceOptions.isInfraredEnabled() ) {
			if ( mStreamInfrared.create( mDevice, openni::SENSOR_IR ) != openni::STATUS_OK ) {
				openni::VideoMode mode;
				mode.setFps( mDeviceOptions.getInfraredFrameRate() );
				mode.setPixelFormat( openni::PixelFormat::PIXEL_FORMAT_GRAY8 );
				mode.setResolution( mDeviceOptions.getInfraredSize().x, mDeviceOptions.getInfraredSize().y );
				mStreamInfrared.setVideoMode( mode );
				mStreamInfrared.start();
				mDeviceOptions.enableInfrared( false );
			}
		}

		if ( mDeviceOptions.isUserTrackingEnabled() ) {
			if ( mTrackerUser.create( &mDevice ) != nite::STATUS_OK ) {
				mDeviceOptions.enableUserTracking( false );
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

	//////////////////////////////////////////////////////////////////////////////////////////////
	
	DeviceManager::DeviceManager()
		: mInitialized( false )
	{
	}

	DeviceManager::~DeviceManager()
	{
		stop();
		mDevices.clear();
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
			try {
				DeviceRef device = getDevice( uri );
			} catch ( ExcDeviceNotFound ex ) {
				uri = mDeviceInfoArray[ i ].getUri();
				options.setUri( uri );
				mDevices.push_back( new Device( options ) );
				return &mDevices[ mDevices.size() - 1 ];
			}
		}

		throw ExcDeviceNotAvailable();
	}

	void DeviceManager::start()
	{
		console() << "test" << endl;
		if ( openni::OpenNI::initialize() != openni::STATUS_OK ) {
			console() << "Unable to initialize OpenNI: " << endl << openni::OpenNI::getExtendedError() << endl;
			return;
		}
		if ( nite::NiTE::initialize() != nite::STATUS_OK ) {
			console() << "Unable to initialize NiTE: " << endl << openni::OpenNI::getExtendedError() << endl;
			return;
		}
		if ( openni::OpenNI::addDeviceConnectedListener( this ) != openni::STATUS_OK ) {
			console() << "Unable to add device connected listener: " << endl << openni::OpenNI::getExtendedError() << endl;
			return;
		}
		if ( openni::OpenNI::addDeviceDisconnectedListener( this ) != openni::STATUS_OK ) {
			console() << "Unable to add device disconnected listener: " << endl << openni::OpenNI::getExtendedError() << endl;
			return;
		}
		if ( openni::OpenNI::addDeviceStateChangedListener( this ) != openni::STATUS_OK ) {
			console() << "Unable to add device state changed listener: " << endl << openni::OpenNI::getExtendedError() << endl;
			return;
		}
		openni::OpenNI::enumerateDevices( &mDeviceInfoArray );
		mInitialized = true;
	}

	void DeviceManager::stop()
	{
		if ( mInitialized ) {
			openni::OpenNI::removeDeviceConnectedListener( this );
			openni::OpenNI::removeDeviceDisconnectedListener( this );
			openni::OpenNI::removeDeviceStateChangedListener( this );

			try {
				openni::OpenNI::shutdown();
			} catch ( ... ) {
			}

			try {
				nite::NiTE::shutdown();
			} catch ( ... ) {
			}

			mInitialized = false;
		}

		openni::OpenNI::shutdown();
	}

	const openni::Array<openni::DeviceInfo>& DeviceManager::getDeviceInfoArray() const
	{
		return mDeviceInfoArray;
	}

	bool DeviceManager::isInitialized() const
	{
		return mInitialized;
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
			DeviceRef device		= getDevice( info->getUri() );
			device->mDeviceState	= state;
		} catch ( ExcDeviceNotFound ex ) {
		}
	}

	void DeviceManager::onDeviceConnected( const openni::DeviceInfo* info )
	{
		try {
			DeviceRef device	= getDevice( info->getUri() );
			device->mConnected	= true;
		} catch ( ExcDeviceNotFound ex ) {
		}

		openni::OpenNI::enumerateDevices( &mDeviceInfoArray );
	}

	void DeviceManager::onDeviceDisconnected( const openni::DeviceInfo* info )
	{
		try {
			DeviceRef device	= getDevice( info->getUri() );
			device->mConnected	= false;
			device->stop();
		} catch ( ExcDeviceNotFound ex ) {
		}

		openni::OpenNI::enumerateDevices( &mDeviceInfoArray );
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
