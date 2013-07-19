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

#pragma once

#include "boost/ptr_container/ptr_vector.hpp"
#include "cinder/AxisAlignedBox.h"
#include "cinder/Channel.h"
#include "cinder/Exception.h"
#include "cinder/Plane.h"
#include "cinder/Quaternion.h"
#include "cinder/Surface.h"
#include "cinder/Vector.h"
#include "cinder/Utilities.h"
#include "OpenNI.h"
#include "Nite.h"

namespace OpenNI
{
	ci::AxisAlignedBox3f	toAxisAlignedBox3f( const nite::Point3f& aMin, const nite::Point3f& aMax );
	ci::Planef				toPlanef( const nite::Point3f& point, const nite::Point3f& normal );
	ci::Quatf				toQuatf( const nite::Quaternion& q );
	ci::Vec3f				toVec3f( const nite::Point3f& v );

	template<typename T>
	std::vector<T>			toVector( const nite::Array<T>& a );

	ci::Channel8u			toChannel8u( const openni::VideoFrameRef& f );
	ci::Channel16u			toChannel16u( const openni::VideoFrameRef& f );
	ci::Surface8u			toSurface8u( const openni::VideoFrameRef& f );
	ci::Surface16u			toSurface16u( const openni::VideoFrameRef& f );
	
	class DeviceOptions
	{
	public:
		DeviceOptions();
		
		bool				isColorEnabled() const;
		bool				isDepthEnabled() const;
		bool				isHandTrackingEnabled() const;
		bool				isInfraredEnabled() const; 
		bool				isUserTrackingEnabled() const;

		float				getColorFrameRate() const;
		float				getDepthFrameRate() const;
		float				getInfraredFrameRate() const;
		
		const ci::Vec2i&	getColorSize() const; 
		const ci::Vec2i&	getDepthSize() const; 
		const ci::Vec2i&	getInfraredSize() const;

		const std::string&	getUri() const;
		
		DeviceOptions&		enableColor( bool enable = true );
		DeviceOptions&		enableDepth( bool enable = true );
		DeviceOptions&		enableHandTracking( bool enable = true );
		DeviceOptions&		enableInfrared( bool enable = true );
		DeviceOptions&		enableUserTracking( bool enable = true );

		DeviceOptions&		setColorFrameRate( float frameRate ); 
		DeviceOptions&		setDepthFrameRate( float frameRate ); 
		DeviceOptions&		setInfraredFrameRate( float frameRate );

		DeviceOptions&		setColorSize( const ci::Vec2i& size );
		DeviceOptions&		setDepthSize( const ci::Vec2i& size );
		DeviceOptions&		setInfraredSize( const ci::Vec2i& size );

		void				setUri( const std::string& uri );
	private:
		bool				mEnabledColor;
		bool				mEnabledDepth;
		bool				mEnabledHandTracking;
		bool				mEnabledInfrared;
		bool				mEnabledUserTracking;

		float				mFrameRateColor;
		float				mFrameRateDepth;
		float				mFrameRateInfrared;

		ci::Vec2i			mSizeColor;
		ci::Vec2i			mSizeDepth;
		ci::Vec2i			mSizeInfrared;

		std::string			mUri;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////
	
	class Device;

	class HandTrackerListener : nite::HandTracker::NewFrameListener
	{
	public:
		void						onNewFrame( nite::HandTracker& tracker );
	private:
		typedef std::function<void ( nite::HandTrackerFrameRef )> EventHandler;
		
		HandTrackerListener( EventHandler eventHandler );

		EventHandler				mEventHandler;
		nite::HandTrackerFrameRef	mFrame;
		
		friend class				Device;
	};

	class UserTrackerListener : nite::UserTracker::NewFrameListener
	{
	public:
		void						onNewFrame( nite::UserTracker& tracker );
	private:
		typedef std::function<void ( nite::UserTrackerFrameRef )> EventHandler;
		
		UserTrackerListener( EventHandler eventHandler );

		EventHandler				mEventHandler;
		nite::UserTrackerFrameRef	mFrame;

		friend class				Device;
	};

	class VideoStreamListener : public openni::VideoStream::NewFrameListener
	{
	public:
		void					onNewFrame( openni::VideoStream& stream );
	private:
		typedef std::function<void ( openni::VideoFrameRef )> EventHandler;
		
		VideoStreamListener( EventHandler eventHandler );

		EventHandler			mEventHandler;
		openni::VideoFrameRef	mFrame;

		friend class			Device;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////
	
	class DeviceManager;

	typedef boost::ptr_vector<Device>	DeviceList;
	typedef DeviceList::pointer			DeviceRef;

	class Device
	{
	public:
		~Device();

		void						start();
		void						stop();

		const openni::Device&		getDevice() const;
		const openni::DeviceInfo&	getDeviceInfo() const;
		const DeviceOptions&		getDeviceOptions() const;
		openni::DeviceState			getDeviceState() const;

		openni::VideoStream&		getColorStream();
		const openni::VideoStream&	getColorStream() const;
		openni::VideoStream&		getDepthStream();
		const openni::VideoStream&	getDepthStream() const;
		nite::HandTracker&			getHandTracker();
		const nite::HandTracker&	getHandTracker() const;
		openni::VideoStream&		getInfraredStream();
		const openni::VideoStream&	getInfraredStream() const;
		nite::UserTracker&			getUserTracker();
		const nite::UserTracker&	getUserTracker() const;

		bool						isConnected() const;

		template<typename T, typename Y>
		inline void					connectColorEventHandler( T callback, Y* callbackObject )
		{
			if ( mDeviceOptions.isColorEnabled() ) {
				mListenerColor = new VideoStreamListener( std::bind( callback, callbackObject, std::placeholders::_1 ) );
				mStreamColor.addNewFrameListener( mListenerColor );
			}
		}

		template<typename T, typename Y>
		inline void					connectDepthEventHandler( T callback, Y* callbackObject )
		{
			if ( mDeviceOptions.isDepthEnabled() ) {
				mListenerDepth = new VideoStreamListener( std::bind( callback, callbackObject, std::placeholders::_1 ) );
				mStreamDepth.addNewFrameListener( mListenerDepth );
			}
		}

		template<typename T, typename Y>
		inline void					connectHandEventHandler( T callback, Y* callbackObject )
		{
			if ( mDeviceOptions.isHandTrackingEnabled() ) {
				mListenerHand = new HandTrackerListener( std::bind( callback, callbackObject, std::placeholders::_1 ) );
				mTrackerHand.addNewFrameListener( mListenerHand );
			}
		}

		template<typename T, typename Y>
		inline void					connectInfraredEventHandler( T callback, Y* callbackObject )
		{
			if ( mDeviceOptions.isInfraredEnabled() ) {
				mListenerInfrared = new VideoStreamListener( std::bind( callback, callbackObject, std::placeholders::_1 ) );
				mStreamInfrared.addNewFrameListener( mListenerInfrared );
			}
		}

		template<typename T, typename Y>
		inline void					connectUserEventHandler( T callback, Y* callbackObject )
		{
			if ( mDeviceOptions.isUserTrackingEnabled() ) {
				mListenerUser = new UserTrackerListener( std::bind( callback, callbackObject, std::placeholders::_1 ) );
				mTrackerUser.addNewFrameListener( mListenerUser );
			}
		}
	private:
		Device( const DeviceOptions& deviceOptions );

		openni::Device							mDevice;
		openni::DeviceInfo						mDeviceInfo;
		openni::DeviceState						mDeviceState;

		bool									mConnected;
		DeviceOptions							mDeviceOptions;

		openni::VideoStream						mStreamColor;
		openni::VideoStream						mStreamDepth;
		openni::VideoStream						mStreamInfrared;
		nite::HandTracker						mTrackerHand;
		nite::UserTracker						mTrackerUser;

		VideoStreamListener*					mListenerColor;
		VideoStreamListener*					mListenerDepth;
		HandTrackerListener*					mListenerHand;
		VideoStreamListener*					mListenerInfrared;
		UserTrackerListener*					mListenerUser;

		friend class							DeviceManager;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////

	class DeviceManager : public openni::OpenNI::DeviceConnectedListener, 
		public openni::OpenNI::DeviceDisconnectedListener, public openni::OpenNI::DeviceStateChangedListener
	{
	public:
		DeviceManager();
		~DeviceManager();

		DeviceRef									createDevice( const DeviceOptions& deviceOptions = DeviceOptions() );

		void										start();
		void										stop();

		const openni::Array<openni::DeviceInfo>&	getDeviceInfoArray() const;

		bool										isInitialized() const;
	private:
		DeviceRef									getDevice( const std::string& uri );
		DeviceList									mDevices;
		openni::Array<openni::DeviceInfo>			mDeviceInfoArray;
		
		bool										mInitialized;
		
		void										onDeviceConnected( const openni::DeviceInfo* info);
		void										onDeviceDisconnected( const openni::DeviceInfo* info );
		void										onDeviceStateChanged( const openni::DeviceInfo* info, openni::DeviceState state );
	};

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	class Exception : public ci::Exception 
	{
	};
	
	class ExcDeviceNotAvailable : public OpenNI::Exception 
	{
	  public:
		ExcDeviceNotAvailable() throw();
		virtual const char* what() const throw() 
		{ 
			return mMessage; 
		}

	  private:
		char mMessage[ 2048 ];
	};

	class ExcDeviceNotFound : public OpenNI::Exception 
	{
	  public:
		ExcDeviceNotFound( const std::string &uri ) throw();
		virtual const char* what() const throw() 
		{ 
			return mMessage; 
		}

	  private:
		char mMessage[ 2048 ];
	};

}
