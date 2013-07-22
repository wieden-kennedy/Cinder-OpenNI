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
#include "cinder/Camera.h"
#include "Cinder-OpenNI.h"

/* 
 * This application demonstrates how to display NiTE users.
 */
class UserApp : public ci::app::AppBasic 
{

public:
	void						draw();
	void						keyDown( ci::app::KeyEvent event );
	void						prepareSettings( ci::app::AppBasic::Settings* settings );
	void						setup();
	void						update();
private:
	struct Bone
	{
		Bone( nite::JointType a, nite::JointType b )
			: mJointA( a ), mJointB( b )
		{
		}
		nite::JointType			mJointA;
		nite::JointType			mJointB;
	};
	std::vector<Bone>			mBones;

	ci::CameraPersp				mCamera;

	OpenNI::DeviceManagerRef	mDeviceManager;
	OpenNI::DeviceRef			mDevice;
	std::vector<nite::UserData>	mUsers;
	void						onUser( nite::UserTrackerFrameRef );

	void						screenShot();
};

#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"

using namespace ci;
using namespace ci::app;
using namespace std;

void UserApp::draw()
{
	gl::setViewport( getWindowBounds() );
	gl::clear( Colorf::black() );
	gl::setMatrices( mCamera );

	gl::color( Colorf( 1.0f, 0.0f, 0.0f ) );
	for ( std::vector<nite::UserData>::const_iterator iter = mUsers.begin(); iter != mUsers.end(); ++iter ) {
		const nite::Skeleton& skeleton = iter->getSkeleton();
		if ( skeleton.getState() == nite::SKELETON_TRACKED ) {
			gl::begin( GL_LINES );
			for ( vector<Bone>::const_iterator iter = mBones.begin(); iter != mBones.end(); ++iter ) {
				const nite::SkeletonJoint& joint0 = skeleton.getJoint( iter->mJointA );
				const nite::SkeletonJoint& joint1 = skeleton.getJoint( iter->mJointB );

				Vec3f v0 = OpenNI::toVec3f( joint0.getPosition() );
				Vec3f v1 = OpenNI::toVec3f( joint1.getPosition() );
				v0.x = -v0.x;
				v1.x = -v1.x;

				gl::vertex( v0 );
				gl::vertex( v1 );
			}
			gl::end();
		}
	}
}

void UserApp::keyDown( KeyEvent event )
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

void UserApp::onUser( nite::UserTrackerFrameRef frame )
{
	mUsers = OpenNI::toVector( frame.getUsers() );

	for ( std::vector<nite::UserData>::iterator iter = mUsers.begin(); iter != mUsers.end(); ++iter ) {
		if ( iter->isNew() ) {
			mDevice->getUserTracker().startSkeletonTracking( iter->getId() );
		} else if ( iter->isLost() ) {
			mDevice->getUserTracker().stopSkeletonTracking( iter->getId() );
		}
	}
}

void UserApp::prepareSettings( Settings* settings )
{
	settings->setFrameRate( 60.0f );
	settings->setWindowSize( 800, 600 );
}

void UserApp::screenShot()
{
	writeImage( getAppPath() / fs::path( "frame" + toString( getElapsedFrames() ) + ".png" ), copyWindowSurface() );
}

void UserApp::setup()
{
	mBones.push_back( Bone( nite::JOINT_HEAD,			nite::JOINT_NECK ) );
	mBones.push_back( Bone( nite::JOINT_LEFT_SHOULDER,	nite::JOINT_LEFT_ELBOW ) );
	mBones.push_back( Bone( nite::JOINT_LEFT_ELBOW,		nite::JOINT_LEFT_HAND ) );
	mBones.push_back( Bone( nite::JOINT_RIGHT_SHOULDER, nite::JOINT_RIGHT_ELBOW ) );
	mBones.push_back( Bone( nite::JOINT_RIGHT_ELBOW,	nite::JOINT_RIGHT_HAND ) );
	mBones.push_back( Bone( nite::JOINT_LEFT_SHOULDER,	nite::JOINT_RIGHT_SHOULDER ) );
	mBones.push_back( Bone( nite::JOINT_LEFT_SHOULDER,	nite::JOINT_TORSO ) );
	mBones.push_back( Bone( nite::JOINT_RIGHT_SHOULDER, nite::JOINT_TORSO ) );
	mBones.push_back( Bone( nite::JOINT_TORSO,			nite::JOINT_LEFT_HIP ) );
	mBones.push_back( Bone( nite::JOINT_TORSO,			nite::JOINT_RIGHT_HIP ) );
	mBones.push_back( Bone( nite::JOINT_LEFT_HIP,		nite::JOINT_RIGHT_HIP ) );
	mBones.push_back( Bone( nite::JOINT_LEFT_HIP,		nite::JOINT_LEFT_KNEE ) );
	mBones.push_back( Bone( nite::JOINT_LEFT_KNEE,		nite::JOINT_LEFT_FOOT ) );
	mBones.push_back( Bone( nite::JOINT_RIGHT_HIP,		nite::JOINT_RIGHT_KNEE ) );
	mBones.push_back( Bone( nite::JOINT_RIGHT_KNEE,		nite::JOINT_RIGHT_FOOT ) );

	mCamera = CameraPersp( getWindowWidth(), getWindowHeight(), 45.0f, 1.0f, 5000.0f );
	mCamera.lookAt( Vec3f::zero(), Vec3f::zAxis(), Vec3f::yAxis() );

	mDeviceManager = OpenNI::DeviceManager::create();
	try {
		mDevice = mDeviceManager->createDevice( OpenNI::DeviceOptions().enableUserTracking() );
	} catch ( OpenNI::ExcDeviceNotAvailable ex ) {
		console() << ex.what() << endl;
		quit();
		return;
	}

	mDevice->getUserTracker().setSkeletonSmoothingFactor( 0.5f );

	mDevice->connectUserEventHandler( &UserApp::onUser, this );
	mDevice->start();
}

void UserApp::update()
{
	mDeviceManager->update();
}

CINDER_APP_BASIC( UserApp, RendererGl )
