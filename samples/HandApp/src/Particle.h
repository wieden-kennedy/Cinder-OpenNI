#pragma once

#include "cinder/Vector.h"

class Particle;
typedef std::shared_ptr< Particle >		ParticleRef;

class Particle
{
public:
	static ParticleRef		create( const ci::Vec3f& position )
	{
		return ParticleRef( new Particle( position ) );
	}

	~Particle();

	bool					isDead() const { return mIsDead; }
	void					update( double elapsedSeconds );

protected:
	Particle();
	Particle( const ci::Vec3f& position );

	ci::Vec3f	mAcceleration;
	ci::Vec3f	mVelocity;
	ci::Vec3f	mPosition;
	float		mDrag;
	float		mAge;
	bool		mIsDead;

	static const float		kMaxSpeed;
	static const float		kLifeSpan;
};