#include "Particle.h"
#include "cinder/Rand.h"

const float Particle::kMaxSpeed		= 10.0f;
const float Particle::kLifeSpan		= 5.0f;		// in seconds

Particle::Particle()
{}

Particle::Particle( const ci::Vec3f& position ) : 
	mPosition( position ),
	mVelocity( ci::Vec3f::zero() ),
	mIsDead( false )
{
	// random acceleration
	mAcceleration	= ci::randVec3f();
	mAcceleration	*= ci::randFloat( kMaxSpeed );

	// random age
	mAge	= ci::randFloat( kLifeSpan * 0.75f );

	// random drag
	mDrag	= ci::randFloat( 0.5f, 0.9f );

	// random color
	mColor	= ci::Color( ci::randFloat(), ci::randFloat(), ci::randFloat() );
}

Particle::~Particle()
{
}

void Particle::update( double elapsedSeconds )
{
	mAge	+= elapsedSeconds;

	if ( mAge >= kLifeSpan )
	{
		mIsDead		= true;

		return;
	}

	mAcceleration	*= mDrag;
	mVelocity		+= ( mVelocity.length() < kMaxSpeed ) ? mAcceleration : ci::Vec3f::zero();
	mPosition		+= mVelocity;
}