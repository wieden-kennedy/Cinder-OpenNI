#include "Emitter.h"

Emitter::Emitter( const ci::Vec3f& position )
{
	mPosition	= position;
}

Emitter::~Emitter()
{
}

void Emitter::update( const ci::Vec3f& position )
{
	mPosition	= position;
}

const ci::Vec3f& Emitter::getPosition() const
{
	return mPosition;
}