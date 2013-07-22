#pragma once

#include "cinder/Vector.h"

class Emitter;
typedef std::shared_ptr< Emitter >	EmitterRef;

class Emitter
{
public:
	static EmitterRef		create( const ci::Vec3f& position )
	{
		return EmitterRef( new Emitter( position ) );
	}

	~Emitter();
	
	void					update( const ci::Vec3f& position );
	const ci::Vec3f&		getPosition() const;

protected:
	Emitter(){}
	Emitter( const ci::Vec3f& position );

	ci::Vec3f		mPosition;
};