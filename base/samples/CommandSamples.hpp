/*
 * CommandSamples.hpp
 *
 *  Created on: 24.10.2013
 *      Author: Steffen Planthaber
 *
 *  A collection of base/commands with timestamps, for use as command feedback and logging
 *  (which command was executed when) or for supervising tasks.
 *
 */

#ifndef COMMANDSAMPLES_HPP_
#define COMMANDSAMPLES_HPP_

#include "../templates/TimeStamped.hpp"
#include "../commands/Motion2D.hpp"

namespace base{
	namespace samples {

	/**
	 * Time stamped version of a Motion2D command
	 */
	class Motion2D : public TimeStamped< commands::Motion2D >{};


	}//namespace templates
}//namespace base

#endif /* TIMESTAMPEDDATA_HPP_ */
