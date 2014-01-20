/*
 * CommandSamples.hpp
 *
 *  Created on: 24.10.2013
 *      Author: Steffen Planthaber
 *
 *  A collection of base/commands with timestamps, for use as command feedback and logging
 *  (which command was executed when) or for supervising tasks.
 *
 * all definitions may be empty : "class Motion2D : public TimeStamped< commands::Motion2D >{};",
 * when only set() and getBase() of the TimeStamped template are used in tasks
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
	class Motion2D : public TimeStamped< commands::Motion2D >{
	public:
		/**
		 * default constructor (not needed when no other constructor is present)
		 */
		Motion2D ():TimeStamped< commands::Motion2D >(){}
		/**
		 * constructor using the base class (allows to write the command directly to the sample port)
		 */
		Motion2D (const commands::Motion2D &from):TimeStamped< commands::Motion2D >(){
			this->set(from);
		}
	};

	}//namespace samples
}//namespace base



#endif /* COMMANDSAMPLES_HPP_ */
