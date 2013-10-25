/*
 * TimeStamped.hpp
 *
 *  Created on: 24.10.2013
 *      Author: Steffen planthaber
 *
 * This template be used to add time stamps to e.g. commands or other types without a "Base::Time time"
 *
 */

#include "../Time.hpp"

#ifndef TIMESTAMPED_HPP_
#define TIMESTAMPED_HPP_

namespace base {

	template <class C> class TimeStamped: public C{
	public:

		TimeStamped():C(){};

		void updateTime(){
			time = Time::now();
		}

		Time time;
	};

}//namespace base

#endif /* TIMESTAMP_HPP_ */
