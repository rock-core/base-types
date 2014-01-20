/*
 * TimeStamped.hpp
 *
 *  Created on: 24.10.2013
 *      Author: Steffen Planthaber
 *
 * This template be used to add time stamps to e.g. commands or other types without a "Base::Time time"
 *
 */

#include "../Time.hpp"

#ifndef TIMESTAMPED_HPP_
#define TIMESTAMPED_HPP_

namespace base {

	template <class BASE> class TimeStamped: public BASE{
	public:

		TimeStamped():BASE(){};

		/**
		 * get the base class (without timestamp)
		 * @return the Base class, which got timestamped by this template
		 */
		inline BASE& getBase(){
			return *((BASE*)this);
		}

		/**
		 * sets a new value for the timestamped class using it's base class
		 * @param from the value to be timestamped
		 */
		inline void set(const BASE &from){
			getBase() = from;
			this->time = Time::now();
		}

		/**
		 * sets a new value for the timestamped class using it's base class
		 * @param from the value to be timestamped
		 */
		inline void set(const BASE &from, Time &timestamp){
			getBase() = from;
			this->time = timestamp;
		}

		/**
		 * updates the time
		 */
		void updateTime(){
			time = Time::now();
		}

		/**
		 * the actual timestamp
		 */
		Time time;
	};

}//namespace base

#endif /* TIMESTAMP_HPP_ */
