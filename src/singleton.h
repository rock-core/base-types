#ifndef _BASE_SINGLETON_H_
#define _BASE_SINGLETON_H_

namespace base {

template<class Derived>
class Singleton
{

private:
	static Derived* msInstance;

protected:
	Singleton() {}

public:
	static Derived* getInstance()
	{
		static CGuard g;

		if(msInstance == 0)
		{
			msInstance = new Derived(); 
		}
	
		return msInstance;
	}

	virtual ~Singleton()
	{
	}


	// Nested singleton helper class
	class CGuard
	{
		public: 
			~CGuard()
			{
				if(msInstance != 0)
				{
					delete msInstance;
					msInstance = 0;
				}
			}

	};

	friend class CGuard;

};

template<typename Derived> Derived* Singleton<Derived>::msInstance = 0;

} // end namespace base;


#endif // _BASE_SINGLETON_H_
