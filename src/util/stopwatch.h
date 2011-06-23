#ifndef _BASIC_OS_STOPWATCH_H
#define _BASIC_OS_STOPWATCH_H

//_________________________________________________________

#ifdef WIN32
#include <windows.h>
#else
#include <sys/types.h>
#include <sys/times.h> 
#endif

//______________________________________________________________________
class SystemStopwatch {
public :
	SystemStopwatch() ;

	/**
	* prints real, user and system times since the
	* construction of this SystemStopWatch (in seconds).
	*/
	void print_elapsed_time() ;

	/**
	* returns user elapsed time since the construction
	* of this SystemStopWatch (in seconds).
	*/
	double elapsed_user_time() const ;


	/**
	* returns current time (in seconds).
	*/
	static double now() ;

private:
#ifdef WIN32
	long start_ ;
#else
	tms start_ ;
	clock_t start_user_ ;
#endif    
} ;

#endif


