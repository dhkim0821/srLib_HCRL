
#pragma once

#ifndef NLOG

#include <iostream>
#include <cassert>
#include <ctime>
//#include <omp.h>

#define LOGIF(statement,message)\
	do{\
		if(!(statement)){std::cout<<(message)<<std::endl;}\
		assert((statement)&&(message));\
	}while(false);

#define LOG(message)\
	std::cout<<(message)<<std::endl;

#define LOG_VALUE(message1,message2)\
	std::cout<<(message1)<<(message2)<<std::endl;

#else

#define LOGIF(statement,message) ;
#define LOG(message) ;

#endif

//#define PERFORM_TEST(loop,iteration)\
//	do{\
//		std::cout<<"Performance Test Start"<<std::endl;\
//		double begin  = omp_get_wtime();\
//		for(int i = 0 ; i < iteration ; i++){loop}\
//		double end = omp_get_wtime();\
//		std::cout<<"Performance Test End"<<std::endl;\
//		std::cout<<"Computation Time:"<<(end-begin)*1000<<"\tms"<<std::endl;\
//	}while(false);

#define PERFORM_TEST(loop,iteration)\
	do{\
		clock_t t1 = clock();\
		for(int i = 0 ; i < iteration ; i++){loop;}\
		clock_t t2 = clock();\
		std::cout<<"Computation Time:"<<(t2-t1)/(double)CLOCKS_PER_SEC*1000.0<<"\tms"<<std::endl;\
	}while(false);
