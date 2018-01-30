#pragma once

#include <ctime>
//#include <omp.h>
#include <srDyn/srSpace.h>

#define SR_SAFE_DELETE(p)			if(p) { delete (p); (p) = NULL; }
#define SR_SAFE_DESTROY_WINDOW(p) if(p) { p->DestroyWindow(); delete (p); (p) = NULL; }
#define SR_SAFE_DELETE_AR(p)		if(p) { delete [] p; (p) = NULL; }
#define SR_SAFE_RELEASE(p)		if(p) { (p)->Release(); (p) = NULL; }

//#define PERFORM_TEST(loop,iteration)\
//	do{\
//		clock_t begin  = clock();\
//		for(int i = 0 ; i < iteration ; i++){loop;}\
//		clock_t end = clock();\
//		std::cout<<"Computation Time:"<<(end-begin)/(double)CLOCKS_PER_SEC*1000.0<<"\tms"<<std::endl;\
//	}while(false);