/**
* The Pillar Library
* Copyright (c) 2013 by Voxar Labs.-UFPE
* Permission to use, copy, modify, distribute and sell this software for any 
*     purpose is hereby granted without fee, provided that the above copyright 
*     notice appear in all copies and that both that copyright notice and this 
*     permission notice appear in supporting documentation.
* The author makes no representations about the 
*     suitability of this software for any purpose. It is provided "as is" 
*     without express or implied warranty.
*/
#ifndef __LibConfiguration_H__
#define __LibConfiguration_H__

/**Common policies used to import/export functions/methods or classes in libraries*/
#ifdef _MSC_VER
#define EXPORT __declspec(dllexport)
#define IMPORT __declspec(dllimport)
#else
#define EXPORT
#define IMPORT
#endif
#define C_NAME		 extern "C"

/**Useful macros for CUDA applications/libraries*/
#ifdef __CUDA_ARCH__	
	#define GPGPU __host__ __device__
#else
	#define GPGPU
#endif

#endif

/**Disable warning "macro redefinition"*/
#ifdef _MSC_VER
#pragma warning ( push )
#pragma warning ( disable: 4005 )

/**Import/Export policy*/
#ifdef NOT_LIBRARY
#define LIBRARY_POLICY IMPORT
#else
#define LIBRARY_POLICY EXPORT
#endif

#pragma warning ( pop )

#endif