#ifndef DSL_SMILEARN_H
#define DSL_SMILEARN_H

// {{SMILE_PUBLIC_HEADER}}

// client-side SMILEARN header
// DO NOT USE for internal SMILEARN development

#ifdef _WIN32
    #if (_MSC_VER >= 1400) && defined(NDEBUG) && (!defined(_SECURE_SCL) || _SECURE_SCL != 0)    
		#error SMILEARN : In release build _SECURE_SCL must be defined and set to zero for VS.NET 2005 or later
    #endif

    #ifndef SMILEARN_VC_NO_AUTOLINK
        // automatically link with SMILE library
        #ifdef NDEBUG
            #ifdef _DLL
                #pragma comment(lib, "smilearn_dyn")
            #else
                #pragma comment(lib, "smilearn")
            #endif 
        #else
			#ifdef _DLL
				#pragma comment(lib, "smilearn_dbg")
			#else
				#error SMILEARN : In debug build the runtime library must be set to Multi-threaded Debug DLL 
			#endif
        #endif
    #endif
#endif

#include "dataset.h"
#include "discretizer.h"
#include "nb.h"
#include "greedythickthinning.h"
#include "essentialsearch.h"
#include "bs.h"
#include "tan.h"
#include "pc.h"
#include "em.h"
#include "validator.h"
#include "datagenerator.h"
#include "pattern.h"
#include "dbcml.h"


#endif
