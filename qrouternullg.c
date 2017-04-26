/*----------------------------------------------------------------------*/
/* qrouternullg.c							*/
/*----------------------------------------------------------------------*/
#define MAX_MEM 1024*1024*6

#include <stdio.h>
#include <sys/resource.h>

#include <X11/Xlib.h>
#include <tcl.h>

/*----------------------------------------------------------------------*/
/* Application initiation.  This is exactly like the AppInit routine	*/
/* for "tclsh", minus the cruft, but with "tcl_rcFileName" set to	*/
/* "qrouter.tcl" instead of "~/.tclshrc".				*/
/*----------------------------------------------------------------------*/

int
qrouter_AppInit(interp)
    Tcl_Interp *interp;
{
    if (Tcl_Init(interp) == TCL_ERROR) {
	return TCL_ERROR;
    }

    /* This is where we replace the home ".wishrc" file with	*/
    /* qrouter's startup script.				*/

    Tcl_SetVar(interp, "tcl_rcFileName", QROUTER_PATH "/qrouter.tcl",
		TCL_GLOBAL_ONLY);

    /* Additional variable can be used to tell if qrouter is in non-	*/
    /* graphics mode.							*/
    Tcl_SetVar(interp, "no_graphics_mode", "true", TCL_GLOBAL_ONLY);

    return TCL_OK;
}

/*----------------------------------------------------------------------*/
/* The main procedure;  replacement for "tclsh".			*/
/*----------------------------------------------------------------------*/

void set_limits(rlim_t rlim_cur, rlim_t rlim_max)
{
	struct rlimit rl; 
	// First get the time limit on CPU 
	getrlimit (RLIMIT_CPU, &rl); 
	// Change the time limit 
	rl.rlim_cur = rlim_cur;
	rl.rlim_max = rlim_max;
	// Now call setrlimit() to set the  
	// changed value. 
	setrlimit(RLIMIT_DATA, &rl); 
	getrlimit(RLIMIT_DATA, &rl); 
}

int
main(argc, argv)
   int argc;
   char **argv;
{
//     set_limits(MAX_MEM,MAX_MEM);
    XInitThreads();
    Tcl_Main(argc, argv, qrouter_AppInit);
    return 0;
}

/*----------------------------------------------------------------------*/
