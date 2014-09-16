/*--------------------------------------------------------------*/
/* qrouter entry point for non-Tcl compile version		*/
/*--------------------------------------------------------------*/

#include <stdio.h>

#include "qrouter.h"

/*--------------------------------------------------------------*/
/* Procedure main() performs the basic route steps without any	*/
/* interaction or scripting, writes a DEF file as output, and	*/
/* exits.							*/
/*--------------------------------------------------------------*/

int
main(int argc, char *argv[])
{
    int result;
 
    result = runqrouter(argc, argv);
    if (result != 0) return result;

    read_def(NULL);
    dofirststage(0);
    dosecondstage(0);
    write_def(NULL);
    return 0;
}

/*--------------------------------------------------------------*/
/* Define graphics routines as empty subroutines.  May want to	*/
/* provide a simple X11 graphics environment outside of Tcl/Tk	*/
/*--------------------------------------------------------------*/

void
highlight_source() {
}

void
highlight_dest() {
}

void
highlight_starts(POINT glist) {
}

void
highlight_mask() {
}

void
highlight(int x, int y) {
}

void
draw_net(NET net, u_char single, int *lastlayer) {
}

void
draw_layout() {
}

int
recalc_spacing() {
   return 0;
}
