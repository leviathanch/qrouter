/*--------------------------------------------------------------*/
/* graphics.h -- graphics routines                             	*/
/*--------------------------------------------------------------*/

#ifndef GRAPHICS_H

#include <tk.h>

/* TODO: Can we make this include independent from qrouter.h ? */
#include "qrouter.h"

void   highlight(int, int);
void   highlight_source(NET net);
void   highlight_dest(NET net);
void   highlight_starts(POINT glist);
void   highlight_mask(NET net);

void   draw_net(NET net, u_char single, int *lastlayer);
void   draw_layout(void);
void   draw_ratnet(NET net);

int    GUI_init(Tcl_Interp *interp);
void   expose(Tk_Window tkwind);
int    redraw(ClientData clientData, Tcl_Interp *interp, int objc,
              Tcl_Obj *CONST objv[]);
int    recalc_spacing(void);
void   resize(Tk_Window tkwind, int locwidth, int locheight);

extern BOOL drawTrunk;

#define GRAPHICS_H
#endif
