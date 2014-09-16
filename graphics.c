/*------------------------------------------------------*/
/* Graphics routines for qrouter			*/
/*------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <X11/Intrinsic.h>
#include <X11/StringDefs.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>

#include <tk.h>

#include "qrouter.h"
#include "qconfig.h"
#include "node.h"
#include "maze.h"
#include "lef.h"

/*------------------------------*/
/* Type declarations		*/
/*------------------------------*/

void load_font(XFontStruct **);
void createGC(Window, GC *, XFontStruct *);

/*----------------------------------*/
/* Global variables for X11 drawing */
/*----------------------------------*/

XFontStruct *font_info;
Pixmap buffer = (Pixmap)0;
Display *dpy;
Window win;
Colormap cmap;
GC gc;
Dimension width, height;

#define SHORTSPAN 10
#define LONGSPAN  127

int spacing;
int bluepix, greenpix, redpix, cyanpix, orangepix, goldpix;
int blackpix, whitepix, graypix, ltgraypix, yellowpix;
int magentapix, purplepix, greenyellowpix;
int brownvector[SHORTSPAN];
int bluevector[LONGSPAN];

/*--------------------------------------------------------------*/
/* Highlight a position on the graph.  Do this on the actual	*/
/* screen, not the buffer.					*/
/*--------------------------------------------------------------*/

void highlight(int x, int y) {

    int i, xspc, yspc, hspc;
    PROUTE *Pr;

    // If Obs2[] at x, y is a source or dest, don't highlight
    // Do this only for layer 0;  it doesn't have to be rigorous. 
    for (i = 0; i < Num_layers; i++) {
	Pr = &Obs2[i][OGRID(x, y, i)];
	if (Pr->flags & (PR_SOURCE | PR_TARGET)) return;
    }

    hspc = spacing >> 1;
    if (hspc == 0) hspc = 1;

    xspc = (x + 1) * spacing - hspc;
    yspc = height - (y + 1) * spacing - hspc;

    XSetForeground(dpy, gc, yellowpix);
    XFillRectangle(dpy, win, gc, xspc, yspc, spacing, spacing);
    XFlush(dpy);
}

/*--------------------------------------*/
/* Highlight source (in magenta)	*/
/*--------------------------------------*/

void highlight_source() {

    int xspc, yspc, hspc;
    int i, x, y;
    PROUTE *Pr;

    if (Obs2[0] == NULL) return;

    // Determine the number of routes per width and height, if
    // it has not yet been computed

    hspc = spacing >> 1;
    if (hspc == 0) hspc = 1;

    // Draw source pins as magenta squares
    XSetForeground(dpy, gc, magentapix);
    for (i = 0; i < Num_layers; i++) {
	for (x = 0; x < NumChannelsX[i]; x++) {
	    xspc = (x + 1) * spacing - hspc;
	    for (y = 0; y < NumChannelsY[i]; y++) {
		Pr = &Obs2[i][OGRID(x, y, i)];
		if (Pr->flags & PR_SOURCE) {
		    yspc = height - (y + 1) * spacing - hspc;
		    XFillRectangle(dpy, win, gc, xspc, yspc,
				spacing, spacing);
		}
	    }
	}
    }
    XFlush(dpy);
}

/*--------------------------------------*/
/* Highlight destination (in purple)	*/
/*--------------------------------------*/

void highlight_dest() {

    int xspc, yspc, hspc, dspc;
    int i, x, y;
    PROUTE *Pr;

    if (Obs2[0] == NULL) return;

    // Determine the number of routes per width and height, if
    // it has not yet been computed

    dspc = spacing + 4;			// Make target more visible
    hspc = dspc >> 1;

    // Draw destination pins as purple squares
    XSetForeground(dpy, gc, purplepix);
    for (i = 0; i < Num_layers; i++) {
	for (x = 0; x < NumChannelsX[i]; x++) {
	    xspc = (x + 1) * spacing - hspc;
	    for (y = 0; y < NumChannelsY[i]; y++) {
		Pr = &Obs2[i][OGRID(x, y, i)];
		if (Pr->flags & PR_TARGET) {
		    yspc = height - (y + 1) * spacing - hspc;
		    XFillRectangle(dpy, win, gc, xspc, yspc,
				dspc, dspc);
		}
	    }
	}
    }
    XFlush(dpy);
}

/*----------------------------------------------*/
/* Highlight all the search starting points	*/
/*----------------------------------------------*/

void highlight_starts(POINT glist) {

    int xspc, yspc, hspc;
    POINT gpoint;

    // Determine the number of routes per width and height, if
    // it has not yet been computed

    hspc = spacing >> 1;

    XSetForeground(dpy, gc, greenyellowpix);
    for (gpoint = glist; gpoint; gpoint = gpoint->next) {
	xspc = (gpoint->x1 + 1) * spacing - hspc;
	yspc = height - (gpoint->y1 + 1) * spacing - hspc;
	XFillRectangle(dpy, win, gc, xspc, yspc, spacing, spacing);
    }
    XFlush(dpy);
}

/*--------------------------------------*/
/* Highlight mask (in tan)		*/
/*--------------------------------------*/

void highlight_mask() {

    int xspc, yspc, hspc;
    int x, y;
    u_char value;

    if (RMask == NULL) return;

    // Determine the number of routes per width and height, if
    // it has not yet been computed

    hspc = spacing >> 1;

    // Draw destination pins as tan squares
    for (x = 0; x < NumChannelsX[0]; x++) {
	xspc = (x + 1) * spacing - hspc;
	for (y = 0; y < NumChannelsY[0]; y++) {
	    XSetForeground(dpy, gc, brownvector[RMask[OGRID(x, y, 0)]]);
	    yspc = height - (y + 1) * spacing - hspc;
	    XFillRectangle(dpy, win, gc, xspc, yspc, spacing, spacing);
	}
    }
    XFlush(dpy);
}

/*----------------------------------------------*/
/* Draw a map of obstructions and pins		*/
/*----------------------------------------------*/

void
map_obstruction()
{
    int xspc, yspc, hspc;
    int i, x, y, n, norm;
    u_char *Congestion;
    u_char value, maxval;

    hspc = spacing >> 1;

    // Draw obstructions as light gray squares
    XSetForeground(dpy, gc, ltgraypix);
    for (i = 0; i < Num_layers; i++) {
	for (x = 0; x < NumChannelsX[i]; x++) {
	    xspc = (x + 1) * spacing - hspc;
	    for (y = 0; y < NumChannelsY[i]; y++) {
		if (Obs[i][OGRID(x, y, i)] & NO_NET) {
		    yspc = height - (y + 1) * spacing - hspc;
		    XFillRectangle(dpy, buffer, gc, xspc, yspc,
				spacing, spacing);
		}
	    }
	}
    }

    // Draw pins as gray squares
    XSetForeground(dpy, gc, graypix);
    for (i = 0; i < Num_layers; i++) {
	for (x = 0; x < NumChannelsX[i]; x++) {
	    xspc = (x + 1) * spacing - hspc;
	    for (y = 0; y < NumChannelsY[i]; y++) {
		if (Nodesav[i][OGRID(x, y, i)] != NULL) {
		    yspc = height - (y + 1) * spacing - hspc;
		    XFillRectangle(dpy, buffer, gc, xspc, yspc,
				spacing, spacing);
		}
	    }
	}
    }
}

/*----------------------------------------------*/
/* Draw a map of actual route congestion	*/
/*----------------------------------------------*/

void
map_congestion()
{
    int xspc, yspc, hspc;
    int i, x, y, n, norm;
    u_char *Congestion;
    u_char value, maxval;

    hspc = spacing >> 1;

    Congestion = (u_char *)calloc(NumChannelsX[0] * NumChannelsY[0],
			sizeof(u_char));

    // Analyze Obs[] array for congestion
    for (i = 0; i < Num_layers; i++) {
	for (x = 0; x < NumChannelsX[i]; x++) {
	    for (y = 0; y < NumChannelsY[i]; y++) {
		value = (u_char)0;
		n = Obs[i][OGRID(x, y, i)];
		if (n & ROUTED_NET) value++;
		if (n & BLOCKED_MASK) value++;
		if (n & NO_NET) value++;
		if (n & PINOBSTRUCTMASK) value++;
		Congestion[OGRID(x, y, 0)] += value;
	    }
	}
    }

    maxval = 0;
    for (x = 0; x < NumChannelsX[0]; x++) {
	for (y = 0; y < NumChannelsY[0]; y++) {
	    value = Congestion[OGRID(x, y, 0)];
	    if (value > maxval) maxval = value;
	}
    }
    norm = (LONGSPAN - 1) / maxval;

    // Draw destination pins as blue squares
    for (x = 0; x < NumChannelsX[0]; x++) {
	xspc = (x + 1) * spacing - hspc;
	for (y = 0; y < NumChannelsY[0]; y++) {
	    XSetForeground(dpy, gc, bluevector[norm * Congestion[OGRID(x, y, 0)]]);
	    yspc = height - (y + 1) * spacing - hspc;
	    XFillRectangle(dpy, buffer, gc, xspc, yspc, spacing, spacing);
	}
    }

    // Cleanup
    free(Congestion);
}

/*----------------------------------------------------------------------*/
/* Draw a map of route congestion estimated from net bounding boxes	*/
/*----------------------------------------------------------------------*/

void
map_estimate()
{
    NET net;
    int xspc, yspc, hspc;
    int i, x, y, nwidth, nheight, area, length, value;
    float density, *Congestion, norm, maxval;

    hspc = spacing >> 1;

    Congestion = (float *)calloc(NumChannelsX[0] * NumChannelsY[0],
			sizeof(float));

    // Use net bounding boxes to estimate congestion

    for (i = 0; i < Numnets; i++) {
	net = Nlnets[i];
	nwidth = (net->xmax - net->xmin + 1);
	nheight = (net->ymax - net->ymin + 1);
	area = nwidth * nheight;
	if (nwidth > nheight) {
	    length = nwidth + (nheight >> 1) * net->numnodes;
	}
	else {
	    length = nheight + (nwidth >> 1) * net->numnodes;
	}
	density = (float)length / (float)area;

	for (x = net->xmin; x < net->xmax; x++)
	    for (y = net->ymin; y < net->ymax; y++)
		Congestion[OGRID(x, y, 0)] += density;
    }

    maxval = 0.0;
    for (x = 0; x < NumChannelsX[0]; x++) {
	for (y = 0; y < NumChannelsY[0]; y++) {
	    density = Congestion[OGRID(x, y, 0)];
	    if (density > maxval) maxval = density;
	}
    }
    norm = (float)(LONGSPAN - 1) / maxval;

    // Draw destination pins as blue squares
    for (x = 0; x < NumChannelsX[0]; x++) {
	xspc = (x + 1) * spacing - hspc;
	for (y = 0; y < NumChannelsY[0]; y++) {
	    value = (int)(norm * Congestion[OGRID(x, y, 0)]);
	    XSetForeground(dpy, gc, bluevector[value]);
	    yspc = height - (y + 1) * spacing - hspc;
	    XFillRectangle(dpy, buffer, gc, xspc, yspc, spacing, spacing);
	}
    }

    // Cleanup
    free(Congestion);
}

/*--------------------------------------*/
/* Draw one net on the display		*/
/*--------------------------------------*/

void draw_net(NET net, u_char single, int *lastlayer) {

    int i, xspc, yspc;
    int layer;
    SEG seg;
    ROUTE rt;

    if (dpy == NULL) return;

    // Draw all nets, much like "emit_routes" does when writing
    // routes to the DEF file.

    rt = net->routes;
    if (single && rt)
	for (rt = net->routes; rt->next; rt = rt->next);

    for (; rt; rt = rt->next) {
	for (seg = rt->segments; seg; seg = seg->next) {
	    layer = seg->layer;
	    if (layer != *lastlayer) {
		*lastlayer = layer;
		switch(layer) {
		    case 0:
			XSetForeground(dpy, gc, bluepix);
			break;
		    case 1:
			XSetForeground(dpy, gc, redpix);
			break;
		    case 2:
			XSetForeground(dpy, gc, cyanpix);
			break;
		    case 3:
			XSetForeground(dpy, gc, goldpix);
			break;
		    default:
			XSetForeground(dpy, gc, greenpix);
			break;
		}
	    }
	    XDrawLine(dpy, buffer, gc, spacing * (seg->x1 + 1),
				height - spacing * (seg->y1 + 1),
				spacing * (seg->x2 + 1),
				height - spacing * (seg->y2 + 1));
	    if (single)
		XDrawLine(dpy, win, gc, spacing * (seg->x1 + 1),
				height - spacing * (seg->y1 + 1),
				spacing * (seg->x2 + 1),
				height - spacing * (seg->y2 + 1));
	}
    }   
    if (single) {
	// The following line to be removed
	XCopyArea(dpy, buffer, win, gc, 0, 0, width, height, 0, 0);
	XFlush(dpy);
    }
}

/*--------------------------------------*/
/* Graphical display of the layout	*/
/*--------------------------------------*/

void draw_layout() {

    int lastlayer, xspc, yspc, hspc;
    int i, x, y;
    NET net;
    NETLIST nlist;

    if (dpy == NULL) return;
    else if (buffer == (Pixmap)NULL) return;

    XSetForeground(dpy, gc, whitepix);
    XFillRectangle(dpy, buffer, gc, 0, 0, width, height);

    // Check if a netlist even exists
    if (Obs[0] == NULL) {
	XCopyArea(dpy, buffer, win, gc, 0, 0, width, height, 0, 0);
	return;
    }

    switch (mapType & MAP_MASK) {
	case MAP_OBSTRUCT:
	    map_obstruction();
	    break;
	case MAP_CONGEST:
	    map_congestion();
	    break;
	case MAP_ESTIMATE:
	    map_estimate();
	    break;
    }

    // Draw all nets, much like "emit_routes" does when writing
    // routes to the DEF file.

    if ((mapType & DRAW_MASK) == DRAW_ROUTES) {
	lastlayer = -1;
	for (i = 0; i < Numnets; i++) {
	    net = Nlnets[i];
	    draw_net(net, FALSE, &lastlayer);
	}
    }

    /* Copy double-buffer onto display window */
    XCopyArea(dpy, buffer, win, gc, 0, 0, width, height, 0, 0);
}

/*--------------------------------------*/
/* GUI initialization			*/
/*--------------------------------------*/

int GUI_init(Tcl_Interp *interp)
{
   Tk_Window tkwind, tktop;
   static char *qrouterdrawdefault = ".qrouter";
   char *qrouterdrawwin;
   XColor cvcolor, cvexact;
   int i;
   float frac;

   tktop = Tk_MainWindow(interp);
   if (tktop == NULL) {
      tcl_printf(stderr, "No Top-level Tk window available. . .\n");
      return;
   }

   qrouterdrawwin = (char *)Tcl_GetVar(interp, "drawwindow", TCL_GLOBAL_ONLY);
   if (qrouterdrawwin == NULL)
      qrouterdrawwin = qrouterdrawdefault;

   tkwind = Tk_NameToWindow(interp, qrouterdrawwin, tktop);
   
   if (tkwind == NULL) {
      tcl_printf(stderr, "The Tk window hierarchy must be rooted at"
		".qrouter or $drawwindow must point to the drawing window\n");		
      return TCL_ERROR;
   }
   
   Tk_MapWindow(tkwind);
   dpy = Tk_Display(tkwind);
   win = Tk_WindowId(tkwind);
   cmap = DefaultColormap (dpy, Tk_ScreenNumber(tkwind));

   load_font(&font_info);

   /* create GC for text and drawing */
   createGC(win, &gc, font_info);

   /* Initialize colors */

   XAllocNamedColor(dpy, cmap, "blue", &cvcolor, &cvexact);
   bluepix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "cyan", &cvcolor, &cvexact);
   cyanpix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "green", &cvcolor, &cvexact);
   greenpix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "red", &cvcolor, &cvexact);
   redpix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "orange", &cvcolor, &cvexact);
   orangepix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "gold", &cvcolor, &cvexact);
   goldpix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "gray70", &cvcolor, &cvexact);
   ltgraypix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "gray50", &cvcolor, &cvexact);
   graypix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "yellow", &cvcolor, &cvexact);
   yellowpix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "purple", &cvcolor, &cvexact);
   purplepix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "magenta", &cvcolor, &cvexact);
   magentapix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "GreenYellow", &cvcolor, &cvexact);
   greenyellowpix = cvcolor.pixel;
   blackpix = BlackPixel(dpy,DefaultScreen(dpy));
   whitepix = WhitePixel(dpy,DefaultScreen(dpy));

   cvcolor.flags = DoRed | DoGreen | DoBlue;
   for (i = 0; i < SHORTSPAN; i++) {
      frac = (float)i / (float)(SHORTSPAN - 1);
      /* gamma correction */
      frac = pow(frac, 0.5);

      cvcolor.green = (int)(53970 * frac);
      cvcolor.blue = (int)(46260 * frac);
      cvcolor.red = (int)(35980 * frac);
      XAllocColor(dpy, cmap, &cvcolor);
      brownvector[i] = cvcolor.pixel;
   }

   cvcolor.green = 0;
   cvcolor.red = 0;

   for (i = 0; i < LONGSPAN; i++) {
      frac = (float)i / (float)(LONGSPAN - 1);
      /* gamma correction */
      frac = pow(frac, 0.5);

      cvcolor.blue = (int)(65535 * frac);
      XAllocColor(dpy, cmap, &cvcolor);
      bluevector[i] = cvcolor.pixel;
   }
   return TCL_OK;	/* proceed to interpreter */
}

/*----------------------------------------------------------------*/

int QuitCallback(ClientData clientData, Tcl_Interp *interp,
	int objc, Tcl_Obj *objv[])
{
   exit(0);
   return TCL_OK;	// Statement not reached
}

/*----------------------------------------------------------------*/

void load_font(XFontStruct **font_info)
{
   char *fontname = "9x15";

   /* Load font and get font information structure. */

   if ((*font_info = XLoadQueryFont (dpy,fontname)) == NULL) {
      (void) Fprintf (stderr, "Cannot open 9x15 font\n");
      // exit(1);
   }
}

/*----------------------------------------------------------------*/

void createGC(Window win, GC *gc, XFontStruct *font_info)
{
   unsigned long valuemask = 0; /* ignore XGCvalues and use defaults */
   XGCValues values;
   unsigned int line_width = 1;
   int line_style = LineSolid;
   int cap_style = CapRound;
   int join_style = JoinRound;

   /* Create default Graphics Context */

   *gc = XCreateGC(dpy, win, valuemask, &values);

   /* specify font */

   if (font_info != NULL)
      XSetFont(dpy, *gc, font_info->fid);

   /* specify black foreground since default window background is
    * white and default foreground is undefined. */

   XSetForeground(dpy, *gc, blackpix);

   /* set line, fill attributes */

   XSetLineAttributes(dpy, *gc, line_width, line_style,
            cap_style, join_style);
   XSetFillStyle(dpy, *gc, FillSolid);
   XSetArcMode(dpy, *gc, ArcPieSlice);
}

/*----------------------------------------------------------------*/

void expose(Tk_Window tkwind)
{
   if (Tk_WindowId(tkwind) == 0) return;
   if (dpy == NULL) return;
   draw_layout();
}

/*----------------------------------------------------------------*/

int redraw(ClientData clientData, Tcl_Interp *interp, int objc,
	Tcl_Obj *objv[])
{
   draw_layout();
   return TCL_OK;
}

/*------------------------------------------------------*/
/* Call to recalculate the spacing if NumChannelsX[0]	*/
/* or NumChannelsY[0] changes.				*/
/*							*/
/* Return 1 if the spacing changed, 0 otherwise.	*/
/*------------------------------------------------------*/

int recalc_spacing()
{
   int xspc, yspc;
   int oldspacing = spacing;

   xspc = width / (NumChannelsX[0] + 1);
   yspc = height / (NumChannelsY[0] + 1);
   spacing = (xspc < yspc) ? xspc : yspc;
   if (spacing == 0) spacing = 1;

   return (spacing == oldspacing) ? 0 : 1;
}

/*----------------------------------------------------------------*/

void resize(Tk_Window tkwind, int locwidth, int locheight)
{
   Window window;

   if ((locwidth == 0) || (locheight == 0)) return;

   if (buffer != (Pixmap)0)
      XFreePixmap (Tk_Display(tkwind), buffer);

   if (Tk_WindowId(tkwind) == 0)
      Tk_MapWindow(tkwind);
   
   buffer = XCreatePixmap (Tk_Display(tkwind), Tk_WindowId(tkwind),
	locwidth, locheight, DefaultDepthOfScreen(Tk_Screen(tkwind)));

   width = locwidth;
   height = locheight;

   recalc_spacing();

   if (dpy) draw_layout();
}

/*----------------------------------------------------------------*/
