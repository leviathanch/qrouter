/*--------------------------------------------------------------*/
/*  qrouter.c -- general purpose autorouter                     */
/*  Reads LEF libraries and DEF netlists, and generates an	*/
/*  annotated DEF netlist as output.				*/
/*--------------------------------------------------------------*/
/* Written by Tim Edwards, June 2011, based on code by Steve	*/
/* Beccue, 2003							*/
/*--------------------------------------------------------------*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>

#include "qrouter.h"
#include "qconfig.h"
#include "node.h"
#include "maze.h"
#include "lef.h"

int  Pathon = -1;
int  TotalRoutes = 0;

NET     *Nlnets;	// list of nets in the design
NET	CurNet;		// current net to route, used by 2nd stage
STRING  DontRoute;      // a list of nets not to route (e.g., power)
STRING  CriticalNet;    // list of critical nets to route first
GATE    GateInfo;       // standard cell macro information
GATE	PinMacro;	// macro definition for a pin
GATE    Nlgates;	// gate instance information
NETLIST FailedNets;	// list of nets that failed to route

u_char *RMask;    	     // mask out best area to route
u_int  *Obs[MAX_LAYERS];     // net obstructions in layer
PROUTE *Obs2[MAX_LAYERS];    // used for pt->pt routes on layer
float  *Stub[MAX_LAYERS];    // used for stub routing to pins
float  *Obsinfo[MAX_LAYERS]; // temporary array used for detailed obstruction info
NODE   *Nodeloc[MAX_LAYERS]; // nodes are here. . .
NODE   *Nodesav[MAX_LAYERS]; // . . . and here (but not to be altered)
DSEG   UserObs;		     // user-defined obstruction layers

u_char needblock[MAX_LAYERS];

char *vddnet = NULL;
char *gndnet = NULL;

int    Numnets = 0;
u_char Verbose = 3;	// Default verbose level
u_char keepTrying = FALSE;
u_char forceRoutable = FALSE;
u_char maskMode = MASK_AUTO;
u_char mapType = MAP_OBSTRUCT | DRAW_ROUTES;

char DEFfilename[256];

ScaleRec Scales;	// record of input and output scales

/*--------------------------------------------------------------*/
/* Check track pitch and set the number of channels (may be	*/
/* called from DefRead)						*/
/*--------------------------------------------------------------*/

int set_num_channels()
{
   int i;

   if (NumChannelsX[0] != 0) return;	/* Already been called */

   for (i = 0; i < Num_layers; i++) {
      if (PitchX[i] == 0.0 || PitchY[i] == 0.0) {
	 Fprintf(stderr, "Have a 0 pitch for layer %d (of %d).  "
			"Exit.\n", i + 1, Num_layers);
	 return (-3);
      }
      NumChannelsX[i] = (int)(1.5 + (Xupperbound - Xlowerbound) / PitchX[i]);
      NumChannelsY[i] = (int)(1.5 + (Yupperbound - Ylowerbound) / PitchY[i]);
      if ((Verbose > 1) || (NumChannelsX[i] <= 0))
	 Fprintf(stdout, "Number of x channels for layer %d is %d\n",
		i, NumChannelsX[i]);
      if ((Verbose > 1) || (NumChannelsY[i] <= 0))
	 Fprintf(stdout, "Number of y channels for layer %d is %d\n",
		i, NumChannelsY[i]);
	
      if (NumChannelsX[i] <= 0) {
	 Fprintf(stderr, "Something wrong with layer %d x bounds.\n", i);
	 return(-3);
      }
      if (NumChannelsY[i] <= 0) {
	 Fprintf(stderr, "Something wrong with layer %d y bounds.\n", i);
	 return(-3);
      }
      Flush(stdout);
   }

   if (recalc_spacing()) draw_layout();
   return 0;
}

/*--------------------------------------------------------------*/
/* Allocate the Obs[] array (may be called from DefRead)	*/
/*--------------------------------------------------------------*/

int allocate_obs_array()
{
   int i;

   if (Obs[0] != NULL) return;	/* Already been called */

   for (i = 0; i < Num_layers; i++) {
      Obs[i] = (u_int *)calloc(NumChannelsX[i] * NumChannelsY[i],
			sizeof(u_int));
      if (!Obs[i]) {
	 Fprintf(stderr, "Out of memory 4.\n");
	 return(4);
      }
   }
   return 0;
}

/*--------------------------------------------------------------*/
/* countlist ---						*/
/*   Count the number of entries in a simple linked list	*/
/*--------------------------------------------------------------*/

int
countlist(NETLIST net)
{
   NETLIST nptr = net;
   int count = 0;

   while (nptr != NULL) {
      count++;
      nptr = nptr->next;
   }
   return count;
}

/*--------------------------------------------------------------*/
/* runqrouter - main program entry point, parse command line	*/
/*								*/
/*   ARGS: argc (count) argv, command line 			*/
/*   RETURNS: to OS						*/
/*   SIDE EFFECTS: 						*/
/*--------------------------------------------------------------*/

int
runqrouter(int argc, char *argv[])
{
   int	i, j, result;
   int length, width;
   FILE *l, *configFILEptr, *fptr;
   u_int u;
   static char configdefault[] = CONFIGFILENAME;
   char *configfile = configdefault;
   char *infofile = NULL;
   char *dotptr, *sptr;
   char Filename[256];
   double sreq;
   NET net;
   u_char readconfig = FALSE;
    
   Scales.iscale = 1;
   Filename[0] = 0;
   DEFfilename[0] = 0;

   while ((i = getopt(argc, argv, "c:i:hkfv:p:g:r:")) != -1) {
      switch (i) {
	 case 'c':
	    configfile = strdup(optarg);
	    break;
	 case 'v':
	    Verbose = atoi(optarg);
	    break;
	 case 'i':
	    infofile = strdup(optarg);
	    break;
	 case 'p':
	    vddnet = strdup(optarg);
	    break;
	 case 'g':
	    gndnet = strdup(optarg);
	    break;
	 case 'r':
	    if (sscanf(optarg, "%d", &Scales.iscale) != 1) {
		Fprintf(stderr, "Bad resolution scalefactor \"%s\", "
			"integer expected.\n", optarg);
		Scales.iscale = 1;
	    }
	    break;
	 case 'h':
	    helpmessage();
	    return 1;
	    break;
	 case 'f':
	    forceRoutable = 1;
	    break;
	 case 'k':
	    keepTrying = 1;
	    break;
	 default:
	    Fprintf(stderr, "bad switch %d\n", i);
      }
   }

   configFILEptr = fopen(configfile, "r");

   if (configFILEptr) {
       read_config(configFILEptr);
       readconfig = TRUE;
   }
   else {
      if (configfile != configdefault)
	 Fprintf(stderr, "Could not open %s\n", configfile );
      else
	 Fprintf(stdout, "No .cfg file specified, continuing without.\n");
   }
   if (configfile != configdefault) free(configfile);

   if (infofile != NULL) {
      FILE *infoFILEptr;

      infoFILEptr = fopen(infofile, "w" );
      if (infoFILEptr != NULL) {

	 /* Print qrouter name and version number at the top */
#ifdef TCL_QROUTER
	 fprintf(infoFILEptr, "qrouter %s.%s.T\n", VERSION, REVISION);
#else
	 fprintf(infoFILEptr, "qrouter %s.%s\n", VERSION, REVISION);
#endif

         /* Print information about route layers, and exit */
         for (i = 0; i < Num_layers; i++) {
	    char *layername = LefGetRouteName(i);
	    if (layername != NULL)
	       fprintf(infoFILEptr, "%s %g %g %g %s\n",
			layername, LefGetRoutePitch(i),
			LefGetRouteOffset(i), LefGetRouteWidth(i),
			(LefGetRouteOrientation(i) == 1) ? "horizontal"
			: "vertical");
	 }
	 fclose(infoFILEptr);
      }
      return 1;
   }

   if (optind < argc) {

      /* process remaining commandline strings */

      strcpy( Filename, argv[optind] );
      dotptr = strrchr(Filename, '.');
      if (dotptr != NULL) *dotptr = '\0';
      sprintf(DEFfilename, "%s.def", Filename);
   }
   else if (readconfig) {
      Fprintf(stdout, "No netlist file specified, continuing without.\n");
      helpmessage();
      return 1;
   }

   Obs[0] = (u_int *)NULL;
   NumChannelsX[0] = 0;	// This is so we can check if NumChannelsX/Y were
			// set from within DefRead() due to reading in
			// existing nets.

   Scales.oscale = 1.0;
   return 0;
}

/*--------------------------------------------------------------*/
/* reinitialize ---						*/
/*								*/
/* Free up memory in preparation for reading another DEF file	*/
/*--------------------------------------------------------------*/

void reinitialize()
{
    int i;
    NETLIST nl;
    NET net;
    ROUTE rt;
    SEG seg;
    DSEG obs, tap;
    NODE node;
    GATE gate;
    DPOINT dpt;

    // Free up all of the matrices

    for (i = 0; i < Num_layers; i++) {
	free(Stub[i]);
	free(Nodeloc[i]);
	free(Nodesav[i]);
	free(Obs2[i]);
	free(Obs[i]);

	Stub[i] = NULL;
	Nodeloc[i] = NULL;
	Nodesav[i] = NULL;
	Obs2[i] = NULL;
	Obs[i] = NULL;
    }

    // Free the netlist of failed nets (if there is one)

    while (FailedNets) {
	nl = FailedNets;
	FailedNets = FailedNets->next;
	free(nl);
    }

    // Free all net and route information

    for (i = 0; i < Numnets; i++) {
	net = Nlnets[i];
	while (net->noripup) {
	    nl = net->noripup;
	    net->noripup = net->noripup->next;
	    free(nl);
	}
	while (net->routes) {
	    rt = net->routes;
	    net->routes = net->routes->next;
	    while (rt->segments) {
		seg = rt->segments;
		rt->segments = rt->segments->next;
		free(seg);
	    }
	    free(rt);
	}
	while (net->netnodes) {
	    node = net->netnodes;
	    net->netnodes = net->netnodes->next;

	    while (node->taps) {
		dpt = node->taps;
		node->taps = node->taps->next;
		free(dpt);
	    }
	    while (node->extend) {
		dpt = node->extend;
		node->extend = node->extend->next;
		free(dpt);
	    }
	    // Note: node->netname is not allocated
	    // but copied from net record
	    free(node);
	}
	free (net->netname);
	free (net);
    }
    free(Nlnets);
    Nlnets = NULL;
    Numnets = 0;

    // Free all gates information

    while (Nlgates) {
	gate = Nlgates;
	Nlgates = Nlgates->next;
	while (gate->obs) {
	    obs = gate->obs;
	    gate->obs = gate->obs->next;
	    free(obs);
	}
	for (i = 0; i < gate->nodes; i++) {
	    while (gate->taps[i]) {
	        tap = gate->taps[i];
		gate->taps[i] = gate->taps[i]->next;
		free(tap);
	    }
	    // Note: gate->node[i] is not allocated
	    // but copied from cell record in GateInfo
	    // Likewise for gate->noderec[i]
	}

	free(gate->gatename);
    }
    Nlgates = NULL;
}

/*--------------------------------------------------------------*/
/* post_def_setup ---						*/
/*								*/
/* Things to do after a DEF file has been read in, and the size	*/
/* of the layout, components, and nets are known.		*/
/*--------------------------------------------------------------*/

int post_def_setup()
{
   NET net;
   int i;
   double sreq1, sreq2;

   if (DEFfilename[0] == '\0') {
      Fprintf(stderr, "No DEF file read, nothing to set up.\n");
      return 1;
   }
   else {
      if (Num_layers <= 0) {
         Fprintf(stderr, "No routing layers defined, nothing to do.\n");
         return 1;
      }
   }

   for (i = 0; i < Numnets; i++) {
      net = Nlnets[i];
      find_bounding_box(net);
      defineRouteTree(net);
   }

   create_netorder(0);		// Choose ordering method (0 or 1)

   set_num_channels();		// If not called from DefRead()
   allocate_obs_array();	// If not called from DefRead()

   initMask();

   for (i = 0; i < Num_layers; i++) {

      Obsinfo[i] = (float *)calloc(NumChannelsX[i] * NumChannelsY[i],
			sizeof(float));
      if (!Obsinfo[i]) {
	 fprintf(stderr, "Out of memory 5.\n");
	 exit(5);
      }

      Stub[i] = (float *)calloc(NumChannelsX[i] * NumChannelsY[i],
			sizeof(float));
      if (!Stub[i]) {
	 fprintf( stderr, "Out of memory 6.\n");
	 exit(6);
      }

      // Nodeloc is the reverse lookup table for nodes

      Nodeloc[i] = (NODE *)calloc(NumChannelsX[i] * NumChannelsY[i],
		sizeof(NODE));
      if (!Nodeloc[i]) {
         fprintf(stderr, "Out of memory 7.\n");
         exit(7);
      }

      Nodesav[i] = (NODE *)calloc(NumChannelsX[i] * NumChannelsY[i],
		sizeof(NODE));
      if (!Nodesav[i]) {
         fprintf(stderr, "Out of memory 8.\n");
         exit(8);
      }
   }
   Flush(stdout);

   if (Verbose > 1)
      Fprintf(stderr, "Diagnostic: memory block is %d bytes\n",
		sizeof(u_int) * NumChannelsX[0] * NumChannelsY[0]);

   /* Be sure to create obstructions from gates first, since we don't	*/
   /* want improperly defined or positioned obstruction layers to over-	*/
   /* write our node list.						*/

   expand_tap_geometry();
   create_obstructions_from_gates();
   create_obstructions_from_nodes();
   tap_to_tap_interactions();
   create_obstructions_from_variable_pitch();
   adjust_stub_lengths();
   find_route_blocks();
   
   // If any nets are pre-routed, place those routes.

   for (i = 0; i < Numnets; i++) {
      net = Nlnets[i];
      writeback_all_routes(net);
   }

   // Remove the Obsinfo array, which is no longer needed, and allocate
   // the Obs2 array for costing information

   for (i = 0; i < Num_layers; i++) free(Obsinfo[i]);

   for (i = 0; i < Num_layers; i++) {
      Obs2[i] = (PROUTE *)calloc(NumChannelsX[i] * NumChannelsY[i],
			sizeof(PROUTE));
      if (!Obs2[i]) {
         fprintf( stderr, "Out of memory 9.\n");
         exit(9);
      }
   }

   // Fill in needblock bit fields, which are used by commit_proute
   // when route layers are too large for the grid size, and grid points
   // around a route need to be marked as blocked whenever something is
   // routed on those layers.

   // "ROUTEBLOCK" is set if the spacing is violated between a normal
   // route and an adjacent via.  "VIABLOCK" is set if the spacing is
   // violated between two adjacent vias.  It may be helpful to define
   // a third category which is route-to-route spacing violation.

   for (i = 0; i < Num_layers; i++) {
      needblock[i] = (u_char)0;
      sreq1 = LefGetRouteSpacing(i);

      if (i == 0)
	 sreq2 = LefGetViaWidth(i, i, 0) + sreq1;
      else
	 sreq2 = LefGetViaWidth(i - 1, i, 0) + sreq1;
      if ((sreq2 - EPS) > PitchX[i]) needblock[i] |= VIABLOCKX;
      if (i == 0)
	 sreq2 = LefGetViaWidth(i, i, 1) + sreq1;
      else
	 sreq2 = LefGetViaWidth(i - 1, i, 1) + sreq1;
      if ((sreq2 - EPS) > PitchY[i]) needblock[i] |= VIABLOCKY;

      sreq1 += 0.5 * LefGetRouteWidth(i);
      if (i == 0)
	 sreq2 = sreq1 + 0.5 * LefGetViaWidth(i, i, 0);
      else
	 sreq2 = sreq1 + 0.5 * LefGetViaWidth(i - 1, i, 0);
      if ((sreq2 - EPS) > PitchX[i]) needblock[i] |= ROUTEBLOCKX;
      if (i == 0)
	 sreq2 = sreq1 + 0.5 * LefGetViaWidth(i, i, 1);
      else
	 sreq2 = sreq1 + 0.5 * LefGetViaWidth(i - 1, i, 1);
      if ((sreq2 - EPS) > PitchY[i]) needblock[i] |= ROUTEBLOCKY;
   }

   // Now we have netlist data, and can use it to get a list of nets.

   FailedNets = (NETLIST)NULL;
   Flush(stdout);
   if (Verbose > 0)
      Fprintf(stdout, "There are %d nets in this design.\n", Numnets);

   return 0;
}

/*--------------------------------------------------------------*/
/* read_def ---							*/
/*								*/
/* Read in the DEF file in DEFfilename				*/
/*--------------------------------------------------------------*/

void read_def(char *filename)
{
   if ((filename == NULL) && (DEFfilename[0] == '\0')) {
      Fprintf(stderr, "No DEF file specified, nothing to read.\n");
      return;
   }
   else if (filename != NULL) {
      if (DEFfilename[0] != '\0') reinitialize();
      strcpy(DEFfilename, filename);
   }
   else reinitialize();

   Scales.oscale = (double)((float)Scales.iscale * DefRead(DEFfilename));
   post_def_setup();
}

/*--------------------------------------------------------------*/
/*--------------------------------------------------------------*/

int dofirststage(u_char graphdebug)
{
   int i, failcount, remaining, result;
   NET net;
   NETLIST nl;

   // Clear the lists of failed routes, in case first
   // stage is being called more than once.

   while (FailedNets) {
      nl = FailedNets->next;
      free(FailedNets);
      FailedNets = nl;
   }

   // Now find and route all the nets

   remaining = Numnets;
 
   for (i = 0; i < Numnets; i++) {
      net = getnettoroute(i);
      if ((net != NULL) && (net->netnodes != NULL)) {
	 result = doroute(net, (u_char)0, graphdebug);
	 if (result == 0) {
	    remaining--;
	    if (Verbose > 0)
	       Fprintf(stdout, "Finished routing net %s\n", net->netname);
	    Fprintf(stdout, "Nets remaining: %d\n", remaining);
	 }
	 else {
	    if (Verbose > 0)
	       Fprintf(stdout, "Failed to route net %s\n", net->netname);
	 }
      }
      else {
	 if (net && (Verbose > 0)) {
	    Fprintf(stdout, "Nothing to do for net %s\n", net->netname);
	 }
	 remaining--;
      }
   }
   failcount = countlist(FailedNets);

   if (Verbose > 0) {
      Flush(stdout);
      Fprintf(stdout, "\n----------------------------------------------\n");
      Fprintf(stdout, "Progress: ");
      Fprintf(stdout, "Stage 1 total routes completed: %d\n", TotalRoutes);
   }
   if (FailedNets == (NETLIST)NULL)
      Fprintf(stdout, "No failed routes!\n");
   else {
      if (FailedNets != (NETLIST)NULL)
          Fprintf(stdout, "Failed net routes: %d\n", failcount);
   }
   if (Verbose > 0)
      Fprintf(stdout, "----------------------------------------------\n");

   return failcount;
}

/*--------------------------------------------------------------*/
/*--------------------------------------------------------------*/

int write_def(char *filename)
{
   NET net;
   NETLIST nl;

   // Finish up by writing the routes to an annotated DEF file
    
   emit_routes((filename == NULL) ? DEFfilename : filename,
		Scales.oscale, Scales.iscale);

   Fprintf(stdout, "----------------------------------------------\n");
   Fprintf(stdout, "Final: ");
   if (FailedNets == (NETLIST)NULL)
      Fprintf(stdout, "No failed routes!\n");
   else {
      if (FailedNets != (NETLIST)NULL) {
         Fprintf(stdout, "Failed net routes: %d\n", countlist(FailedNets));
	 Fprintf(stdout, "List of failed nets follows:\n");

	 // Make sure FailedNets is cleaned up as we output the failed nets

 	 while (FailedNets) {
	    net = FailedNets->net;
	    Fprintf(stdout, " %s\n", net->netname);
	    nl = FailedNets->next;
	    free(FailedNets);
	    FailedNets = nl;
	 }
	 Fprintf(stdout, "\n");
      }
   }
   Fprintf(stdout, "----------------------------------------------\n");

   return 0;

} /* write_def() */

/*--------------------------------------------------------------*/
/* pathstart - begin a DEF format route path           		*/
/*								*/
/* 	If "special" is true, then this path is in a		*/
/*	SPECIALNETS section, in which each route specifies	*/
/*	a width.						*/
/*--------------------------------------------------------------*/

void pathstart(FILE *cmd, int layer, int x, int y, u_char special, double oscale,
		double invscale, u_char horizontal)
{
   if (Pathon == 1) {
      Fprintf( stderr, "pathstart():  Major error.  Started a new "
		"path while one is in progress!\n"
		"Doing it anyway.\n" );
   }

   if (layer >= 0) {
      if (Pathon == -1)
	 fprintf(cmd, "+ ROUTED ");
      else
	 fprintf(cmd, "\n  NEW ");
      if (special) {
	 double wvia;

	 if (layer == 0)
	    wvia = LefGetViaWidth(layer, layer, horizontal);
	 else
	    wvia = LefGetViaWidth(layer - 1, layer, horizontal);

         fprintf(cmd, "%s %g ( %g %g ) ", CIFLayer[layer],
			invscale * (int)(oscale * wvia + 0.5),
			invscale * x, invscale * y);
      }
      else
         fprintf(cmd, "%s ( %g %g ) ", CIFLayer[layer], invscale * x, invscale * y);
   }
   Pathon = 1;

} /* pathstart() */

/*--------------------------------------------------------------*/
/* pathto  - continue a path to the next point        		*/
/*								*/
/*   ARGS: coordinate pair					*/
/*   RETURNS: 							*/
/*   SIDE EFFECTS: 						*/
/*--------------------------------------------------------------*/

void pathto(FILE *cmd, int x, int y, int horizontal, int lastx, int lasty,
		double invscale)
{
    if (Pathon <= 0) {
	Fprintf(stderr, "pathto():  Major error.  Added to a "
		"non-existent path!\n"
		"Doing it anyway.\n");
    }

    /* If the route is not manhattan, then it's because an offset
     * was added to the last point, and we need to add a small
     * jog to the route.
     */

    if ((x != lastx) && (y != lasty)) {
	if (horizontal)
	   pathto(cmd, x, y, FALSE, x, lasty, invscale);
	else
	   pathto(cmd, x, y, TRUE, lastx, y, invscale);
    }

    fprintf(cmd, "( ");
    if (horizontal)
	fprintf(cmd, "%g ", invscale * x);
    else
	fprintf(cmd, "* ");

    if (horizontal)
	fprintf(cmd, "* ");
    else
	fprintf(cmd, "%g ", invscale * y);

    fprintf(cmd, ") ");

} /* pathto() */

/*--------------------------------------------------------------*/
/* pathvia  - add a via to a path               		*/
/*								*/
/*   ARGS: coord						*/
/*   RETURNS: 							*/
/*   SIDE EFFECTS: 						*/
/*--------------------------------------------------------------*/

void pathvia(FILE *cmd, int layer, int x, int y, int lastx, int lasty,
		int gridx, int gridy, double invscale)
{
    char *s;
    char checkersign = (gridx + gridy + layer) & 0x01;

    if ((ViaPattern == VIA_PATTERN_NONE) || (ViaY[layer] == NULL))
	s = ViaX[layer];
    else if (ViaPattern == VIA_PATTERN_NORMAL)
	s = (checkersign == 0) ?  ViaX[layer] : ViaY[layer];
    else
	s = (checkersign == 0) ?  ViaY[layer] : ViaX[layer];

    if (Pathon <= 0) {
       if (Pathon == -1)
	  fprintf(cmd, "+ ROUTED ");
       else 
	  fprintf(cmd, "\n  NEW ");
       fprintf(cmd, "%s ( %g %g ) ", CIFLayer[layer], invscale * x, invscale * y);
    }
    else {
       // Normally the path will be manhattan and only one of
       // these will be true.  But if the via gets an offset to
       // avoid a DRC spacing violation with an adjacent via,
       // then we may need to apply both paths to make a dog-leg
       // route to the via.

       if (x != lastx)
	  pathto(cmd, x, y, TRUE, lastx, y, invscale);
       if (y != lasty)
	  pathto(cmd, x, y, FALSE, x, lasty, invscale);
    }
    fprintf(cmd, "%s ", s);
    Pathon = 0;

} /* pathvia() */

/*--------------------------------------------------------------*/
/* print_nets - print the nets list - created from Nlgates list */
/*								*/
/*   ARGS: filename to list to					*/
/*   RETURNS: nothing						*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Sat July 26		*/
/*--------------------------------------------------------------*/

void print_nets(char *filename)
{
   FILE *o;
   GATE g;
   int i;
   DSEG drect;

   if (!strcmp(filename, "stdout")) {
	o = stdout;
   } else {
	o = fopen(filename, "w");
   }
   if (!o) {
	Fprintf(stderr, "route:print_nets.  Couldn't open output file\n");
	return;
   }

   for (g = Nlgates; g; g = g->next) {
      fprintf(o, "%s: %s: nodes->", g->gatename, g->gatetype->gatename);
      for (i = 0; i < g->nodes; i++) {
	 // This prints the first tap position only.
	 drect = g->taps[i];
	 fprintf( o, "%s(%g,%g) ", g->node[i], drect->x1, drect->y1);
      }
   }
   fprintf( o, "\n");
} /* print_nets() */

/*--------------------------------------------------------------*/
/* print_routes - print the routes list				*/
/*								*/
/*   ARGS: filename to list to					*/
/*   RETURNS: nothing						*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Sat July 26		*/
/*--------------------------------------------------------------*/

void print_routes( char *filename )
{
    FILE *o;
    GATE g;
    int i;

    if( !strcmp( filename, "stdout" ) ) {
	o = stdout;
    } else {
	o = fopen( filename, "w" );
    }
    if( !o ) {
	Fprintf( stderr, "route:print_routes.  Couldn't open output file\n" );
	return;
    }

    for (g = Nlgates; g; g = g->next) {
	fprintf( o, "%s: %s: nodes->", g->gatename, g->gatetype->gatename );
	for( i = 0 ; i < g->nodes; i++ ) {
	    fprintf( o, "%s ", g->node[i] );
	}
	fprintf(o, "\n");
    }
} /* print_routes() */

/*--------------------------------------------------------------*/
/* print_nlgates - print the nlgate list			*/
/*								*/
/*   ARGS: filename to list to					*/
/*   RETURNS: nothing						*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Wed July 23		*/
/*--------------------------------------------------------------*/

void print_nlgates( char *filename )
{
    FILE *o;
    GATE g;
    int i;
    DSEG drect;

    if( !strcmp( filename, "stdout" ) ) {
	o = stdout;
    } else {
	o = fopen( filename, "w" );
    }
    if( !o ) {
	Fprintf( stderr, "route:print_nlgates.  Couldn't open output file\n" );
	return;
    }

    for (g = Nlgates; g; g = g->next) {
	fprintf( o, "%s: %s: nodes->", g->gatename, g->gatetype->gatename );
	for( i = 0 ; i < g->nodes; i++ ) {
	    // This prints the first tap position only.
	    drect = g->taps[i];
	    fprintf( o, "%s(%g,%g)", g->node[i], drect->x1, drect->y1);
	}
        fprintf(o, "\n");
    }
} /* print_nlgates() */

/*--------------------------------------------------------------*/
/* getnettoroute - get a net to route				*/
/*								*/
/*   ARGS: 							*/
/*   RETURNS: 							*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Fri Aug 8		*/
/*--------------------------------------------------------------*/

NET getnettoroute(int order)
{
   NET net;

   net = Nlnets[order]; 
   if (net == NULL) return NULL;
  
   if (net->flags & NET_IGNORED) return NULL;
   if (net->numnodes >= 2) return net;

   // Qrouter will route power and ground nets even if the
   // standard cell power and ground pins are not listed in
   // the nets section.  Because of this, it is okay to have
   // only one node.

   if ((net->numnodes == 1) && (net->netnum == VDD_NET ||
		net->netnum == GND_NET))
      return net;

   if (Verbose > 3) {
      Flush(stdout);
      Fprintf(stderr, "getnettoroute():  Fell through\n");
   }
   return NULL;

} /* getnettoroute() */

/*--------------------------------------------------------------*/
/* Find all routes that collide with net "net", remove them	*/
/* from the Obs[] matrix, append them to the FailedNets list,	*/
/* and then write the net "net" back to the Obs[] matrix.	*/
/*								*/
/* Return the number of nets ripped up				*/
/*--------------------------------------------------------------*/

int ripup_colliding(NET net)
{
    NETLIST nl, nl2, fn;
    int ripped = 0;

    // Analyze route for nets with which it collides

    nl = find_colliding(net);

    // Remove the colliding nets from the route grid and append
    // them to FailedNets.

    while(nl) {
	ripped++;
	nl2 = nl->next;
	if (Verbose > 0)
            Fprintf(stdout, "Ripping up blocking net %s\n", nl->net->netname);
	if (ripup_net(nl->net, (u_char)1) == TRUE) { 
	    for (fn = FailedNets; fn && fn->next != NULL; fn = fn->next);
	    if (fn)
		fn->next = nl;
	    else
		FailedNets = nl;

	    // Add nl->net to "noripup" list for this net, so it won't be
	    // routed over again by the net.  Avoids infinite looping in
	    // the second stage.

	    fn = (NETLIST)malloc(sizeof(struct netlist_));
	    fn->next = net->noripup;
	    net->noripup = fn;
	    fn->net = nl->net;
	}

	nl->next = (NETLIST)NULL;
	nl = nl2;
     }

     // Now we copy the net we routed above into Obs
     writeback_all_routes(net);
     return ripped;
}

/*--------------------------------------------------------------*/
/* Do a second-stage route (rip-up and re-route) of a single	*/
/* net "net".							*/
/*--------------------------------------------------------------*/

int route_net_ripup(NET net, u_char graphdebug)
{
    int result;
    NETLIST nl, nl2;

    // Find the net in the Failed list and remove it.
    if (FailedNets->net == net) {
	nl2 = FailedNets;
	FailedNets = FailedNets->next;
	free(nl2);
    }
    else {
	for (nl = FailedNets; nl->next; nl = nl->next) {
	    if (nl->next->net == net)
		break;
	}
	nl2 = nl->next;
	nl2->next = nl2->next->next;
	free(nl2);
    }

    result = doroute(net, (u_char)1, graphdebug);
    if (result != 0) {
	if (net->noripup != NULL) {
	    if ((net->flags & NET_PENDING) == 0) {
		// Clear this net's "noripup" list and try again.

		while (net->noripup) {
		    nl = net->noripup->next;
		    free(net->noripup);
		    net->noripup = nl;
		}
		result = doroute(net, (u_char)1, graphdebug);
		net->flags |= NET_PENDING;	// Next time we abandon it.
	    }
	}
    }
    if (result != 0)
	result = ripup_colliding(net);
    return result;
}

/*--------------------------------------------------------------*/
/* dosecondstage() ---						*/
/*								*/
/* Second stage:  Rip-up and reroute failing nets.		*/
/* Method:							*/
/* 1) Route a failing net with stage = 1 (other nets become	*/
/*    costs, not blockages, no copying to Obs)			*/
/* 2) If net continues to fail, flag it as unroutable and	*/
/*    remove it from the list.					*/
/* 3) Otherwise, determine the nets with which it collided.	*/
/* 4) Remove all of the colliding nets, and add them to the	*/
/*    FailedNets list						*/
/* 5) Route the original failing net.				*/
/* 6) Continue until all failed nets have been processed.	*/
/*								*/
/* Return value:  The number of failing nets			*/
/*--------------------------------------------------------------*/

int
dosecondstage(u_char graphdebug)
{
   int failcount, origcount, result, maxtries, lasttries;
   NET net;
   NETLIST nl, nl2, fn;
   NETLIST Abandoned;	// Abandoned routes---not even trying any more.

   origcount = countlist(FailedNets);
   if (FailedNets)
      maxtries = TotalRoutes + ((origcount < 20) ? 20 : origcount) * 8;
   else
      maxtries = 0;

   fillMask(0);
   Abandoned = NULL;

   while (FailedNets != NULL) {

      // Diagnostic:  how are we doing?
      failcount = countlist(FailedNets);
      if (Verbose > 1) Fprintf(stdout, "------------------------------\n");
      Fprintf(stdout, "Nets remaining: %d\n", failcount);
      if (Verbose > 1) Fprintf(stdout, "------------------------------\n");

      net = FailedNets->net;

      // Remove this net from the fail list
      nl2 = FailedNets;
      FailedNets = FailedNets->next;
      free(nl2);

      if (Verbose > 2)
	 Fprintf(stdout, "Routing net %s with collisions\n", net->netname);
      Flush(stdout);

      result = doroute(net, (u_char)1, graphdebug);
      if (result != 0) {
	 if (net->noripup != NULL) {
	    if ((net->flags & NET_PENDING) == 0) {
	       // Clear this net's "noripup" list and try again.

	       while (net->noripup) {
	          nl = net->noripup->next;
	          free(net->noripup);
	          net->noripup = nl;
	       }
	       result = doroute(net, (u_char)1, graphdebug);
	       net->flags |= NET_PENDING;	// Next time we abandon it.
	    }
	 }
      }
      if (result != 0) {
	 // Complete failure to route, even allowing collisions.
	 // Abandon routing this net.
	 if (Verbose > 0) {
	    Flush(stdout);
	    Fprintf(stderr, "----------------------------------------------\n");
	    Fprintf(stderr, "Complete failure on net %s:  Abandoning.\n",
			net->netname);
	    Fprintf(stderr, "----------------------------------------------\n");
	 }
	 // Add the net to the "abandoned" list
	 nl = (NETLIST)malloc(sizeof(struct netlist_));
	 nl->net = net;
	 nl->next = Abandoned;
	 Abandoned = nl;

	 while (FailedNets && (FailedNets->net == net)) {
	    nl = FailedNets->next;
	    free(FailedNets);
	    FailedNets = nl;
	 }
	 continue;
      }

      // Find nets that collide with "net" and remove them, adding them
      // to the end of the FailedNets list.
      ripup_colliding(net);

      // Failsafe---if we have been looping enough times to exceed
      // maxtries (which is set to 8 route attempts per original failed
      // net), then we check progress.  If we have reduced the number
      // of failed nets by half or more, then we have an indication of
      // real progress, and will continue.  If not, we give up.  Qrouter
      // is almost certainly hopelessly stuck at this point.

      if (TotalRoutes >= maxtries) {
	 if (failcount <= (origcount / 2)) {
	    maxtries = TotalRoutes + failcount * 8;
	    origcount = failcount;
	 }
	 else if (keepTrying == 0) {
	    Fprintf(stderr, "\nQrouter is stuck, abandoning remaining routes.\n");
	    break;
	 }
      }
   }

   // If the list of abandoned nets is non-null, attach it to the
   // end of the failed nets list.

   if (Abandoned != NULL) {
      if (FailedNets == NULL) {
	 FailedNets = Abandoned;
	 Abandoned = NULL;
      }
      else {
	 for (nl = FailedNets; nl->next; nl = nl->next);
	 nl->next = Abandoned;
	 Abandoned = NULL;
      }
   }

   if (Verbose > 0) {
      Flush(stdout);
      Fprintf(stdout, "\n----------------------------------------------\n");
      Fprintf(stdout, "Progress: ");
      Fprintf(stdout, "Stage 2 total routes completed: %d\n", TotalRoutes);
   }
   if (FailedNets == (NETLIST)NULL) {
      failcount = 0;
      Fprintf(stdout, "No failed routes!\n");
   }
   else {
      failcount = countlist(FailedNets);
      if (FailedNets != (NETLIST)NULL)
          Fprintf(stdout, "Failed net routes: %d\n", failcount);
   }
   if (Verbose > 0)
      Fprintf(stdout, "----------------------------------------------\n");

   return failcount;
}

/*--------------------------------------------------------------*/
/* initMask() ---						*/
/*--------------------------------------------------------------*/

void initMask()
{
   RMask = (u_char *)calloc(NumChannelsX[0] * NumChannelsY[0],
			sizeof(u_char));
   if (!RMask) {
      fprintf(stderr, "Out of memory 3.\n");
      exit(3);
   }
}

/*--------------------------------------------------------------*/
/* Fill mask around the area of a vertical line			*/
/*--------------------------------------------------------------*/

void
create_vbranch_mask(int x, int y1, int y2, u_char slack, u_char halo)
{
   int gx1, gx2, gy1, gy2;
   int i, j, v;
   u_char m;

   gx1 = x - slack;
   gx2 = x + slack;
   if (y1 > y2) {
      gy1 = y2 - slack;
      gy2 = y1 + slack;
   }
   else {
      gy1 = y1 - slack;
      gy2 = y2 + slack;
   }
   if (gx1 < 0) gx1 = 0;
   if (gx2 >= NumChannelsX[0]) gx2 = NumChannelsX[0] - 1;
   if (gy1 < 0) gy1 = 0;
   if (gy2 >= NumChannelsY[0]) gy2 = NumChannelsY[0] - 1;

   for (i = gx1; i <= gx2; i++)
      for (j = gy1; j <= gy2; j++)
	 RMask[OGRID(i, j, 0)] = (u_char)0;

   for (v = 1; v < halo; v++) {
      if (gx1 > 0) gx1--;
      if (gx2 < NumChannelsX[0] - 1) gx2++;
      if (y1 > y2) {
         if (gy1 < NumChannelsY[0] - 1) gy1++;
         if (gy2 < NumChannelsY[0] - 1) gy2++;
      }
      else {
	 if (gy1 > 0) gy1--;
	 if (gy2 > 0) gy2--;
      }
      for (i = gx1; i <= gx2; i++)
         for (j = gy1; j <= gy2; j++) {
	    m = RMask[OGRID(i, j, 0)];
	    if (m > v) RMask[OGRID(i, j, 0)] = (u_char)v;
	 }
   }
}

/*--------------------------------------------------------------*/
/* Fill mask around the area of a horizontal line		*/
/*--------------------------------------------------------------*/

void
create_hbranch_mask(int y, int x1, int x2, u_char slack, u_char halo)
{
   int gx1, gx2, gy1, gy2;
   int i, j, v;
   u_char m;

   gy1 = y - slack;
   gy2 = y + slack;
   if (x1 > x2) {
      gx1 = x2 - slack;
      gx2 = x1 + slack;
   }
   else {
      gx1 = x1 - slack;
      gx2 = x2 + slack;
   }
   if (gx1 < 0) gx1 = 0;
   if (gx2 >= NumChannelsX[0]) gx2 = NumChannelsX[0] - 1;
   if (gy1 < 0) gy1 = 0;
   if (gy2 >= NumChannelsY[0]) gy2 = NumChannelsY[0] - 1;

   for (i = gx1; i <= gx2; i++)
      for (j = gy1; j <= gy2; j++)
	 RMask[OGRID(i, j, 0)] = (u_char)0;

   for (v = 1; v < halo; v++) {
      if (gy1 > 0) gy1--;
      if (gy2 < NumChannelsY[0] - 1) gy2++;
      if (x1 > x2) {
         if (gx1 < NumChannelsX[0] - 1) gx1++;
         if (gx2 < NumChannelsX[0] - 1) gx2++;
      }
      else {
	 if (gx1 > 0) gx1--;
	 if (gx2 > 0) gx2--;
      }
      for (i = gx1; i <= gx2; i++)
         for (j = gy1; j <= gy2; j++) {
	    m = RMask[OGRID(i, j, 0)];
	    if (m > v) RMask[OGRID(i, j, 0)] = (u_char)v;
	 }
   }
}

/*--------------------------------------------------------------*/
/* createBboxMask() ---						*/
/*								*/
/* Create mask limiting the area to search for routing		*/
/*								*/
/* The bounding box mask generates an area including the	*/
/* bounding box as defined in the net record, includes all pin	*/
/* positions in the mask, and increases the mask area by one	*/
/* route track for each pass, up to "halo".			*/
/*--------------------------------------------------------------*/

void createBboxMask(NET net, u_char halo)
{
    int xmin, ymin, xmax, ymax;
    int i, j, gx1, gy1, gx2, gy2;

    fillMask((u_char)halo);

    xmin = net->xmin;
    xmax = net->xmax;
    ymin = net->ymin;
    ymax = net->ymax;
  
    for (gx1 = xmin; gx1 <= xmax; gx1++)
	for (gy1 = ymin; gy1 <= ymax; gy1++)
	    RMask[OGRID(gx1, gy1, 0)] = (u_char)0;

    for (i = 1; i <= halo; i++) {
	gx1 = xmin - i;
	if (gx1 >= 0 && gx1 < NumChannelsX[0])
           for (j = ymin - i; j <= ymax + i; j++)
	      if (j >= 0 && j < NumChannelsY[0])
		 RMask[OGRID(gx1, j, 0)] = (u_char)i;

	gx2 = xmax + i;
	if (gx2 >= 0 && gx2 < NumChannelsX[0])
           for (j = ymin - i; j <= ymax + i; j++)
	      if (j >= 0 && j < NumChannelsY[0])
		 RMask[OGRID(gx2, j, 0)] = (u_char)i;

	gy1 = ymin - i;
	if (gy1 >= 0 && gy1 < NumChannelsY[0])
           for (j = xmin - i; j <= xmax + i; j++)
	      if (j >= 0 && j < NumChannelsX[0])
		 RMask[OGRID(j, gy1, 0)] = (u_char)i;

	gy2 = ymax + i;
	if (gy2 >= 0 && gy2 < NumChannelsY[0])
           for (j = xmin - i; j <= xmax + i; j++)
	      if (j >= 0 && j < NumChannelsX[0])
		 RMask[OGRID(j, gy2, 0)] = (u_char)i;
     }
}

/*--------------------------------------------------------------*/
/* analyzeCongestion() ---					*/
/*								*/
/* Given a trunk route at ycent, between ymin and ymax, score	*/
/* the neighboring positions as a function of congestion and	*/
/* offset from the ideal location.  Return the position of the	*/
/* best location for the trunk route.				*/
/*--------------------------------------------------------------*/

int analyzeCongestion(int ycent, int ymin, int ymax, int xmin, int xmax)
{
    int x, y, i, minidx, sidx, n, o;
    int *score, minscore;

    score = (int *)malloc((ymax - ymin + 1) * sizeof(int));

    for (y = ymin; y <= ymax; y++) {
	sidx = y - ymin;
	score[sidx] = ABSDIFF(ycent, y) * Num_layers;
	for (x = xmin; x <= xmax; x++) {
	    for (i = 0; i < Num_layers; i++) {
		n = Obs[i][OGRID(x, y, i)];
		if (n & ROUTED_NET) score[sidx]++;
		if (n & NO_NET) score[sidx]++;
		if (n & PINOBSTRUCTMASK) score[sidx]++;
	    }
	}
    }
    minscore = MAXRT;
    for (i = 0; i < (ymax - ymin + 1); i++) {
	if (score[i] < minscore) {
	    minscore = score[i];
	    minidx = i + ymin;
	}
    }

    free(score);
    return minidx;
}

/*--------------------------------------------------------------*/
/* createMask() ---						*/
/*								*/
/* Create mask limiting the area to search for routing		*/
/*								*/
/* For 2-node routes, find the two L-shaped routes between the	*/
/* two closest points of the nodes.				*/
/* For multi-node (>2) routes, find the best trunk line that	*/
/* passes close to all nodes, and generate stems to the closest	*/
/* point on each node.						*/
/*								*/
/* Optimizations:  (1) multi-node routes that are in a small	*/
/* enough area, just mask the bounding box.  (2) Where nodes	*/
/* at the end of two branches are closer to each other than to	*/
/* the trunk, mask an additional cross-connection between the	*/
/* two branches.						*/
/*								*/
/* Values are "halo" where there is no mask, 0 on the		*/
/* closest "slack" routes to the ideal (typically 1), and	*/
/* values increasing out to a distance of "halo" tracks away	*/
/* from the ideal.  This allows a greater search area as the	*/
/* number of passes of the search algorithm increases.		*/
/*								*/
/* To do:  Choose the position of trunk line based on		*/
/* congestion analysis.						*/
/*--------------------------------------------------------------*/

void createMask(NET net, u_char slack, u_char halo)
{
  NODE n1, n2;
  DPOINT d1tap, d2tap, dtap;
  int i, j, orient, l, v;
  int dx, dy, gx1, gx2, gy1, gy2;
  int xcent, ycent, xmin, ymin, xmax, ymax;
  int branchx, branchy;

  fillMask((u_char)halo);

  xmin = net->xmin;
  xmax = net->xmax;
  ymin = net->ymin;
  ymax = net->ymax;

  xcent = net->trunkx;
  ycent = net->trunky;

  orient = 0;

  // Construct the trunk line mask

  if (!(net->flags & NET_VERTICAL_TRUNK) || (net->numnodes == 2)) {
     // Horizontal trunk
     orient |= 1;

     ycent = analyzeCongestion(net->trunky, ymin, ymax, xmin, xmax);
     ymin = ymax = ycent;

     for (i = xmin - slack; i <= xmax + slack; i++) {
	if (i < 0 || i >= NumChannelsX[0]) continue;
	for (j = ycent - slack; j <= ycent + slack; j++) {
	   if (j < 0 || j >= NumChannelsY[0]) continue;
	   RMask[OGRID(i, j, 0)] = (u_char)0;
	}
     }

     for (i = 1; i < halo; i++) {
	gy1 = ycent - slack - i;
	gy2 = ycent + slack + i;
        for (j = xmin - slack - i; j <= xmax + slack + i; j++) {
	   if (j < 0 || j >= NumChannelsX[0]) continue;
	   if (gy1 >= 0)
	      RMask[OGRID(j, gy1, 0)] = (u_char)i;
	   if (gy2 < NumChannelsY[0])
	      RMask[OGRID(j, gy2, 0)] = (u_char)i;
	}
	gx1 = xmin - slack - i;
	gx2 = xmax + slack + i;
        for (j = ycent - slack - i; j <= ycent + slack + i; j++) {
	   if (j < 0 || j >= NumChannelsY[0]) continue;
	   if (gx1 >= 0)
	      RMask[OGRID(gx1, j, 0)] = (u_char)i;
	   if (gx2 < NumChannelsX[0])
	      RMask[OGRID(gx2, j, 0)] = (u_char)i;
	}
     }
  }
  if ((net->flags & NET_VERTICAL_TRUNK) || (net->numnodes == 2)) {
     // Vertical trunk
     orient |= 2;
     xmin = xmax = xcent;

     for (i = xcent - slack; i <= xcent + slack; i++) {
	if (i < 0 || i >= NumChannelsX[0]) continue;
	for (j = ymin - slack; j <= ymax + slack; j++) {
	   if (j < 0 || j >= NumChannelsY[0]) continue;
	   RMask[OGRID(i, j, 0)] = (u_char)0;
	}
     }

     for (i = 1; i < halo; i++) {
	gx1 = xcent - slack - i;
	gx2 = xcent + slack + i;
        for (j = ymin - slack - i; j <= ymax + slack + i; j++) {
	   if (j < 0 || j >= NumChannelsY[0]) continue;
	   if (gx1 >= 0)
	      RMask[OGRID(gx1, j, 0)] = (u_char)i;
	   if (gx2 < NumChannelsX[0])
	      RMask[OGRID(gx2, j, 0)] = (u_char)i;
	}
	gy1 = ymin - slack - i;
	gy2 = ymax + slack + i;
        for (j = xcent - slack - i; j <= xcent + slack + i; j++) {
	   if (j < 0 || j >= NumChannelsX[0]) continue;
	   if (gy1 >= 0)
	      RMask[OGRID(j, gy1, 0)] = (u_char)i;
	   if (gy2 < NumChannelsY[0])
	      RMask[OGRID(j, gy2, 0)] = (u_char)i;
	}
     }
  }
     
  // Construct the branch line masks

  for (n1 = net->netnodes; n1; n1 = n1->next) {
     dtap = (n1->taps == NULL) ? n1->extend : n1->taps;
     if (!dtap) continue;

     if (orient | 1) 	// Horizontal trunk, vertical branches
	create_vbranch_mask(n1->branchx, n1->branchy, ycent, slack, halo);
     if (orient | 2) 	// Vertical trunk, horizontal branches
	create_hbranch_mask(n1->branchy, n1->branchx, xcent, slack, halo);
  }

  // Look for branches that are closer to each other than to the
  // trunk line.  If any are found, make a cross-connection between
  // the branch end that is closer to the trunk and the branch that
  // is its nearest neighbor.

  if (orient | 1) {	// Horizontal trunk, vertical branches
     for (n1 = net->netnodes; n1; n1 = n1->next) {
	for (n2 = net->netnodes->next; n2; n2 = n2->next) {

	   // Check if both ends are on the same side of the trunk
	   if ((n2->branchy > ycent && n1->branchy > ycent) ||
		  	(n2->branchy < ycent && n1->branchy < ycent)) {

	      // Check if branches are closer to each other than
	      // the shortest branch is away from the trunk
	      dx = ABSDIFF(n2->branchx, n1->branchx);
	      gy1 = ABSDIFF(n1->branchy, ycent);
	      gy2 = ABSDIFF(n2->branchy, ycent);
	      if ((dx < gy1) && (dx < gy2)) {
		 if (gy1 < gy2)
		    create_hbranch_mask(n1->branchy, n2->branchx,
				n1->branchx, slack, halo);
		 else
		    create_hbranch_mask(n2->branchy, n2->branchx,
				n1->branchx, slack, halo);
	      }
 	   }
        }
     }
  }
  if (orient | 2) {		// Vertical trunk, horizontal branches
     for (n1 = net->netnodes; n1; n1 = n1->next) {
	for (n2 = net->netnodes->next; n2; n2 = n2->next) {

	   // Check if both ends are on the same side of the trunk
	   if ((n2->branchx > xcent && n1->branchx > xcent) ||
		  	(n2->branchx < xcent && n1->branchx < xcent)) {

	      // Check if branches are closer to each other than
	      // the shortest branch is away from the trunk
	      dy = ABSDIFF(n2->branchy, n1->branchy);
	      gx1 = ABSDIFF(n1->branchx, xcent);
	      gx2 = ABSDIFF(n2->branchx, xcent);
	      if ((dy < gx1) && (dy < gx2)) {
		 if (gx1 < gx2)
		    create_vbranch_mask(n1->branchx, n2->branchy,
				n1->branchy, slack, halo);
		 else
		    create_vbranch_mask(n2->branchx, n2->branchy,
				n1->branchy, slack, halo);
	      }
 	   }
        }
     }
  }

  // Allow routes at all tap and extension points
  for (n1 = net->netnodes; n1 != NULL; n1 = n1->next) {
     for (dtap = n1->taps; dtap != NULL; dtap = dtap->next)
	RMask[OGRID(dtap->gridx, dtap->gridy, 0)] = (u_char)0;
     for (dtap = n1->extend; dtap != NULL; dtap = dtap->next)
	RMask[OGRID(dtap->gridx, dtap->gridy, 0)] = (u_char)0;
  }

  if (Verbose > 2) {
     if (net->numnodes == 2)
        Fprintf(stdout, "Two-port mask has bounding box (%d %d) to (%d %d)\n",
			xmin, ymin, xmax, ymax);
     else
        Fprintf(stdout, "multi-port mask has trunk line (%d %d) to (%d %d)\n",
			xmin, ymin, xmax, ymax);
  }
}

/*--------------------------------------------------------------*/
/* fillMask() fills the Mask[] array with all 1s as a last	*/
/* resort, ensuring that no valid routes are missed due to a	*/
/* bad guess about the optimal route positions.			*/
/*--------------------------------------------------------------*/

void fillMask(int value) {
   int i;

   memset((void *)RMask, value,
		(size_t)(NumChannelsX[0] * NumChannelsY[0]
		* sizeof(u_char)));
}

/*--------------------------------------------------------------*/
/* doroute - basic route call					*/
/*								*/
/*	stage = 0 is normal routing				*/
/*	stage = 1 is the rip-up and reroute stage		*/
/*								*/
/*   ARGS: two nodes to be connected				*/
/*   RETURNS: 0 on success, -1 on failure			*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Fri Aug 8		*/
/*--------------------------------------------------------------*/

int doroute(NET net, u_char stage, u_char graphdebug)
{
  POINT gpoint;
  ROUTE rt1, lrt;
  NETLIST nlist;
  int result, lastlayer, unroutable;
  struct routeinfo_ iroute;

  if (!net) {
     Fprintf(stderr, "doroute():  no net to route.\n");
     return 0;
  }

  CurNet = net;				// Global, used by 2nd stage

  // Fill out route information record
  iroute.net = net;
  iroute.rt = NULL;
  iroute.glist = NULL;
  iroute.nsrc = NULL;
  iroute.nsrctap = NULL;
  iroute.maxcost = MAXRT;
  iroute.do_pwrbus = (u_char)0;
  iroute.pwrbus_src = 0;

  lastlayer = -1;

  /* Set up Obs2[] matrix for first route */

  result = route_setup(&iroute, stage);
  unroutable = result - 1;
  if (graphdebug) highlight_mask();

  // Keep going until we are unable to route to a terminal

  while (net && (result > 0)) {

     if (graphdebug) highlight_source();
     if (graphdebug) highlight_dest();
     if (graphdebug) highlight_starts(iroute.glist);

     rt1 = createemptyroute();
     rt1->netnum = net->netnum;
     iroute.rt = rt1;

     if (Verbose > 3) {
        Fprintf(stdout,"doroute(): added net %d path start %d\n", 
	       net->netnum, net->netnodes->nodenum);
     }

     result = route_segs(&iroute, stage, graphdebug);

     if (result < 0) {		// Route failure.

	// If we failed this on the last round, then stop
	// working on this net and move on to the next.
	if (FailedNets && (FailedNets->net == net)) break;

	nlist = (NETLIST)malloc(sizeof(struct netlist_));
	nlist->net = net;
	nlist->next = FailedNets;
	FailedNets = nlist;
	free(rt1);
     }
     else {

        TotalRoutes++;

        if (net->routes) {
           for (lrt = net->routes; lrt->next; lrt = lrt->next);
	   lrt->next = rt1;
        }
        else {
	   net->routes = rt1;
        }
        draw_net(net, TRUE, &lastlayer);
     }

     // For power routing, clear the list of existing pending route
     // solutions---they will not be relevant.

     if (iroute.do_pwrbus) {
        while (iroute.glist) {
           gpoint = iroute.glist;
           iroute.glist = iroute.glist->next;
           free(gpoint);
        }
     }

     /* Set up for next route and check if routing is done */
     result = next_route_setup(&iroute, stage);
  }

  /* Finished routing (or error occurred) */

  while (iroute.glist) {
     gpoint = iroute.glist;
     iroute.glist = iroute.glist->next;
     free(gpoint);
  }

  /* Route failure due to no taps or similar error---Log it */
  if ((result < 0) || (unroutable > 0)) {
     if ((FailedNets == NULL) || (FailedNets->net != net)) {
	nlist = (NETLIST)malloc(sizeof(struct netlist_));
	nlist->net = net;
	nlist->next = FailedNets;
	FailedNets = nlist;
     }
  }
  return result;
  
} /* doroute() */

/*--------------------------------------------------------------*/
/* Catch-all routine when no tap points are found.  This is a	*/
/* common problem when the technology is not set up correctly	*/
/* and it's helpful to have all these error conditions pass	*/
/* to a single subroutine.					*/
/*--------------------------------------------------------------*/

void unable_to_route(char *netname, unsigned char forced)
{
    Fprintf(stderr, "Node of net %s has no tap points---", netname);
    if (forced)
	Fprintf(stderr, "forcing a tap point.\n");
    else
	Fprintf(stderr, "unable to route!\n");
}

/*--------------------------------------------------------------*/
/* next_route_setup --						*/
/*								*/
/*--------------------------------------------------------------*/

int next_route_setup(struct routeinfo_ *iroute, u_char stage)
{
  ROUTE rt;
  NODE node;
  POINT gpoint;
  int  i, x, y;
  int  rval, result;

  if (iroute->do_pwrbus == TRUE) {

     iroute->pwrbus_src++;
     iroute->nsrc = iroute->nsrc->next;
     rval = -2;
     while (rval == -2) {
	if ((iroute->pwrbus_src > iroute->net->numnodes) || (iroute->nsrc == NULL)) {
	    result = 0;
	    break;
	}
	else {
	    result = set_powerbus_to_net(iroute->nsrc->netnum);
	    clear_target_node(iroute->nsrc);
	    rval = set_node_to_net(iroute->nsrc, PR_SOURCE, &iroute->glist,
			&iroute->bbox, stage);
	    if (rval == -2) {
		if (forceRoutable) {
		    make_routable(iroute->nsrc);
		}
		else {
		    iroute->pwrbus_src++;
		    iroute->nsrc = iroute->nsrc->next;
		}
		unable_to_route(iroute->net->netname, forceRoutable);
	    }
	    else if (rval < 0) return -1;
	}
     }
  }
  else {

     for (rt = iroute->net->routes; (rt && rt->next); rt = rt->next);

     // Set positions on last route to PR_SOURCE
     if (rt) {
	result = set_route_to_net(iroute->net, rt, PR_SOURCE, &iroute->glist,
			&iroute->bbox, stage);

        if (result == -2) {
	   unable_to_route(iroute->net->netname, 0);
           return -1;
	}
     }
     else return -1;

     result = (count_targets(iroute->net) == 0) ? 0 : 1;
  }

  // Check for the possibility that there is already a route to the target

  if (!result) {

     // Remove nodes of the net from Nodeloc so that they will not be
     // used for crossover costing of future routes.

     for (i = 0; i < Num_layers; i++) {
        for (x = 0; x < NumChannelsX[i]; x++) {
	   for (y = 0; y < NumChannelsY[i]; y++) {
	      node = Nodeloc[i][OGRID(x, y, i)];
	      if (node != (NODE)NULL)
		 if (node->netnum == iroute->net->netnum)
		    Nodeloc[i][OGRID(x, y, i)] = (NODE)NULL;
	   }
        }
     }

     while (iroute->glist) {
	gpoint = iroute->glist;
	iroute->glist = iroute->glist->next;
	free(gpoint);
     }
     return 0;
  }

  if (!iroute->do_pwrbus) {

     // If any target is found during the search, but is not the
     // target that is chosen for the minimum-cost path, then it
     // will be left marked "processed" and never visited again.
     // Make sure this doesn't happen my clearing the "processed"
     // flag from all such target nodes, and placing the positions
     // on the stack for processing again.

     clear_non_source_targets(iroute->net, &iroute->glist);
  }

  if (Verbose > 1) {
     Fprintf(stdout, "netname = %s, route number %d\n",
		iroute->net->netname, TotalRoutes );
     Flush(stdout);
  }

  if (iroute->maxcost > 2)
      iroute->maxcost >>= 1;	// Halve the maximum cost from the last run

  return 1;		// Successful setup
}

/*--------------------------------------------------------------*/
/* route_setup --						*/
/*								*/
/*--------------------------------------------------------------*/

int route_setup(struct routeinfo_ *iroute, u_char stage)
{
  POINT gpoint;
  int  i, x, y;
  u_int netnum, dir;
  int  result, rval, unroutable;
  NODE node;
  PROUTE *Pr;

  // Make Obs2[][] a copy of Obs[][].  Convert pin obstructions to
  // terminal positions for the net being routed.

  for (i = 0; i < Num_layers; i++) {
      for (x = 0; x < NumChannelsX[i]; x++) {
	  for (y = 0; y < NumChannelsY[i]; y++) {
	      netnum = Obs[i][OGRID(x, y, i)] & (~BLOCKED_MASK);
	      Pr = &Obs2[i][OGRID(x, y, i)];
	      if (netnum != 0) {
	         Pr->flags = 0;		// Clear all flags
	         Pr->prdata.net = netnum & NETNUM_MASK;
	         dir = netnum & PINOBSTRUCTMASK;
	         if ((dir != 0) && ((dir & STUBROUTE_X) == STUBROUTE_X)) {
		    if ((netnum & NETNUM_MASK) == iroute->net->netnum)
		       Pr->prdata.net = 0;	// STUBROUTE_X not routable
	         }
	      } else {
	         Pr->flags = PR_COST;		// This location is routable
	         Pr->prdata.cost = MAXRT;
	      }
	  }
      }
  }

  if (iroute->net->netnum == VDD_NET || iroute->net->netnum == GND_NET) {
     // The normal method of selecting source and target is not amenable
     // to power bus routes.  Instead, we use the global standard cell
     // power rails as the target, and each net in sequence becomes the
     // sole source node
     
     iroute->do_pwrbus = TRUE;
     iroute->nsrc = find_unrouted_node(iroute->net);
     result = (iroute->nsrc == NULL) ? 0 : 1;
  }
  else {
     iroute->do_pwrbus = FALSE;
     if (iroute->net->netnodes != NULL)
	 iroute->nsrc = iroute->net->netnodes;
     else {
	 Fprintf(stderr, "Net %s has no nodes, unable to route!\n",
			iroute->net->netname);
	 return -1;
     }
     result = 1;
  }

  // We start at the node referenced by the route structure, and flag all
  // of its taps as PR_SOURCE, as well as all connected routes.

  unroutable = 0;

  if (result) {
     iroute->bbox.x2 = iroute->bbox.y2 = 0;
     iroute->bbox.x1 = NumChannelsX[0];
     iroute->bbox.y1 = NumChannelsY[0];

     while(1) {
        rval = set_node_to_net(iroute->nsrc, PR_SOURCE, &iroute->glist,
			&iroute->bbox, stage);
	if (rval == -2) {
	   iroute->nsrc = iroute->nsrc->next;
	   if (iroute->nsrc == NULL) break;
	}
	else break;
     }
     if (rval == -2) {
        if (forceRoutable) make_routable(iroute->net->netnodes);
	unable_to_route(iroute->net->netname, forceRoutable);
        return -1;
     }

     if (iroute->do_pwrbus == FALSE) {

        // Set associated routes to PR_SOURCE
        rval = set_routes_to_net(iroute->net, PR_SOURCE, &iroute->glist,
		&iroute->bbox, stage);

        if (rval == -2) {
	   unable_to_route(iroute->net->netname, 0);
           return -1;
        }

        // Now search for all other nodes on the same net that have not
        // yet been routed, and flag all of their taps as PR_TARGET

        result = 0;
        for (node = iroute->net->netnodes; node; node = node->next) {
	   if (node == iroute->nsrc) continue;
           rval = set_node_to_net(node, PR_TARGET, NULL,
			&iroute->bbox, stage);
           if (rval == 0) {
	      result = 1;
           }
           else if (rval == -2) {
	      if (forceRoutable) make_routable(node);
	      unable_to_route(iroute->net->netname, forceRoutable);
	      if (result == 0) result = -1;
	      unroutable++;
           }
        }

        /* If there's only one node and it's not routable, then fail. */
        if (result == -1) return -1;
     }
     else {	/* Do this for power bus connections */

        /* Set all nodes that are NOT nsrc to an unused net number */
        for (node = iroute->net->netnodes; node; node = node->next) {
	   if (node != iroute->nsrc) {
	      disable_node_nets(node);
	   }
        }
        set_powerbus_to_net(iroute->nsrc->netnum);
     }
  }

  // Check for the possibility that there is already a route to the target

  if (!result) {
     // Remove nodes of the net from Nodeloc so that they will not be
     // used for crossover costing of future routes.

     for (i = 0; i < Num_layers; i++) {
        for (x = 0; x < NumChannelsX[i]; x++) {
	   for (y = 0; y < NumChannelsY[i]; y++) {
	      iroute->nsrc = Nodeloc[i][OGRID(x, y, i)];
	      if (iroute->nsrc != (NODE)NULL)
		 if (iroute->nsrc->netnum == iroute->net->netnum)
		    Nodeloc[i][OGRID(x, y, i)] = (NODE)NULL;
	   }
        }
     }

     while (iroute->glist) {
	gpoint = iroute->glist;
	iroute->glist = iroute->glist->next;
	free(gpoint);
     }
     return 0;
  }

  // Generate a search area mask representing the "likely best route".
  if ((iroute->do_pwrbus == FALSE) && (maskMode == MASK_AUTO)) {
     if (stage == 0)
	createMask(iroute->net, MASK_SMALL, (u_char)Numpasses);
     else
	createMask(iroute->net, MASK_LARGE, (u_char)Numpasses);
  }
  else if ((iroute->do_pwrbus == TRUE) || (maskMode == MASK_NONE))
     fillMask((u_char)0);
  else if (maskMode == MASK_BBOX)
     createBboxMask(iroute->net, (u_char)Numpasses);
  else
     createMask(iroute->net, maskMode, (u_char)Numpasses);

  // Heuristic:  Set the initial cost beyond which we stop searching.
  // This value is twice the cost of a direct route across the
  // maximum extent of the source to target, divided by the square
  // root of the number of nodes in the net.  We purposely set this
  // value low.  It has a severe impact on the total run time of the
  // algorithm.  If the initial max cost is so low that no route can
  // be found, it will be doubled on each pass.

  if (iroute->do_pwrbus)
     iroute->maxcost = 20;	// Maybe make this SegCost * row height?
  else {
     iroute->maxcost = 1 + 2 * MAX((iroute->bbox.x2 - iroute->bbox.x1),
		(iroute->bbox.y2 - iroute->bbox.y1))
		* SegCost + (int)stage * ConflictCost;
     iroute->maxcost /= (iroute->nsrc->numnodes - 1);
  }

  netnum = iroute->net->netnum;

  iroute->nsrctap = iroute->nsrc->taps;
  if (iroute->nsrctap == NULL) iroute->nsrctap = iroute->nsrc->extend;
  if (iroute->nsrctap == NULL) {
     unable_to_route(iroute->net->netname, 0);
     return -1;
  }

  if (Verbose > 2) {
     Fprintf(stdout, "Source node @ %gum %gum layer=%d grid=(%d %d)\n",
	  iroute->nsrctap->x, iroute->nsrctap->y, iroute->nsrctap->layer,
	  iroute->nsrctap->gridx, iroute->nsrctap->gridy);
  }

  if (Verbose > 1) {
     Fprintf(stdout, "netname = %s, route number %d\n", iroute->net->netname,
		TotalRoutes );
     Flush(stdout);
  }

  // Successful setup, although if nodes were marked unroutable,
  // this information is passed back;  routing will proceed for
  // all routable nodes and the net will be then marked as
  // abandoned.

  return (unroutable + 1);
}

/*--------------------------------------------------------------*/
/* route_segs - detailed route from node to node using onestep	*/
/*	method   						*/
/*								*/
/*   ARGS: ROUTE, ready to add segments to do route		*/
/*   RETURNS: NULL if failed, manhattan distance if success	*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Fri Aug 8		*/
/*--------------------------------------------------------------*/

int route_segs(struct routeinfo_ *iroute, u_char stage, u_char graphdebug)
{
  POINT gpoint, gunproc;
  SEG  seg;
  int  i, j, k, o;
  int  x, y;
  int  pass, maskpass;
  u_int forbid;
  GRIDP best, curpt;
  int  result, rval;
  u_char first = (u_char)1;
  u_char check_order[6];
  u_char max_reached;
  PROUTE *Pr;

  best.cost = MAXRT;
  best.x = 0;
  best.y = 0;
  best.lay = 0;
  gunproc = (POINT)NULL;
  maskpass = 0;
  
  for (pass = 0; pass < Numpasses; pass++) {

    max_reached = (u_char)0;
    if (!first && (Verbose > 2)) {
       Fprintf(stdout, "\n");
       first = (u_char)1;
    }
    if (Verbose > 2) {
       Fprintf(stdout, "Pass %d", pass + 1);
       Fprintf(stdout, " (maxcost is %d)\n", iroute->maxcost);
    }

    while (gpoint = iroute->glist) {

      iroute->glist = gpoint->next;

      curpt.x = gpoint->x1;
      curpt.y = gpoint->y1;
      curpt.lay = gpoint->layer;

      if (graphdebug) highlight(curpt.x, curpt.y);
	
      Pr = &Obs2[curpt.lay][OGRID(curpt.x, curpt.y, curpt.lay)];

      // ignore grid positions that have already been processed
      if (Pr->flags & PR_PROCESSED) {
	 free(gpoint);
	 continue;
      }

      if (Pr->flags & PR_COST)
	 curpt.cost = Pr->prdata.cost;	// Route points, including target
      else
	 curpt.cost = 0;			// For source tap points

      // if the grid position is the destination, save the position and
      // cost if minimum.

      if (Pr->flags & PR_TARGET) {

 	 if (curpt.cost < best.cost) {
	    if (first) {
	       if (Verbose > 2)
		  Fprintf(stdout, "Found a route of cost ");
	       first = (u_char)0;
	    }
	    else if (Verbose > 2) {
	       Fprintf(stdout, "|");
	       Fprintf(stdout, "%d", curpt.cost);
	       Flush(stdout);
	    }

	    // This position may be on a route, not at a terminal, so
	    // record it.
	    best.x = curpt.x;
	    best.y = curpt.y;
	    best.lay = curpt.lay;
	    best.cost = curpt.cost;

	    // If a complete route has been found, then there's no point
	    // in searching paths with a greater cost than this one.
	    if (best.cost < iroute->maxcost) iroute->maxcost = best.cost;
	 }

         // Don't continue processing from the target
	 Pr->flags |= PR_PROCESSED;
	 free(gpoint);
	 continue;
      }

      if (curpt.cost < MAXRT) {

	 // Severely limit the search space by not processing anything that
	 // is not under the current route mask, which identifies a narrow
	 // "best route" solution.

	 if (RMask[OGRID(curpt.x, curpt.y, 0)] > (u_char)maskpass) {
	    gpoint->next = gunproc;
	    gunproc = gpoint;
	    continue;
	 }

         // Quick check:  Limit maximum cost to limit search space
         // Move the point onto the "unprocessed" stack and we'll pick up
         // from this point on the next pass, if needed.

         if (curpt.cost > iroute->maxcost) {
	    max_reached = (u_char)1;
	    gpoint->next = gunproc;
	    gunproc = gpoint;
	    continue;
	 }
      }
      free(gpoint);

      // check east/west/north/south, and bottom to top

      // 1st optimization:  Direction of route on current layer is preferred.
      o = LefGetRouteOrientation(curpt.lay);
      forbid = Obs[curpt.lay][OGRID(curpt.x, curpt.y, curpt.lay)] & BLOCKED_MASK;

      if (o == 1) {			// horizontal routes---check EAST and WEST first
	 check_order[0] = (forbid & BLOCKED_E) ? 0 : EAST;
	 check_order[1] = (forbid & BLOCKED_W) ? 0 : WEST;
	 check_order[2] = UP;
	 check_order[3] = DOWN;
	 check_order[4] = (forbid & BLOCKED_N) ? 0 : NORTH;
	 check_order[5] = (forbid & BLOCKED_S) ? 0 : SOUTH;
      }
      else {				// vertical routes---check NORTH and SOUTH first
	 check_order[0] = (forbid & BLOCKED_N) ? 0 : NORTH;
	 check_order[1] = (forbid & BLOCKED_S) ? 0 : SOUTH;
	 check_order[2] = UP;
	 check_order[3] = DOWN;
	 check_order[4] = (forbid & BLOCKED_E) ? 0 : EAST;
	 check_order[5] = (forbid & BLOCKED_W) ? 0 : WEST;
      }

      // Check order is from 0 (1st priority) to 5 (last priority).  However, this
      // is a stack system, so the last one placed on the stack is the first to be
      // pulled and processed.  Therefore we evaluate and drop positions to check
      // on the stack in reverse order (5 to 0).

      for (i = 5; i >= 0; i--) {
	 switch (check_order[i]) {
	    case EAST:
               if ((curpt.x + 1) < NumChannelsX[curpt.lay]) {
         	  if ((result = eval_pt(&curpt, PR_PRED_W, stage)) == 1) {
         	     gpoint = (POINT)malloc(sizeof(struct point_));
         	     gpoint->x1 = curpt.x + 1;
         	     gpoint->y1 = curpt.y;
         	     gpoint->layer = curpt.lay;
         	     gpoint->next = iroute->glist;
         	     iroute->glist = gpoint;
                   }
               }
	       break;

	    case WEST:
               if ((curpt.x - 1) >= 0) {
         	  if ((result = eval_pt(&curpt, PR_PRED_E, stage)) == 1) {
         	     gpoint = (POINT)malloc(sizeof(struct point_));
         	     gpoint->x1 = curpt.x - 1;
         	     gpoint->y1 = curpt.y;
         	     gpoint->layer = curpt.lay;
         	     gpoint->next = iroute->glist;
         	     iroute->glist = gpoint;
                  }
               }
	       break;
         
	    case SOUTH:
               if ((curpt.y - 1) >= 0) {
         	  if ((result = eval_pt(&curpt, PR_PRED_N, stage)) == 1) {
         	     gpoint = (POINT)malloc(sizeof(struct point_));
         	     gpoint->x1 = curpt.x;
         	     gpoint->y1 = curpt.y - 1;
         	     gpoint->layer = curpt.lay;
         	     gpoint->next = iroute->glist;
         	     iroute->glist = gpoint;
                   }
               }
	       break;

	    case NORTH:
               if ((curpt.y + 1) < NumChannelsY[curpt.lay]) {
         	  if ((result = eval_pt(&curpt, PR_PRED_S, stage)) == 1) {
         	     gpoint = (POINT)malloc(sizeof(struct point_));
         	     gpoint->x1 = curpt.x;
         	     gpoint->y1 = curpt.y + 1;
         	     gpoint->layer = curpt.lay;
         	     gpoint->next = iroute->glist;
         	     iroute->glist = gpoint;
                  }
               }
	       break;
      
	    case DOWN:
               if (curpt.lay > 0) {
         	  if ((result = eval_pt(&curpt, PR_PRED_U, stage)) == 1) {
         	     gpoint = (POINT)malloc(sizeof(struct point_));
         	     gpoint->x1 = curpt.x;
         	     gpoint->y1 = curpt.y;
         	     gpoint->layer = curpt.lay - 1;
         	     gpoint->next = iroute->glist;
         	     iroute->glist = gpoint;
         	  }
               }
	       break;
         
	    case UP:
               if (curpt.lay < (Num_layers - 1)) {
         	  if ((result = eval_pt(&curpt, PR_PRED_D, stage)) == 1) {
         	     gpoint = (POINT)malloc(sizeof(struct point_));
         	     gpoint->x1 = curpt.x;
         	     gpoint->y1 = curpt.y;
         	     gpoint->layer = curpt.lay + 1;
         	     gpoint->next = iroute->glist;
         	     iroute->glist = gpoint;
         	  }
               }
	       break;
            }
         }

      // Mark this node as processed
      Pr->flags |= PR_PROCESSED;

    } // while stack is not empty

    while (iroute->glist) {
       gpoint = iroute->glist;
       iroute->glist = iroute->glist->next;
       free(gpoint);
    }

    // If we found a route, save it and return

    if (best.cost <= iroute->maxcost) {
	curpt.x = best.x;
	curpt.y = best.y;
	curpt.lay = best.lay;
	if ((rval = commit_proute(iroute->rt, &curpt, stage)) != 1) break;
	if (Verbose > 2) {
	   Fprintf(stdout, "\nCommit to a route of cost %d\n", best.cost);
	   Fprintf(stdout, "Between positions (%d %d) and (%d %d)\n",
			best.x, best.y, curpt.x, curpt.y);
	}
	goto done;	/* route success */
    }

    // Continue loop to next pass if any positions were ignored due to
    // masking or due to exceeding maxcost.

    // If the cost of the route exceeded maxcost at one or more locations,
    // then increase maximum cost for next pass.

    if (max_reached == (u_char)1) {
       iroute->maxcost <<= 1;
       // Cost overflow;  we're probably completely hosed long before this.
       if (iroute->maxcost > MAXRT) break;
    }
    else
       maskpass++;			// Increase the mask size

    if (gunproc == NULL) break;		// route failure not due to limiting
					// search to maxcost or to masking

    // Regenerate the stack of unprocessed nodes
    iroute->glist = gunproc;
    gunproc = NULL;
    
  } // pass
  
  if (!first && (Verbose > 2)) {
     Fprintf(stdout, "\n");
     Flush(stdout);
  }
  if (Verbose > 1) {
     Fprintf(stderr, "Fell through %d passes\n", pass);
  }
  if (!iroute->do_pwrbus && (Verbose > 2)) {
     Fprintf(stderr, "(%g,%g) net=%s\n",
		iroute->nsrctap->x, iroute->nsrctap->y, iroute->net->netname);
  }
  rval = -1;

done:
  
  // Regenerate the stack of unprocessed nodes
  iroute->glist = gunproc;
  gunproc = NULL;
  return rval;
  
} /* route_segs() */

/*--------------------------------------------------------------*/
/* createemptyroute - begin a ROUTE structure			*/
/*								*/
/*   ARGS: a nodes						*/
/*   RETURNS: ROUTE calloc'd and ready to begin			*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Fri Aug 8		*/
/*--------------------------------------------------------------*/

ROUTE createemptyroute()
{
  ROUTE rt;

  rt = (ROUTE)calloc(1, sizeof(struct route_));
  rt->netnum = 0;
  rt->segments = (SEG)NULL;
  rt->flags = (u_char)0;
  rt->next = (ROUTE)NULL;
  return rt;

} /* createemptyroute() */

/*--------------------------------------------------------------*/
/* cleanup_net --						*/
/*								*/
/* Special handling for layers where needblock[] is non-zero,	*/
/* and shows that two vias cannot be placed on adjacent routes. */
/* emit_routed_net() will add specialnets to merge two adjacent	*/
/* vias on the same route.  However, this cannot be used for	*/
/* adjacent vias that are each in a different route record.  It	*/
/* is easier just to find any such instances and remove them by	*/
/* eliminating one of the vias and adding a segment to connect	*/
/* the route to the neighboring via.				*/
/*--------------------------------------------------------------*/

void cleanup_net(NET net)
{
   SEG segf, segl, seg;
   ROUTE rt, rt2;
   int ls, lf, ll, layer, lastrlayer, lastlayer;
   u_char fcheck, lcheck, fixed;
   u_char xcheck, ycheck; 

   for (layer = 0; layer < Num_layers; layer++) {
      xcheck = needblock[layer] & VIABLOCKX;
      ycheck = needblock[layer] & VIABLOCKY;
      if (xcheck || ycheck) {
	 for (rt = net->routes; rt; rt = rt->next) {
	    fixed = FALSE;
	    fcheck = lcheck = FALSE;

	    /* This problem will only show up on route endpoints */
	    // segf is the first segment of the route.
	    // segl is the last segment of the route.

	    segf = rt->segments;
	    lastlayer = -1;
	    for (segl = segf->next; segl && segl->next; segl = segl->next)
		if (segl->segtype != ST_VIA) lastlayer = segl->layer;

	    // Set flag fcheck if segf needs checking, and set flag
	    // lcheck if segl needs checking.

	    if (segf->segtype & ST_VIA) {
	       lf = segf->layer;
	       fcheck = (lf != layer && lf != layer - 1) ? FALSE : TRUE;
	       // We're going to remove the contact so it can't be a tap
	       if (Nodesav[lf][OGRID(segf->x1, segf->y1, lf)] != NULL)
		  fcheck = FALSE;
	    }
	    if (segl && (segl->segtype & ST_VIA)) {
	       ll = segl->layer;
	       lcheck = (ll != layer && ll != layer - 1) ? FALSE : TRUE;
	       // We're going to remove the contact so it can't be a tap
	       if (Nodesav[ll][OGRID(segl->x1, segl->y1, ll)] != NULL)
		  lcheck = FALSE;
	    }
	    if (fcheck == FALSE && lcheck == FALSE) continue;

	    // For each route rt2 that is not rt, look at every via
	    // and see if it is adjacent to segf or segl.

	    for (rt2 = net->routes; rt2; rt2 = rt2->next) {
	       if (rt2 == rt) continue;

	       lastrlayer = -1;
	       for (seg = rt2->segments; seg; seg = seg->next) {
		  if (seg->segtype & ST_VIA) {
		     ls = seg->layer;
		     if (ls != layer && ls != layer - 1) continue;
		     if (fcheck) {
			if (ls != lf) {
			   // NOTE:  This is still an error, and we should
			   // deal with it by creating a special net.
			   continue;
			}
			if ((ABSDIFF(seg->x1, segf->x1) == 1) &&
				(seg->y1 == segf->y1) && xcheck) {
			   segf->segtype = ST_WIRE;
			   segf->x1 = seg->x1;
			   fixed = TRUE;
			   break;
			}
			else if ((ABSDIFF(seg->y1, segf->y1) == 1) &&
				(seg->x1 == segf->x1) && ycheck) {
			   segf->segtype = ST_WIRE;
			   segf->y1 = seg->y1;
			   fixed = TRUE;
			   break;
			}
		     }
		     if (lcheck) {
			if (ls != ll) {
			   // NOTE:  This is still an error, and we should
			   // deal with it by creating a special net.
			   continue;
			}
			if ((ABSDIFF(seg->x1, segl->x1) == 1) &&
				(seg->y1 == segl->y1) && xcheck) {
			   if (lastrlayer < lastlayer) {
			      seg->segtype = ST_WIRE;
			      seg->x2 = segl->x2;
			   }
			   else {
			      segl->segtype = ST_WIRE;
			      segl->x2 = seg->x2;
			   }
			   fixed = TRUE;
			   break;
			}
			else if ((ABSDIFF(seg->y1, segl->y1) == 1) &&
				(seg->x1 == segl->x1) && ycheck) {
			   if (lastrlayer < lastlayer) {
			      seg->segtype = ST_WIRE;
			      seg->y2 = segl->y2;
			   }
			   else {
			      segl->segtype = ST_WIRE;
			      segl->y2 = seg->y2;
			   }
			   fixed = TRUE;
			   break;
			}
		     }
		  }
		  else {
		     lastrlayer = seg->layer;
		  }
	       }
	       if (fixed) break;
	    }
	 }
      }
   }
}

/*--------------------------------------------------------------*/
/* emit_routed_net --						*/
/*								*/
/* Core part of emit_routes().  Dumps the DEF format for a	*/
/* complete net route to file Cmd.  If "special" is TRUE, then	*/
/* it looks only for stub routes between a grid point and an	*/
/* off-grid terminal, and dumps only the stub route geometry as	*/
/* a SPECIALNET, which takes a width parameter.  This allows	*/
/* the stub routes to be given the same width as a via, when	*/
/* the via is larger than a route width, to avoid DRC notch	*/
/* errors between the via and the terminal.  The SPECIALNETS	*/
/* are redundant;  all routing information is in the NETS	*/
/* section.  The SPECIALNETS only specify a wider route for the	*/
/* stub connection.						*/
/*--------------------------------------------------------------*/

void
emit_routed_net(FILE *Cmd, NET net, u_char special, double oscale, int iscale)
{
   SEG seg, saveseg, lastseg, prevseg;
   ROUTE rt;
   u_int dir1, dir2, tdir;
   int i, layer;
   int x, y, x2, y2;
   double dc;
   int lastx, lasty;
   int horizontal;
   DPOINT dp1, dp2;
   float offset1, offset2;
   u_char cancel;
   double invscale = (double)(1.0 / (double)iscale); 

   /* If the STUB flag is set, then we need to write out the net name	*/
   /* in the SPECIALNETS section.					*/

   if ((special == (u_char)1) && (net->flags & NET_STUB)) {
      fprintf(Cmd, ";\n- %s\n", net->netname);
   }

   int viaOffsetX[MAX_LAYERS][2];
   int viaOffsetY[MAX_LAYERS][2];

   /* Compute via offsets, if needed for adjacent vias */

   for (layer = 0; layer < Num_layers - 1; layer++) {

      dc = LefGetRouteSpacing(layer) + LefGetViaWidth(layer, layer, 1)
		- PitchY[layer] - EPS;
      viaOffsetY[layer][0] = (dc > 0.0) ? (int)(oscale * dc * 0.5) : 0;

      dc = LefGetRouteSpacing(layer) + LefGetViaWidth(layer, layer, 0)
		- PitchX[layer] - EPS;
      viaOffsetX[layer][0] = (dc > 0.0) ? (int)(oscale * dc * 0.5) : 0;

      dc = LefGetRouteSpacing(layer + 1) + LefGetViaWidth(layer, layer + 1, 1)
		- PitchY[layer + 1] - EPS;
      viaOffsetY[layer][1] = (dc > 0.0) ? (int)(oscale * dc * 0.5) : 0;

      dc = LefGetRouteSpacing(layer + 1) + LefGetViaWidth(layer, layer + 1, 0)
		- PitchX[layer + 1] - EPS;
      viaOffsetX[layer][1] = (dc > 0.0) ? (int)(oscale * dc * 0.5) : 0;
   }

   Pathon = -1;

   /* Insert routed net here */
   for (rt = net->routes; rt; rt = rt->next) {
      if (rt->segments && !(rt->flags & RT_OUTPUT)) {
	 horizontal = FALSE;
	 cancel = FALSE;

	 // Check first position for terminal offsets
	 seg = (SEG)rt->segments;
	 lastseg = saveseg = seg;
	 layer = seg->layer;
	 if (seg) {

	    // It is rare but possible to have a stub route off of an
	    // endpoint via, so check this case, and use the layer type
	    // of the via top if needed.

	    if ((seg->segtype & ST_VIA) && seg->next && (seg->next->layer <=
			seg->layer))
	       layer++;

	    dir1 = Obs[layer][OGRID(seg->x1, seg->y1, layer)];
	    dir1 &= PINOBSTRUCTMASK;
	    if (dir1 && !(seg->segtype & (ST_OFFSET_START | ST_OFFSET_END))) {
	       if ((special == (u_char)0) && (Verbose > 2))
		  Fprintf(stdout, "Stub route distance %g to terminal"
				" at %d %d (%d)\n",
				Stub[layer][OGRID(seg->x1, seg->y1, layer)],
				seg->x1, seg->y1, layer);

	       dc = Xlowerbound + (double)seg->x1 * PitchX[layer];
	       x = (int)((REPS(dc)) * oscale);
	       if (dir1 == STUBROUTE_EW)
		  dc += Stub[layer][OGRID(seg->x1, seg->y1, layer)];
	       x2 = (int)((REPS(dc)) * oscale);
	       dc = Ylowerbound + (double)seg->y1 * PitchY[layer];
	       y = (int)((REPS(dc)) * oscale);
	       if (dir1 == STUBROUTE_NS)
		  dc += Stub[layer][OGRID(seg->x1, seg->y1, layer)];
	       y2 = (int)((REPS(dc)) * oscale);
	       if (dir1 == STUBROUTE_EW) {
		  horizontal = TRUE;

		  // If the gridpoint ahead of the stub has a route
		  // on the same net, and the stub is long enough
		  // to come within a DRC spacing distance of the
		  // other route, then lengthen it to close up the
		  // distance and resolve the error.  (NOTE:  This
		  // unnecessarily stretches routes to cover taps
		  // that have not been routed to.  At least on the
		  // test standard cell set, these rules remove a
		  // handful of DRC errors and don't create any new
		  // ones.  If necessary, a flag can be added to
		  // distinguish routes from taps.

		  if ((x < x2) && (seg->x1 < (NumChannelsX[layer] - 1))) {
		     tdir = Obs[layer][OGRID(seg->x1 + 1, seg->y1, layer)];
		     if ((tdir & ~PINOBSTRUCTMASK) ==
					(net->netnum | ROUTED_NET)) {
			if (Stub[layer][OGRID(seg->x1, seg->y1, layer)] +
					LefGetRouteKeepout(layer) >= PitchX[layer]) {
		      	   dc = Xlowerbound + (double)(seg->x1 + 1)
					* PitchX[layer];
		      	   x2 = (int)((REPS(dc)) * oscale);
			}
		     }
		  }
		  else if ((x > x2) && (seg->x1 > 0)) {
		     tdir = Obs[layer][OGRID(seg->x1 - 1, seg->y1, layer)];
		     if ((tdir & ~PINOBSTRUCTMASK) ==
					(net->netnum | ROUTED_NET)) {
			if (-Stub[layer][OGRID(seg->x1, seg->y1, layer)] +
					LefGetRouteKeepout(layer) >= PitchX[layer]) {
		      	   dc = Xlowerbound + (double)(seg->x1 - 1)
					* PitchX[layer];
		      	   x2 = (int)((REPS(dc)) * oscale);
			}
		     }
		  }

		  dc = oscale * 0.5 * LefGetRouteWidth(layer);
		  if (special == (u_char)0) {
		     // Regular nets include 1/2 route width at
		     // the ends, so subtract from the stub terminus
		     if (x < x2) {
			x2 -= dc;
			if (x >= x2) cancel = TRUE;
		     }
		     else {
			x2 += dc;
			if (x <= x2) cancel = TRUE;
		     }
		  }
		  else {
		     // Special nets don't include 1/2 route width
		     // at the ends, so add to the route at the grid
		     if (x < x2)
			x -= dc;
		     else
			x += dc;

		     // Routes that extend for more than one track
		     // without a bend do not need a wide stub
		     if (seg->x1 != seg->x2) cancel = TRUE;
	  	  }
	       }
	       else {
		  horizontal = FALSE;

		  // If the gridpoint ahead of the stub has a route
		  // on the same net, and the stub is long enough
		  // to come within a DRC spacing distance of the
		  // other route, then lengthen it to close up the
		  // distance and resolve the error.

		  if ((y < y2) && (seg->y1 < (NumChannelsY[layer] - 1))) {
		     tdir = Obs[layer][OGRID(seg->x1, seg->y1 + 1, layer)];
		     if ((tdir & ~PINOBSTRUCTMASK) ==
						(net->netnum | ROUTED_NET)) {
			if (Stub[layer][OGRID(seg->x1, seg->y1, layer)] +
					LefGetRouteKeepout(layer) >= PitchY[layer]) {
		      	   dc = Ylowerbound + (double)(seg->y1 + 1)
					* PitchY[layer];
		      	   y2 = (int)((REPS(dc)) * oscale);
			}
		     }
		  }
		  else if ((y > y2) && (seg->y1 > 0)) {
		     tdir = Obs[layer][OGRID(seg->x1, seg->y1 - 1, layer)];
		     if ((tdir & ~PINOBSTRUCTMASK) ==
						(net->netnum | ROUTED_NET)) {
			if (-Stub[layer][OGRID(seg->x1, seg->y1, layer)] +
					LefGetRouteKeepout(layer) >= PitchY[layer]) {
		      	   dc = Ylowerbound + (double)(seg->y1 - 1)
					* PitchY[layer];
		      	   y2 = (int)((REPS(dc)) * oscale);
			}
		     }
		  }

		  dc = oscale * 0.5 * LefGetRouteWidth(layer);
		  if (special == (u_char)0) {
		     // Regular nets include 1/2 route width at
		     // the ends, so subtract from the stub terminus
		     if (y < y2) {
			y2 -= dc;
			if (y >= y2) cancel = TRUE;
		     }
		     else {
			y2 += dc;
			if (y <= y2) cancel = TRUE;
		     }
		  }
		  else {
		     // Special nets don't include 1/2 route width
		     // at the ends, so add to the route at the grid
		     if (y < y2)
			y -= dc;
		     else
			y += dc;

		     // Routes that extend for more than one track
		     // without a bend do not need a wide stub
		     if (seg->y1 != seg->y2) cancel = TRUE;
		  }
	       }

	       if (cancel == FALSE) {
		  net->flags |= NET_STUB;
		  rt->flags |= RT_STUB;
		  pathstart(Cmd, layer, x2, y2, special, oscale, invscale, horizontal);
		  pathto(Cmd, x, y, horizontal, x2, y2, invscale);
	       }
	       lastx = x;
	       lasty = y;
	    }
	 }

	 prevseg = NULL;
	 lastseg = NULL;
	 for (seg = rt->segments; seg; seg = seg->next) {
	    layer = seg->layer;

	    // Check for offset terminals at either point

	    offset1 = 0.0;
	    offset2 = 0.0;
	    dir1 = 0;
	    dir2 = 0;

	    if (seg->segtype & ST_OFFSET_START) {
	       dir1 = Obs[seg->layer][OGRID(seg->x1, seg->y1, seg->layer)] &
				PINOBSTRUCTMASK;
	       if (dir1 == 0 && lastseg) {
		  dir1 = Obs[lastseg->layer][OGRID(lastseg->x2, lastseg->y2,
					lastseg->layer)] & PINOBSTRUCTMASK;
		  offset1 = Stub[lastseg->layer][OGRID(lastseg->x2,
					lastseg->y2, lastseg->layer)];
	       }
	       else
		  offset1 = Stub[seg->layer][OGRID(seg->x1, seg->y1, seg->layer)];

	       // Offset was calculated for vias;  plain metal routes
	       // typically will need less offset distance, so subtract off
	       // the difference.

	       if (!(seg->segtype & ST_VIA)) {
		  if (offset1 < 0) {
		     offset1 += 0.5 * (LefGetViaWidth(seg->layer, seg->layer, 
				horizontal) - LefGetRouteWidth(seg->layer));
		     if (offset1 > 0) offset1 = 0;
		  }
		  else if (offset1 > 0) {
		     offset1 -= 0.5 * (LefGetViaWidth(seg->layer, seg->layer,
				horizontal) - LefGetRouteWidth(seg->layer));
		     if (offset1 < 0) offset1 = 0;
		  }
	       }

	       if (special == (u_char)0) {
		  if ((seg->segtype & ST_VIA) && (Verbose > 2))
		     Fprintf(stdout, "Offset terminal distance %g to grid"
					" at %d %d (%d)\n", offset1,
					seg->x1, seg->y1, layer);
	       }
	    }
	    if (seg->segtype & ST_OFFSET_END) {
	       dir2 = Obs[seg->layer][OGRID(seg->x2, seg->y2, seg->layer)] &
				PINOBSTRUCTMASK;
	       if (dir2 == 0 && seg->next) {
		  dir2 = Obs[seg->next->layer][OGRID(seg->next->x1,
					seg->next->y1, seg->next->layer)] &
					PINOBSTRUCTMASK;
		  offset2 = Stub[seg->next->layer][OGRID(seg->next->x1,
					seg->next->y1, seg->next->layer)];
	       }
	       else
		  offset2 = Stub[seg->layer][OGRID(seg->x2, seg->y2, seg->layer)];

	       // Offset was calculated for vias;  plain metal routes
	       // typically will need less offset distance, so subtract off
	       // the difference.

	       if (!(seg->segtype & ST_VIA)) {
		  if (offset2 < 0) {
		     offset2 += 0.5 * (LefGetViaWidth(seg->layer, seg->layer,
				horizontal) - LefGetRouteWidth(seg->layer));
		     if (offset2 > 0) offset2 = 0;
		  }
		  else if (offset2 > 0) {
		     offset2 -= 0.5 * (LefGetViaWidth(seg->layer, seg->layer, 
				horizontal) - LefGetRouteWidth(seg->layer));
		     if (offset2 < 0) offset2 = 0;
		  }
	       }

	       if (special == (u_char)0) {
		  if ((seg->segtype & ST_VIA)
					&& !(seg->segtype & ST_OFFSET_START))
		     if (Verbose > 2)
		        Fprintf(stdout, "Offset terminal distance %g to grid"
					" at %d %d (%d)\n", offset2,
					seg->x2, seg->y2, layer);
	       }
	    }

	    // To do: pick up route layer name from lefInfo.
	    // At the moment, technology names don't even match,
	    // and are redundant between CIFLayer[] from the
	    // config file and lefInfo.

	    dc = Xlowerbound + (double)seg->x1 * PitchX[layer];
	    if (dir1 == (STUBROUTE_EW | OFFSET_TAP)) dc += offset1;
	    x = (int)((REPS(dc)) * oscale);
	    dc = Ylowerbound + (double)seg->y1 * PitchY[layer];
	    if (dir1 == (STUBROUTE_NS | OFFSET_TAP)) dc += offset1;
	    y = (int)((REPS(dc)) * oscale);
	    dc = Xlowerbound + (double)seg->x2 * PitchX[layer];
	    if (dir2 == (STUBROUTE_EW | OFFSET_TAP)) dc += offset2;
	    x2 = (int)((REPS(dc)) * oscale);
	    dc = Ylowerbound + (double)seg->y2 * PitchY[layer];
	    if (dir2 == (STUBROUTE_NS | OFFSET_TAP)) dc += offset2;
	    y2 = (int)((REPS(dc)) * oscale);
	    switch (seg->segtype & ~(ST_OFFSET_START | ST_OFFSET_END)) {
	       case ST_WIRE:
		  if (Pathon != 1) {	// 1st point of route seg
		     if (x == x2) {
			horizontal = FALSE;
		     }
		     else if (y == y2) {
			horizontal = TRUE;
		     }
		     else if (Verbose > 3) {
			// NOTE:  This is a development diagnostic.  The
			// occasional non-Manhanhattan route is due to a
			// tap offset and is corrected automatically by
			// making an L-bend in the wire.

		     	Flush(stdout);
			Fprintf(stderr, "Warning:  non-Manhattan wire in route"
				" at (%d %d) to (%d %d)\n", x, y, x2, y2);
		     }
		     if (special == (u_char)0) {
			pathstart(Cmd, seg->layer, x, y, (u_char)0, oscale, invscale,
				horizontal);
			lastx = x;
			lasty = y;
		     }
		  }
		  rt->flags |= RT_OUTPUT;
		  if (horizontal && x == x2) {
		     horizontal = FALSE;
		  }
		  if ((!horizontal) && y == y2) {
		     horizontal = TRUE;
		  }
		  if (!(x == x2) && !(y == y2)) {
		     horizontal = FALSE;
		  }
		  if (special == (u_char)0) {
		     pathto(Cmd, x2, y2, horizontal, lastx, lasty, invscale);
		     lastx = x2;
		     lasty = y2;
		  }

		  // If a segment is 1 track long, there is a via on either
		  // end, and the needblock flag is set for the layer, then
		  // draw a stub route along the length of the track.

		  if (horizontal && needblock[seg->layer] & VIABLOCKX) {
		     if (ABSDIFF(seg->x2, seg->x1) == 1) {
			if ((lastseg && lastseg->segtype == ST_VIA) ||
			    (seg->next && seg->next->segtype == ST_VIA)) {
			   if (special == (u_char)0) {
			      net->flags |= NET_STUB;
			      rt->flags |= RT_STUB;
			   }
			   else {
			      if (Pathon != -1) Pathon = 0;
			      pathstart(Cmd, layer, x, y, special, oscale,
						invscale, horizontal);
			      pathto(Cmd, x2, y2, horizontal, x, y, invscale);
			   }
			}
		     }
		  }
		  else if (!horizontal && needblock[seg->layer] & VIABLOCKY) {
		     if (ABSDIFF(seg->y2, seg->y1) == 1)  {
			if ((lastseg && lastseg->segtype == ST_VIA) ||
			    (seg->next && seg->next->segtype == ST_VIA)) {
			   if (special == (u_char)0) {
			      net->flags |= NET_STUB;
			      rt->flags |= RT_STUB;
			   }
			   else {
			      if (Pathon != -1) Pathon = 0;
			      pathstart(Cmd, layer, x, y, special, oscale,
						invscale, horizontal);
			      pathto(Cmd, x2, y2, horizontal, x, y, invscale);
			   }
			}
		     }
		  }
		  break;
	       case ST_VIA:
		  rt->flags |= RT_OUTPUT;
		  if (special == (u_char)0) {
		     if (lastseg == NULL) {
			// Make sure last position is valid
			lastx = x;
			lasty = y;
		     }
		     // Check for vias between adjacent but different nets
		     // that need position offsets to avoid a DRC spacing error

		     if (viaOffsetX[layer][0] > 0) {
			if (seg->x1 > 0 && ((tdir = (Obs[layer][OGRID(seg->x1 - 1,
				seg->y1, layer)] & ~PINOBSTRUCTMASK)) != 
				(net->netnum | ROUTED_NET)) &&
				((tdir & (ROUTED_NET | NO_NET) == ROUTED_NET))) {
			   pathvia(Cmd, layer, x + viaOffsetX[layer][0],
					y, lastx, lasty, seg->x1, seg->y1, invscale);
			}
			else if ((seg->x1 < NumChannelsX[layer] - 1)
				&& ((tdir = (Obs[layer][OGRID(seg->x1 + 1, seg->y1,
				layer)] & ~PINOBSTRUCTMASK)) != 
				(net->netnum | ROUTED_NET)) &&
				((tdir & (ROUTED_NET | NO_NET) == ROUTED_NET))) {
			   pathvia(Cmd, layer, x - viaOffsetX[layer][0],
					y, lastx, lasty, seg->x1, seg->y1, invscale);
			}
			else
			    pathvia(Cmd, layer, x, y, lastx, lasty, seg->x1,
					seg->y1, invscale);
		     }
		     else if (viaOffsetY[layer][0] > 0) {
			if (seg->y1 > 0 && ((tdir = (Obs[layer][OGRID(seg->x1,
				seg->y1 - 1, layer)] & ~PINOBSTRUCTMASK)) != 
				(net->netnum | ROUTED_NET)) &&
				((tdir & (ROUTED_NET | NO_NET) == ROUTED_NET))) {
			   pathvia(Cmd, layer, x, y - viaOffsetY[layer][0],
					lastx, lasty, seg->x1, seg->y1, invscale);
			}
			else if ((seg->y1 < NumChannelsY[layer] - 1)
				&& ((tdir = (Obs[layer][OGRID(seg->x1, seg->y1 + 1,
				layer)] & ~PINOBSTRUCTMASK)) != 
				(net->netnum | ROUTED_NET)) &&
				((tdir & (ROUTED_NET | NO_NET) == ROUTED_NET))) {
			   pathvia(Cmd, layer, x, y - viaOffsetY[layer][0],
					lastx, lasty, seg->x1, seg->y1, invscale);
			}
			else
			    pathvia(Cmd, layer, x, y, lastx, lasty, seg->x1,
					seg->y1, invscale);
		     }
		     else if (viaOffsetX[layer][1] > 0) {
			if (seg->x1 > 0 && ((tdir = (Obs[layer + 1][OGRID(seg->x1 - 1,
				seg->y1, layer + 1)] & ~PINOBSTRUCTMASK)) != 
				(net->netnum | ROUTED_NET)) &&
				((tdir & (ROUTED_NET | NO_NET) == ROUTED_NET))) {
			   pathvia(Cmd, layer, x + viaOffsetX[layer][1],
					y, lastx, lasty, seg->x1, seg->y1, invscale);
			}
			else if ((seg->x1 < NumChannelsX[layer + 1] - 1)
				&& ((tdir = (Obs[layer + 1][OGRID(seg->x1 + 1, seg->y1,
				layer + 1)] & ~PINOBSTRUCTMASK)) != 
				(net->netnum | ROUTED_NET)) &&
				((tdir & (ROUTED_NET | NO_NET) == ROUTED_NET))) {
			   pathvia(Cmd, layer, x - viaOffsetX[layer][1],
					y, lastx, lasty, seg->x1, seg->y1, invscale);
			}
			else
			    pathvia(Cmd, layer, x, y, lastx, lasty, seg->x1,
					seg->y1, invscale);
		     }
		     else if (viaOffsetY[layer][1] > 0) {
			if (seg->y1 > 0 && ((tdir = (Obs[layer + 1][OGRID(seg->x1,
				seg->y1 - 1, layer + 1)] & ~PINOBSTRUCTMASK)) != 
				(net->netnum | ROUTED_NET)) &&
				((tdir & (ROUTED_NET | NO_NET) == ROUTED_NET))) {
			   pathvia(Cmd, layer, x, y - viaOffsetY[layer][1],
					lastx, lasty, seg->x1, seg->y1, invscale);
			}
			else if ((seg->y1 < NumChannelsY[layer + 1] - 1)
				&& ((tdir = (Obs[layer][OGRID(seg->x1, seg->y1 + 1,
				layer + 1)] & ~PINOBSTRUCTMASK)) != 
				(net->netnum | ROUTED_NET)) &&
				((tdir & (ROUTED_NET | NO_NET) == ROUTED_NET))) {
			   pathvia(Cmd, layer, x, y - viaOffsetY[layer][1],
					lastx, lasty, seg->x1, seg->y1, invscale);
			}
			else
			    pathvia(Cmd, layer, x, y, lastx, lasty, seg->x1,
					seg->y1, invscale);
		     }
		     else
			pathvia(Cmd, layer, x, y, lastx, lasty, seg->x1,
					seg->y1, invscale);

		     lastx = x;
		     lasty = y;
		  }
		  break;
	       default:
		  break;
	    }

	    // Break here on last segment so that seg and lastseg are valid
	    // in the following section of code.

	    if (seg->next == NULL) break;
	    prevseg = lastseg;
	    lastseg = seg;
	 }

	 // For stub routes, reset the path between terminals, since
	 // the stubs are not connected.
	 if (special == (u_char)1 && Pathon != -1) Pathon = 0;

	 // Check last position for terminal offsets
	 if (seg && ((seg != saveseg)
				|| (seg->segtype & ST_WIRE))) {
	     cancel = FALSE;
	     layer = seg->layer;
	     dir2 = Obs[layer][OGRID(seg->x2, seg->y2, layer)];
	     dir2 &= PINOBSTRUCTMASK;
	     if (dir2 && !(seg->segtype & (ST_OFFSET_END | ST_OFFSET_START))) {
		if ((special == (u_char)0) && (Verbose > 2))
		   Fprintf(stdout, "Stub route distance %g to terminal"
				" at %d %d (%d)\n",
				Stub[layer][OGRID(seg->x2, seg->y2, layer)],
				seg->x2, seg->y2, layer);

		dc = Xlowerbound + (double)seg->x2 * PitchX[layer];
		x = (int)((REPS(dc)) * oscale);
		if (dir2 == STUBROUTE_EW)
		   dc += Stub[layer][OGRID(seg->x2, seg->y2, layer)];
		x2 = (int)((REPS(dc)) * oscale);
		dc = Ylowerbound + (double)seg->y2 * PitchY[layer];
		y = (int)((REPS(dc)) * oscale);
		if (dir2 == STUBROUTE_NS)
		   dc += Stub[layer][OGRID(seg->x2, seg->y2, layer)];
		y2 = (int)((REPS(dc)) * oscale);
		if (dir2 == STUBROUTE_EW) {
		   horizontal = TRUE;

		   // If the gridpoint ahead of the stub has a route
		   // on the same net, and the stub is long enough
		   // to come within a DRC spacing distance of the
		   // other route, then lengthen it to close up the
		   // distance and resolve the error.

		   if ((x < x2) && (seg->x2 < (NumChannelsX[layer] - 1))) {
		      tdir = Obs[layer][OGRID(seg->x2 + 1, seg->y2, layer)];
		      if ((tdir & ~PINOBSTRUCTMASK) ==
						(net->netnum | ROUTED_NET)) {
			 if (Stub[layer][OGRID(seg->x2, seg->y2, layer)] +
					LefGetRouteKeepout(layer) >= PitchX[layer]) {
		      	    dc = Xlowerbound + (double)(seg->x2 + 1)
					* PitchX[layer];
		      	    x2 = (int)((REPS(dc)) * oscale);
			 }
		      }
		   }
		   else if ((x > x2) && (seg->x2 > 0)) {
		      tdir = Obs[layer][OGRID(seg->x2 - 1, seg->y2, layer)];
		      if ((tdir & ~PINOBSTRUCTMASK) ==
						(net->netnum | ROUTED_NET)) {
			 if (-Stub[layer][OGRID(seg->x2, seg->y2, layer)] +
					LefGetRouteKeepout(layer) >= PitchX[layer]) {
		      	    dc = Xlowerbound + (double)(seg->x2 - 1)
					* PitchX[layer];
		      	    x2 = (int)((REPS(dc)) * oscale);
			 }
		      }
		   }

		   dc = oscale * 0.5 * LefGetRouteWidth(layer);
		   if (special == (u_char)0) {
		      // Regular nets include 1/2 route width at
		      // the ends, so subtract from the stub terminus
		      if (x < x2) {
			 x2 -= dc;
			 if (x >= x2) cancel = TRUE;
		      }
		      else {
			 x2 += dc;
			 if (x <= x2) cancel = TRUE;
		      }
		   }
		   else {
		      // Special nets don't include 1/2 route width
		      // at the ends, so add to the route at the grid
		      if (x < x2)
			 x -= dc;
		      else
			 x += dc;

		      // Routes that extend for more than one track
		      // without a bend do not need a wide stub
		      if (seg->x1 != seg->x2) cancel = TRUE;
		   }
		}
		else {
		   horizontal = FALSE;

		   // If the gridpoint ahead of the stub has a route
		   // on the same net, and the stub is long enough
		   // to come within a DRC spacing distance of the
		   // other route, then lengthen it to close up the
		   // distance and resolve the error.

		   if ((y < y2) && (seg->y2 < (NumChannelsY[layer] - 1))) {
		      tdir = Obs[layer][OGRID(seg->x2, seg->y2 + 1, layer)];
		      if ((tdir & ~PINOBSTRUCTMASK) ==
						(net->netnum | ROUTED_NET)) {
			 if (Stub[layer][OGRID(seg->x2, seg->y2, layer)] +
					LefGetRouteKeepout(layer) >= PitchY[layer]) {
		      	    dc = Ylowerbound + (double)(seg->y2 + 1)
					* PitchY[layer];
		      	    y2 = (int)((REPS(dc)) * oscale);
			 }
		      }
		   }
		   else if ((y > y2) && (seg->y2 > 0)) {
		      tdir = Obs[layer][OGRID(seg->x2, seg->y2 - 1, layer)];
		      if ((tdir & ~PINOBSTRUCTMASK) ==
						(net->netnum | ROUTED_NET)) {
			 if (-Stub[layer][OGRID(seg->x2, seg->y2, layer)] +
					LefGetRouteKeepout(layer) >= PitchY[layer]) {
		      	    dc = Ylowerbound + (double)(seg->y2 - 1)
					* PitchY[layer];
		      	    y2 = (int)((REPS(dc)) * oscale);
			 }
		      }
		   }

		   dc = oscale * 0.5 * LefGetRouteWidth(layer);
		   if (special == (u_char)0) {
		      // Regular nets include 1/2 route width at
		      // the ends, so subtract from the stub terminus
		      if (y < y2) {
			 y2 -= dc;
			 if (y >= y2) cancel = TRUE;
		      }
		      else {
			 y2 += dc;
			 if (y <= y2) cancel = TRUE;
		      }
		   }
		   else {
		      // Special nets don't include 1/2 route width
		      // at the ends, so add to the route at the grid
		      if (y < y2)
			 y -= dc;
		      else
			 y += dc;

		      // Routes that extend for more than one track
		      // without a bend do not need a wide stub
		      if (seg->y1 != seg->y2) cancel = TRUE;
		   }
		}
		if (cancel == FALSE) {
	           net->flags |= NET_STUB;
	           rt->flags |= RT_STUB;
		   if (Pathon != 1) {
		      pathstart(Cmd, layer, x, y, special, oscale, invscale,
				horizontal);
		      lastx = x;
		      lasty = y;
		   }
		   pathto(Cmd, x2, y2, horizontal, lastx, lasty, invscale);
		   lastx = x2;
		   lasty = y2;
		}
	    }
	    else if (dir2 && (seg->segtype & ST_VIA) && prevseg) {
	       // (seg->segtype & (ST_OFFSET_END | ST_OFFSET_START)) != 0
	       // is implied.
	       //
	       // Additional handling for offset taps.  When a tap position
	       // is a via and is offset in the direction of the last
	       // route segment, then a DRC violation can be created if
	       // (1) the via is wider than the route width, and (2) the
	       // adjacent track position is another via or a bend in the
	       // route, and (3) the tap offset is large enough to create
	       // a spacing violation between the via and the adjacent via
	       // or perpendicular route.  If these three conditions are
	       // satisfied, then generate a stub route the width of the
	       // via and one track pitch in length back toward the last
	       // track position.

	       // Problems only arise when the via width is larger than
	       // the width of the metal route leaving the via.

	       if (LefGetViaWidth(seg->layer, lastseg->layer, 1 - horizontal) >
			LefGetRouteWidth(lastseg->layer)) {

		  // Problems only arise when the last segment is exactly
		  // one track long.

		  if ((ABSDIFF(lastseg->x2, lastseg->x1) == 1) ||
			(ABSDIFF(lastseg->y2, lastseg->y1) == 1)) {

		     if (prevseg->segtype & ST_VIA) {

			dc = Xlowerbound + (double)seg->x1 * PitchX[layer];
			x = (int)((REPS(dc)) * oscale);
			dc = Ylowerbound + (double)seg->y1 * PitchY[layer];
			y = (int)((REPS(dc)) * oscale);

			dc = Xlowerbound + (double)prevseg->x1 * PitchX[layer];
			x2 = (int)((REPS(dc)) * oscale);
			dc = Ylowerbound + (double)prevseg->y1 * PitchY[layer];
			y2 = (int)((REPS(dc)) * oscale);

			// Setup is (via, 1 track route, via with offset)

			if (prevseg->x1 != seg->x1) {
			   if ((PitchX[lastseg->layer] -
				0.5 * LefGetViaWidth(seg->layer, lastseg->layer, 1) -
				0.5 * LefGetViaWidth(prevseg->layer, lastseg->layer, 1) -
				(prevseg->x1 - seg->x1) *
				Stub[seg->layer][OGRID(seg->x2, seg->y2, layer)])
				< LefGetRouteSpacing(lastseg->layer)) {
			      if (special == (u_char)0) {
				 rt->flags |= RT_STUB;
				 net->flags |= NET_STUB;
			      }
			      else {
				 pathstart(Cmd, lastseg->layer, x, y,
					(u_char)1, oscale, invscale, 1);
				 pathto(Cmd, x2, y2, 1, x, y, invscale);
			      }
			   }
			}
			else if (prevseg->y1 != seg->y1) {
			   if ((PitchY[lastseg->layer] -
				0.5 * LefGetViaWidth(seg->layer, lastseg->layer, 0) -
				0.5 * LefGetViaWidth(prevseg->layer, lastseg->layer, 0)
				- (prevseg->y1 - seg->y1) *
				Stub[seg->layer][OGRID(seg->x2, seg->y2, layer)])
				< LefGetRouteSpacing(lastseg->layer)) {
			      if (special == (u_char)0) {
				 rt->flags |= RT_STUB;
				 net->flags |= NET_STUB;
			      }
			      else {
				 pathstart(Cmd, lastseg->layer, x, y,
					(u_char)1, oscale, invscale, 0);
				 pathto(Cmd, x2, y2, 0, x, y, invscale);
			      }
			   }
			}
		     }
		     else {	// Metal route bends at next track
			if (prevseg->x1 != seg->x1) {
			   if ((PitchX[lastseg->layer] -
				0.5 * LefGetViaWidth(seg->layer, lastseg->layer, 1) -
				0.5 * LefGetRouteWidth(prevseg->layer) -
				(prevseg->x1 - seg->x1) *
				Stub[seg->layer][OGRID(seg->x2, seg->y2, layer)])
				< LefGetRouteSpacing(lastseg->layer)) {
			      if (special == (u_char)0) {
				 rt->flags |= RT_STUB;
				 net->flags |= NET_STUB;
			      }
			      else {
				 pathstart(Cmd, lastseg->layer, x, y,
					(u_char)1, oscale, invscale, 1);
				 pathto(Cmd, x2, y2, 1, x, y, invscale);
			      }
			   }
			}
			else if (prevseg->y1 != seg->y1) {
			   if ((PitchY[lastseg->layer] -
				0.5 * LefGetViaWidth(seg->layer, lastseg->layer, 0) -
				0.5 * LefGetRouteWidth(prevseg->layer) -
				(prevseg->y1 - seg->y1) *
				Stub[seg->layer][OGRID(seg->x2, seg->y2, layer)])
				< LefGetRouteSpacing(lastseg->layer)) {
			      if (special == (u_char)0) {
				 rt->flags |= RT_STUB;
				 net->flags |= NET_STUB;
			      }
			      else {
				 pathstart(Cmd, lastseg->layer, x, y,
					(u_char)1, oscale, invscale, 0);
				 pathto(Cmd, x2, y2, 0, x, y, invscale);
			      }
			   }
			}
		     }
		  }
	       }
	    }
	 }
	 if (Pathon != -1) Pathon = 0;

      } // if (rt->segments && !(rt->flags & RT_OUTPUT))
   }
}

/*--------------------------------------------------------------*/
/* emit_routes - DEF file output from the list of routes	*/
/*								*/
/*  Reads the <project>.def file and rewrites file		*/
/*  <project>_route.def, where each net definition has the	*/
/*  physical route appended.					*/
/*								*/
/*   ARGS: filename to list to					*/
/*   RETURNS: nothing						*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Mon Aug 11 2003		*/
/*--------------------------------------------------------------*/

void emit_routes(char *filename, double oscale, int iscale)
{
    FILE *Cmd;
    int i, j, numnets, stubroutes;
    char line[MAX_LINE_LEN + 1], *lptr;
    char netname[MAX_NAME_LEN];
    NET net;
    ROUTE rt;
    char newDEFfile[256];
    FILE *fdef;
    u_char errcond = FALSE;

    fdef = fopen(filename, "r");
    if (fdef == NULL) {
	if (strchr(filename, '.') == NULL) {
	    char *extfilename = malloc(strlen(filename) + 5);
	    sprintf(extfilename, "%s.def", filename);
	    fdef = fopen(extfilename, "r");
	    free(extfilename);
	}
    }
    if (fdef == NULL) {
	Fprintf(stderr, "emit_routes(): Cannot open DEF file for reading.\n");
	return;
    } 

    if (!strcmp(filename, "stdout")) {
	Cmd = stdout;
    }
    else {
	char *dotptr;

	if (filename == DEFfilename) {
	    strcpy(newDEFfile, filename);
	    dotptr = strrchr(newDEFfile, '.');
	    if (dotptr)
		strcpy(dotptr, "_route.def");
	    else
		strcat(newDEFfile, "_route.def");
	    
	    Cmd = fopen(newDEFfile, "w");
	}
	else
	    Cmd = fopen(filename, "w");
    }
    if (!Cmd) {
	Fprintf(stderr, "emit_routes():  Couldn't open output (routed) DEF file.\n");
	return;
    }

    // Copy DEF file up to NETS line
    numnets = 0;
    while (fgets(line, MAX_LINE_LEN, fdef) != NULL) {
       lptr = line;
       while (isspace(*lptr)) lptr++;
       if (!strncmp(lptr, "NETS", 4)) {
	  sscanf(lptr + 4, "%d", &numnets);
	  break;
       }
       fputs(line, Cmd);
    }
    fputs(line, Cmd);	// Write the NETS line

    // NOTE:  May want to remove this message.  It may merely reflect
    // that the DEF file defined one or more SPECIALNETS.

    if (numnets != Numnets) {
      	Flush(stdout);
	Fprintf(stderr, "emit_routes():  DEF file has %d nets, but we want"
		" to write %d\n", numnets, Numnets);
    }

    for (i = 0; i < numnets; i++) {
       if (errcond == TRUE) break;
       netname[0] == '\0';
       while (fgets(line, MAX_LINE_LEN, fdef) != NULL) {
	  if ((lptr = strchr(line, ';')) != NULL) {
	     *lptr = '\n';
	     *(lptr + 1) = '\0';
	     break;
	  }
	  else {
             lptr = line;
             while (isspace(*lptr)) lptr++;
	     if (*lptr == '-') {
		lptr++;
                while (isspace(*lptr)) lptr++;
	        sscanf(lptr, "%s", netname);
		fputs(line, Cmd);
	     }
	     else if (*lptr == '+') {
		lptr++;
                while (isspace(*lptr)) lptr++;
		if (!strncmp(lptr, "ROUTED", 6)) {
		   // This net is being handled by qrouter, so remove
		   // the original routing information
		   while (fgets(line, MAX_LINE_LEN, fdef) != NULL) {
		      if ((lptr = strchr(line, ';')) != NULL) {
			 *lptr = '\n';
			 *(lptr + 1) = '\0';
			 break;
		      }
		   }
		   break;
		}
		else
		   fputs(line, Cmd);
	     }
	     else if (!strncmp(lptr, "END", 3)) {	// This should not happen
		fputs(line, Cmd);
		errcond = TRUE;
		break;
	     }
	     else
		fputs(line, Cmd);
	  }
       }

       /* Find this net */

       for (j = 0; j < Numnets; j++) {
          net = Nlnets[j];
	  if (!strcmp(net->netname, netname))
	     break;
       }
       if (!net) {
	  Fprintf(stderr, "emit_routes():  Net %s cannot be found.\n",
		netname);

	  /* Dump rest of net and continue---no routing information */
	  *(lptr) = ';';
	  fputs(line, Cmd);
	  continue;
       }
       else {
	  /* Add last net terminal, without the semicolon */
	  fputs(line, Cmd);

	  cleanup_net(net);
	  emit_routed_net(Cmd, net, (u_char)0, oscale, iscale);
	  fprintf(Cmd, ";\n");
       }
    }

    // Finish copying the rest of the NETS section
    if (errcond == FALSE) {
       while (fgets(line, MAX_LINE_LEN, fdef) != NULL) {
	  lptr = line;
	  while (isspace(*lptr)) lptr++;
	  fputs(line, Cmd);
	  if (!strncmp(lptr, "END", 3)) {
	     break;
	  }
       }
    }

    // Determine how many stub routes we will write to SPECIALNETS
    // Also reset the OUTPUT flag for each route needing a stubroute
    // to be written.

    stubroutes = 0;
    for (i = 0; i < Numnets; i++) {
	net = Nlnets[i];
	if (net->flags & NET_STUB) {
	    stubroutes++;
	    for (rt = net->routes; rt; rt = rt->next)
		if (rt->flags & RT_STUB)
		    rt->flags &= ~RT_OUTPUT;
	}
    }

    // If there were stub routes, repeat them in SPECIALNETS at the
    // proper width.
    if (stubroutes > 0) {

        fprintf(Cmd, "\nSPECIALNETS %d ", stubroutes);
	for (i = 0; i < Numnets; i++) {
	     net = Nlnets[i];
	     emit_routed_net(Cmd, net, (u_char)1, oscale, iscale);
	}
	fprintf(Cmd, ";\nEND SPECIALNETS\n");
    }    

    // Finish copying the rest of the file
    while (fgets(line, MAX_LINE_LEN, fdef) != NULL) {
       fputs(line, Cmd);
    }
    fclose(fdef);
    fclose(Cmd);

} /* emit_routes() */

/*--------------------------------------------------------------*/
/* helpmessage - tell user how to use the program		*/
/*								*/
/*   ARGS: none.						*/
/*   RETURNS: nothing.						*/
/*   SIDE EFFECTS: 						*/
/*								*/
/* NOTES:							*/
/* 1) "qrouter -v0 -h" prints only the version number and exits	*/
/* 2) Tcl-Tk based version adds ".T" to the end to alert tools	*/
/*    attempting to query the capabilities of qrouter of the	*/
/*    availability of the scripting.				*/
/*								*/
/*--------------------------------------------------------------*/

void helpmessage()
{
    if (Verbose > 0) {
	Fprintf(stdout, "qrouter - maze router by Tim Edwards\n\n");
	Fprintf(stdout, "usage:  qrouter [-switches] design_name\n\n");
	Fprintf(stdout, "switches:\n");
	Fprintf(stdout, "\t-c <file>\t\t\tConfiguration file name if not route.cfg.\n");
	Fprintf(stdout, "\t-v <level>\t\t\tVerbose output level.\n");
	Fprintf(stdout, "\t-i <file>\t\t\tPrint route names and pitches and exit.\n");
	Fprintf(stdout, "\t-p <name>\t\t\tSpecify global power bus name.\n");
	Fprintf(stdout, "\t-g <name>\t\t\tSpecify global ground bus name.\n");
	Fprintf(stdout, "\n");
    }
#ifdef TCL_QROUTER
    Fprintf(stdout, "%s.%s.T\n", VERSION, REVISION);
#else
    Fprintf(stdout, "%s.%s\n", VERSION, REVISION);
#endif

} /* helpmessage() */

/* end of qrouter.c */
