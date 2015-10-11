/*--------------------------------------------------------------*/
/* node.c -- Generation	of detailed network and obstruction	*/
/* information on the routing grid based on the geometry of the	*/
/* layout of the standard cell macros.				*/
/*								*/
/*--------------------------------------------------------------*/
/* Written by Tim Edwards, June, 2011, based on work by Steve	*/
/* Beccue.							*/
/*--------------------------------------------------------------*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "qrouter.h"
#include "node.h"
#include "qconfig.h"
#include "lef.h"

/*--------------------------------------------------------------*/
/* Comparison routine used for qsort.  Sort nets by number of	*/
/* nodes.							*/
/*--------------------------------------------------------------*/

int compNets(NET *a, NET *b)
{
   NET p = *a;
   NET q = *b;

   // NULL nets get shoved up front
   if (p == NULL) return ((q == NULL) ? 0 : -1);
   if (q == NULL) return 1;

   // Sort critical nets at the front by assigned order

   if (p->flags & NET_CRITICAL) {
      if (q->flags & NET_CRITICAL) {
	 return (p->netorder < q->netorder) ? -1 : 1;
      }
      else return -1;
   }

   // Otherwise sort by number of nodes

   if (p->numnodes < q->numnodes)
      return 1;
   if (p->numnodes > q->numnodes)
      return -1;
   return 0;
}

/*--------------------------------------------------------------*/
/* Alternative net comparison used for qsort.  Sort nets by	*/
/* minimum dimension of the bounding box, and if equal, by the	*/
/* number of nodes in the net.  Bounding box dimensions are	*/
/* ordered smallest to largest, and number of nodes are ordered	*/
/* largest to smallest.						*/
/*--------------------------------------------------------------*/

int altCompNets(NET *a, NET *b)
{
   NET p = *a;
   NET q = *b;

   int pwidth, qwidth, pheight, qheight, pdim, qdim;

   // Any NULL nets get shoved up front
   if (p == NULL) return ((q == NULL) ? 0 : -1);
   if (q == NULL) return 1;

   // Sort critical nets at the front by assigned order

   if (p->flags & NET_CRITICAL) {
      if (q->flags & NET_CRITICAL) {
	 return (p->netorder < q->netorder) ? -1 : 1;
      }
      else return -1;
   }

   // Otherwise sort as described above.

   pwidth = p->xmax - p->xmin;
   pheight = p->ymax - p->ymin;
   pdim = (pwidth > pheight) ? pheight : pwidth;

   qwidth = q->xmax - q->xmin;
   qheight = q->ymax - q->ymin;
   qdim = (qwidth > qheight) ? qheight : qwidth;

   if (pdim < qdim)
      return (-1);
   else if (pdim > qdim)
      return (1);
   else {
      if (p->numnodes < q->numnodes)
         return (1);
      if (p->numnodes > q->numnodes)
         return (-1);
      return (0);
   }
}

/*--------------------------------------------------------------*/
/* create_netorder --- assign indexes to net->netorder    	*/
/* Re-sort Nlnets according to net order.  Since Nlnets is a	*/
/* global variable, nothing is returned from this routine.	*/
/*								*/
/* method = 0							*/
/* 	Nets are ordered simply from those with the most nodes	*/
/*	to those with the fewest.  However, any nets marked	*/
/* 	critical in the configuration or critical net files	*/
/*	will be given precedence.				*/
/*								*/
/* method = 1							*/
/*	Nets are ordered by minimum bounding box dimension.	*/
/*	This is based on the principle that small or narrow	*/
/*	nets have little room to be moved around without	*/
/*	greatly increasing the net length.  If these are put	*/
/*	down first, then remaining nets can route around them.	*/
/*--------------------------------------------------------------*/

void create_netorder(u_char method)
{
  int i, j, max;
  NET  net;
  STRING cn;

  i = 1;
  for (cn = CriticalNet; cn; cn = cn->next) {
     if (Verbose > 1)
	Fprintf(stdout, "critical net %s\n", cn->name);
     for (j = 0; j < Numnets; j++) {
	net = Nlnets[j];
	if (!strcmp(net->netname, (char *)cn->name)) {
           net->netorder = i++;
	   net->flags |= NET_CRITICAL;
	}
     }
  }

  switch (method) {
      case 0:
	 qsort((char *)Nlnets, Numnets, (int)sizeof(NET),
			(__compar_fn_t)compNets);
	 break;
      case 1:
	 qsort((char *)Nlnets, Numnets, (int)sizeof(NET),
			(__compar_fn_t)altCompNets);
	 break;
  }

  for (i = 0; i < Numnets; i++) {
     net = Nlnets[i];
     net->netorder = i++;
  }

} /* create_netorder() */

/*--------------------------------------------------------------*/
/* Measure and record the bounding box of a net.		*/
/* This is preparatory to generating a mask for the net.	*/
/* Find the bounding box of each node, and record that		*/
/* information, at the same time computing the whole net's	*/
/* bounding box as the area bounding all of the nodes.		*/
/* Determine if the bounding box is more horizontal or		*/
/* vertical, and specify a direction for the net's trunk line.	*/
/* Initialize the trunk line as the midpoint between all of the	*/
/* nodes, extending the width (or height) of the bounding box.	*/
/* Initialize the node branch position as the line extending	*/
/* from the middle of the node's bounding box to the trunk	*/
/* line.  These positions (trunk and branches) will be sorted	*/
/* and readjusted by "create_nodeorder()".			*/
/*--------------------------------------------------------------*/

void find_bounding_box(NET net)
{
   NODE n1, n2;
   DPOINT d1tap, d2tap, dtap, mintap;
   int mindist, dist, dx, dy;

   if (net->numnodes == 2) {

      n1 = (NODE)net->netnodes;
      n2 = (NODE)net->netnodes->next;

      // Simple 2-pass---pick up first tap on n1, find closest tap on n2,
      // then find closest tap on n1.

      d1tap = (n1->taps == NULL) ? n1->extend : n1->taps;
      if (d1tap == NULL) return;
      d2tap = (n2->taps == NULL) ? n2->extend : n2->taps;
      if (d2tap == NULL) return;
      dx = d2tap->gridx - d1tap->gridx;
      dy = d2tap->gridy - d1tap->gridy;
      mindist = dx * dx + dy * dy;
      mintap = d2tap;
      for (d2tap = d2tap->next; d2tap != NULL; d2tap = d2tap->next) {
         dx = d2tap->gridx - d1tap->gridx;
         dy = d2tap->gridy - d1tap->gridy;
         dist = dx * dx + dy * dy;
         if (dist < mindist) {
            mindist = dist;
            mintap = d2tap;
         }
      }
      d2tap = mintap;
      d1tap = (n1->taps == NULL) ? n1->extend : n1->taps;
      dx = d2tap->gridx - d1tap->gridx;
      dy = d2tap->gridy - d1tap->gridy;
      mindist = dx * dx + dy * dy;
      mintap = d1tap;
      for (d1tap = d1tap->next; d1tap != NULL; d1tap = d1tap->next) {
         dx = d2tap->gridx - d1tap->gridx;
         dy = d2tap->gridy - d1tap->gridy;
         dist = dx * dx + dy * dy;
         if (dist < mindist) {
            mindist = dist;
            mintap = d1tap;
         }
      }
      d1tap = mintap;

      net->xmin = (d1tap->gridx < d2tap->gridx) ? d1tap->gridx : d2tap->gridx;
      net->xmax = (d1tap->gridx < d2tap->gridx) ? d2tap->gridx : d1tap->gridx;
      net->ymin = (d1tap->gridy < d2tap->gridy) ? d1tap->gridy : d2tap->gridy;
      net->ymax = (d1tap->gridy < d2tap->gridy) ? d2tap->gridy : d1tap->gridy;
   }
   else {	// Net with more than 2 nodes

      // Use the first tap point for each node to get a rough bounding box and
      // centroid of all taps
      net->xmax = net->ymax = -(MAXRT);
      net->xmin = net->ymin = MAXRT;
      for (n1 = net->netnodes; n1 != NULL; n1 = n1->next) {
         dtap = (n1->taps == NULL) ? n1->extend : n1->taps;
	 if (dtap) {
            if (dtap->gridx > net->xmax) net->xmax = dtap->gridx;
            if (dtap->gridx < net->xmin) net->xmin = dtap->gridx;
            if (dtap->gridy > net->ymax) net->ymax = dtap->gridy;
            if (dtap->gridy < net->ymin) net->ymin = dtap->gridy;
	 }
      }
   }
}

/*--------------------------------------------------------------*/
/* defineRouteTree() ---					*/
/*								*/
/* Define a trunk-and-branches potential best route for a net.	*/
/*								*/
/* The net is analyzed for aspect ratio, and is determined if	*/
/* it will have a horizontal or vertical trunk.  Then, each	*/
/* node will define a branch line extending from the node	*/
/* position to the trunk.  Trunk position is recorded in the	*/
/* net record, and branch positions are recorded in the	node	*/
/* records.							*/
/*								*/
/* To do:							*/
/* Trunk and branch lines will be analyzed for immediate	*/
/* collisions and sorted to help ensure a free track exists for	*/
/* each net's trunk line.					*/
/*--------------------------------------------------------------*/

void defineRouteTree(NET net)
{
    NODE n1;
    DPOINT dtap;
    int xcent, ycent, xmin, ymin, xmax, ymax;

    // This is called after create_bounding_box(), so bounds have
    // been calculated.

    xmin = net->xmin;
    xmax = net->xmax;
    ymin = net->ymin;
    ymax = net->ymax;

    if (net->numnodes == 2) {

	// For 2-node nets, record the initial position as
	// one horizontal trunk + one branch for one "L" of
	// the bounding box, and one vertical trunk + one
	// branch for the other "L" of the bounding box.

	net->trunkx = xmin;
	net->trunky = ymin;
    }
    else if (net->numnodes > 0) {

	// Use the first tap point for each node to get a rough
	// centroid of all taps

	xcent = ycent = 0;
	for (n1 = net->netnodes; n1 != NULL; n1 = n1->next) {
	    dtap = (n1->taps == NULL) ? n1->extend : n1->taps;
	    if (dtap == NULL) continue;
	    xcent += dtap->gridx;
	    ycent += dtap->gridy;
	}
	xcent /= net->numnodes;
	ycent /= net->numnodes;

	// Record the trunk line in the net record

	net->trunkx = xcent;
	net->trunky = ycent;
    }

    if (xmax - xmin > ymax - ymin) {
	// Horizontal trunk preferred
	net->flags &= ~NET_VERTICAL_TRUNK;
    }
    else {
	// Vertical trunk preferred
	net->flags |= NET_VERTICAL_TRUNK;
    }

    // Set the branch line positions to the node tap points

    for (n1 = net->netnodes; n1; n1 = n1->next) {
	dtap = (n1->taps == NULL) ? n1->extend : n1->taps;
	if (!dtap) continue;
	n1->branchx = dtap->gridx;
	n1->branchy = dtap->gridy;
    }
}

/*--------------------------------------------------------------*/
/* print_nodes - show the nodes list				*/
/*         ARGS: filename to print to
        RETURNS: nothing
   SIDE EFFECTS: none
AUTHOR and DATE: steve beccue      Tue Aug 04  2003
\*--------------------------------------------------------------*/

void print_nodes(char *filename)
{
  FILE *o;
  int i, j;
  NET net;
  NODE node;
  DPOINT dp;

    if (!strcmp(filename, "stdout")) {
	o = stdout;
    } else {
	o = fopen(filename, "w");
    }
    if (!o) {
	Fprintf( stderr, "node.c:print_nodes.  Couldn't open output file\n" );
	return;
    }

    for (i = 0; i < Numnets; i++) {
       net = Nlnets[i];
       for (node = net->netnodes; node; node = node->next) {
	  dp = (DPOINT)node->taps;
	  fprintf(o, "%d\t%s\t(%g,%g)(%d,%d) :%d:num=%d netnum=%d\n",
		node->nodenum, 
		node->netname,
		// legacy:  print only the first point
		dp->x, dp->y, dp->gridx, dp->gridy,
		node->netnum, node->numnodes, node->netnum );
		 
	  /* need to print the routes to this node (deprecated)
	  for (j = 0 ; j < g->nodes; j++) {
	      fprintf(o, "%s(%g,%g) ", g->node[j], *(g->x[j]), *(g->y[j]));
	  }
	  */
       }
    }
    fclose(o);

} /* void print_nodes() */

/*--------------------------------------------------------------*/
/*C print_nlnets - show the nets				*/
/*         ARGS: filename to print to
        RETURNS: nothing
   SIDE EFFECTS: none
AUTHOR and DATE: steve beccue      Tue Aug 04  2003
\*--------------------------------------------------------------*/

void print_nlnets( char *filename )
{
  FILE *o;
  int i;
  NODE nd;
  NET net;

    if (!strcmp(filename, "stdout")) {
	o = stdout;
    } else {
	o = fopen(filename, "w");
    }
    if (!o) {
	Fprintf(stderr, "node.c:print_nlnets.  Couldn't open output file\n");
	return;
    }

    for (i = 0; i < Numnets; i++) {
        net = Nlnets[i];
	fprintf(o, "%d\t#=%d\t%s   \t\n", net->netnum, 
		 net->numnodes, net->netname);

	for (nd = net->netnodes; nd; nd = nd->next) {
	   fprintf(o, "%d ", nd->nodenum);
	}
    }

    fprintf(o, "%d nets\n", Numnets);
    fflush(o);

} /* void print_nlnets() */

/*--------------------------------------------------------------*/
/* create_obstructions_from_variable_pitch()			*/
/*								*/
/*  Although it would be nice to have an algorithm that would	*/
/*  work with any arbitrary pitch, qrouter will work around	*/
/*  having larger pitches on upper metal layers by selecting	*/
/*  1 out of every N tracks for routing, and placing 		*/
/*  obstructions in the interstices.  This makes the possibly	*/
/*  unwarranted assumption that the contact down to the layer	*/
/*  below does not cause spacing violations to neighboring	*/
/*  tracks.  If that assumption fails, this routine will have	*/
/*  to be revisited.						*/
/*--------------------------------------------------------------*/

void create_obstructions_from_variable_pitch()
{
   int l, o, vnum, hnum, x, y;
   double vpitch, hpitch, wvia;

   for (l = 0; l < Num_layers; l++) {
      o = LefGetRouteOrientation(l);

      // Pick the best via size for the layer.  Usually this means the
      // via whose base is at layer - 1, because the top metal layer
      // will either have the same width or a larger width.  

      // Note that when "horizontal" (o = 1) is passed to LefGetViaWidth,
      // it returns the via width side-to-side; but for horizontal routing
      // the dimension of interest is the height of the via.  Therefore
      // the direction argument passed to LefGetViaWidth is (1 - o).

      if (l == 0)
	 wvia = LefGetViaWidth(l, l, (1 - o));
      else
	 wvia = LefGetViaWidth(l - 1, l, (1 - o));

      if (o == 1) {	// Horizontal route
	 vpitch = LefGetRoutePitch(l);
	 // Changed:  routes must be able to accomodate the placement
	 // of a via in the track next to it.

	 // hpitch = LefGetRouteWidth(l) + LefGetRouteSpacing(l);
	 hpitch = 0.5 * (LefGetRouteWidth(l) + wvia) + LefGetRouteSpacing(l);
      }
      else {		// Vertical route
	 hpitch = LefGetRoutePitch(l);
	 // vpitch = LefGetRouteWidth(l) + LefGetRouteSpacing(l);
	 vpitch = 0.5 * (LefGetRouteWidth(l) + wvia) + LefGetRouteSpacing(l);
      }

      vnum = 1;
      while (vpitch > PitchY[l] + EPS) {
	 vpitch /= 2.0;
	 vnum++;
      }
      hnum = 1;
      while (hpitch > PitchX[l] + EPS) {
	 hpitch /= 2.0;
	 hnum++;
      }

      // This could be better handled by restricting
      // access from specific directions rather than
      // marking a position as NO_NET.  Since the
      // routine below will mark no positions restricted
      // if either hnum is 1 or vnum is 1, regardless of
      // the other value, then we force both values to
      // be at least 2.

      if (vnum > 1 && hnum == 1) hnum++;
      if (hnum > 1 && vnum == 1) vnum++;

      if (vnum > 1 || hnum > 1) {
	 for (x = 0; x < NumChannelsX[l]; x++) {
	    if (x % hnum == 0) continue;
	    for (y = 0; y < NumChannelsY[l]; y++) {
	       if (y % vnum == 0) continue;

	       // If there is a node in an adjacent grid
	       // then allow routing from that direction.

	       if ((x > 0) && Nodeloc[l][OGRID(x - 1, y, l)] != NULL)
		  Obs[l][OGRID(x, y, l)] = BLOCKED_MASK & ~BLOCKED_W;
	       else if ((y > 0) && Nodeloc[l][OGRID(x , y - 1, l)] != NULL)
		  Obs[l][OGRID(x, y, l)] = BLOCKED_MASK & ~BLOCKED_S;
	       else if ((x < NumChannelsX[l] - 1)
			&& Nodeloc[l][OGRID(x + 1, y, l)] != NULL)
		  Obs[l][OGRID(x, y, l)] = BLOCKED_MASK & ~BLOCKED_E;
	       else if ((y < NumChannelsY[l] - 1)
			&& Nodeloc[l][OGRID(x, y + 1, l)] != NULL)
		  Obs[l][OGRID(x, y, l)] = BLOCKED_MASK & ~BLOCKED_N;
	       else
		  Obs[l][OGRID(x, y, l)] = NO_NET;
	    }
	 }
      }
   }
}

/*--------------------------------------------------------------*/
/* disable_gridpos() ---					*/
/*	Render the position at (x, y, lay) unroutable by	*/
/*	setting its Obs[] entry to NO_NET and removing it from	*/
/*	the Nodeloc and Nodesav records.			*/
/*--------------------------------------------------------------*/

void
disable_gridpos(int x, int y, int lay)
{
    int apos = OGRID(x, y, lay);

    Obs[lay][apos] = (u_int)(NO_NET | OBSTRUCT_MASK);
    Nodeloc[lay][apos] = NULL;
    Nodesav[lay][apos] = NULL;
    Stub[lay][apos] = 0.0;
}

/*--------------------------------------------------------------*/
/* count_pinlayers()---						*/
/*	Check which layers have non-NULL Nodeloc, Nodesav, and	*/
/* 	Stub entries.  Then set "Pinlayers" and free all the	*/
/*	unused layers.  This saves a lot of memory, especially	*/
/*	when the number of routing layers becomes large.	*/ 
/*--------------------------------------------------------------*/

void
count_pinlayers()
{
   int x, y, l, haspin;

   Pinlayers = 0;
   for (l = 0; l < Num_layers; l++) {
      haspin = 0;
      for (x = 0; x < NumChannelsX[l]; x++) {
	 for (y = 0; y < NumChannelsY[l]; y++) {
	    if (Nodesav[l][OGRID(x, y, l)] != NULL) {
	       haspin = 1;
	       Pinlayers = l + 1;
	       break;
	    }
	 }
	 if (haspin) break;
      }
   }

   for (l = Pinlayers; l < Num_layers; l++) {
      free(Stub[l]);
      free(Nodeloc[l]);
      free(Nodesav[l]);
      Stub[l] = NULL;
      Nodeloc[l] = NULL;
      Nodesav[l] = NULL;
   }
}

/*--------------------------------------------------------------*/
/* check_obstruct()---						*/
/*	Called from create_obstructions_from_gates(), this	*/
/* 	routine takes a grid point at (gridx, gridy) (physical	*/
/* 	position (dx, dy)) and an obstruction defined by the	*/
/*	rectangle "ds", and sets flags and fills the Obsinfo	*/
/*	array to reflect how the obstruction affects routing to	*/
/*	the grid position.					*/
/*--------------------------------------------------------------*/

void
check_obstruct(int gridx, int gridy, DSEG ds, double dx, double dy)
{
    int *obsptr;
    float dist;

    obsptr = &(Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]);
    dist = Obsinfo[ds->layer][OGRID(gridx, gridy, ds->layer)];

    // Grid point is inside obstruction + halo.
    *obsptr |= NO_NET;

    // Completely inside obstruction?
    if (dy > ds->y1 && dy < ds->y2 && dx > ds->x1 && dx < ds->x2)
       *obsptr |= OBSTRUCT_MASK;

    else {

       // Make more detailed checks in each direction

       if (dy < ds->y1) {
	  if ((*obsptr & (OBSTRUCT_MASK & ~OBSTRUCT_N)) == 0) {
	     if ((dist == 0) || ((ds->y1 - dy) < dist))
		Obsinfo[ds->layer][OGRID(gridx, gridy, ds->layer)] = ds->y1 - dy;
	     *obsptr |= OBSTRUCT_N;
	  }
	  else *obsptr |= OBSTRUCT_MASK;
       }
       else if (dy > ds->y2) {
	  if ((*obsptr & (OBSTRUCT_MASK & ~OBSTRUCT_S)) == 0) {
	     if ((dist == 0) || ((dy - ds->y2) < dist))
		Obsinfo[ds->layer][OGRID(gridx, gridy, ds->layer)] = dy - ds->y2;
	     *obsptr |= OBSTRUCT_S;
	  }
	  else *obsptr |= OBSTRUCT_MASK;
       }
       if (dx < ds->x1) {
	  if ((*obsptr & (OBSTRUCT_MASK & ~OBSTRUCT_E)) == 0) {
	     if ((dist == 0) || ((ds->x1 - dx) < dist))
		Obsinfo[ds->layer][OGRID(gridx, gridy, ds->layer)] = ds->x1 - dx;
             *obsptr |= OBSTRUCT_E;
	  }
	  else *obsptr |= OBSTRUCT_MASK;
       }
       else if (dx > ds->x2) {
	  if ((*obsptr & (OBSTRUCT_MASK & ~OBSTRUCT_W)) == 0) {
	     if ((dist == 0) || ((dx - ds->x2) < dist))
		Obsinfo[ds->layer][OGRID(gridx, gridy, ds->layer)] = dx - ds->x2;
	     *obsptr |= OBSTRUCT_W;
	  }
	  else *obsptr |= OBSTRUCT_MASK;
       }
   }
}

/*--------------------------------------------------------------*/
/* Find the amount of clearance needed between an obstruction	*/
/* and a route track position.  This takes into consideration	*/
/* whether the obstruction is wide or narrow metal, if the	*/
/* spacing rules are graded according to metal width, and if a	*/
/* via placed at the position is or is not symmetric in X and Y	*/
/*--------------------------------------------------------------*/

double get_via_clear(int lay, int horiz, DSEG rect) {
   double vdelta, v2delta, mdelta, mwidth;

   vdelta = LefGetViaWidth(lay, lay, 1 - horiz);
   if (lay > 0) {
	v2delta = LefGetViaWidth(lay - 1, lay, 1 - horiz);
	if (v2delta > vdelta) vdelta = v2delta;
   }
   vdelta = vdelta / 2.0;

   // Spacing rule is determined by the minimum metal width,
   // either in X or Y, regardless of the position of the
   // metal being checked.

   mwidth = MIN(rect->x2 - rect->x1, rect->y2 - rect->y1);
   mdelta = LefGetRouteWideSpacing(lay, mwidth);

   return vdelta + mdelta;
}

/*--------------------------------------------------------------*/
/* Find the distance from an obstruction to a grid point, 	*/
/* considering only routes which are placed at the position,	*/
/* not vias.							*/
/*--------------------------------------------------------------*/

double get_route_clear(int lay, DSEG rect) {
   double rdelta, mdelta, mwidth;

   rdelta = LefGetRouteWidth(lay);
   rdelta = rdelta / 2.0;

   // Spacing rule is determined by the minimum metal width,
   // either in X or Y, regardless of the position of the
   // metal being checked.

   mwidth = MIN(rect->x2 - rect->x1, rect->y2 - rect->y1);
   mdelta = LefGetRouteWideSpacing(lay, mwidth);

   return rdelta + mdelta;
}

/*--------------------------------------------------------------*/
/* Find the distance from a point to the edge of a clearance	*/
/* around a rectangle.  To satisfy euclidean distance rules, 	*/
/* the clearance is clrx on the left and right sides of the	*/
/* rectangle, clry on the top and bottom sides, and rounded on	*/
/* the corners.							*/
/*								*/
/* Return 1 if point passes clearance test, 0 if not.		*/
/*								*/
/* This routine not currently being used, but probably will	*/
/* need to be, eventually, to get a correct evaluation of	*/
/* tightly-spaced taps that violate manhattan rules but pass	*/
/* euclidean rules.						*/
/*--------------------------------------------------------------*/

char point_clearance_to_rect(double dx, double dy, DSEG ds,
	double clrx, double clry)
{
   double delx, dely, dist, xp, yp, alpha, yab;
   struct dseg_ dexp;

   dexp.x1 = ds->x1 - clrx;
   dexp.x2 = ds->x2 + clrx;
   dexp.y1 = ds->y1 - clry;
   dexp.y2 = ds->y2 + clry;

   /* If the point is between ds top and bottom, distance is	*/
   /* simple.							*/

   if (dy <= ds->y2 && dy >= ds->y1) {
      if (dx < dexp.x1)
	return (dexp.x1 - dx) > 0 ? 1 : 0;
      else if (dx > dexp.x2)
	return (dx - dexp.x2) > 0 ? 1 : 0;
      else
	return 0;	// Point is inside rect
   }

   /* Likewise if the point is between ds right and left	*/

   if (dx <= ds->x2 && dx >= ds->x1) {
      if (dy < dexp.y1)
	return (dexp.y1 - dy) > 0 ? 1 : 0;
      else if (dy > dexp.y2)
	return (dy - dexp.y2) > 0 ? 1 : 0;
      else
	return 0;	// Point is inside rect
   }

   /* Treat corners individually */

   if (dy > ds->y2)
      yab = dy - ds->y2;
   else if (dy < ds->y1)
      yab = ds->y1 - dy;

   if (dx > ds->x2)
      delx = dx - ds->x2;
   else if (dx < ds->x1)
      delx = ds->x1 - dx;

   dely = yab * (clrx / clry);	// Normalize y clearance to x
   dist = delx * delx + dely * dely;
   return (dist > (clrx * clrx)) ? 1 : 0;
}

/*--------------------------------------------------------------*/
/* create_obstructions_from_gates()				*/
/*								*/
/*  Fills in the Obs[][] grid from obstructions that were	*/
/*  defined for each macro in the technology LEF file and	*/
/*  translated into a list of grid coordinates in each		*/
/*  instance.							*/
/*								*/
/*  Also, fills in the Obs[][] grid with obstructions that	*/
/*  are defined by nodes of the gate that are unconnected in	*/
/*  this netlist.						*/
/*--------------------------------------------------------------*/

void create_obstructions_from_gates()
{
    GATE g;
    DSEG ds;
    int i, gridx, gridy, *obsptr;
    double deltax, deltay, delta[MAX_LAYERS];
    double dx, dy, deltaxy;
    float dist;

    // Give a single net number to all obstructions, over the range of the
    // number of known nets, so these positions cannot be routed through.
    // If a grid position is not wholly inside an obstruction, then we
    // maintain the direction of the nearest obstruction in Obs and the
    // distance to it in Obsinfo.  This indicates that a route can avoid
    // the obstruction by moving away from it by the amount in Obsinfo
    // plus spacing clearance.  If another obstruction is found that
    // prevents such a move, then all direction flags will be set, indicating
    // that the position is not routable under any condition. 

    for (g = Nlgates; g; g = g->next) {
       for (ds = g->obs; ds; ds = ds->next) {

	  deltax = get_via_clear(ds->layer, 1, ds);
	  gridx = (int)((ds->x1 - Xlowerbound - deltax)
			/ PitchX[ds->layer]) - 1;
	  while (1) {
	     dx = (gridx * PitchX[ds->layer]) + Xlowerbound;
	     if ((dx + EPS) > (ds->x2 + deltax)
			|| gridx >= NumChannelsX[ds->layer]) break;
	     else if ((dx - EPS) > (ds->x1 - deltax) && gridx >= 0) {
		deltay = get_via_clear(ds->layer, 0, ds);
	        gridy = (int)((ds->y1 - Ylowerbound - deltay)
			/ PitchY[ds->layer]) - 1;
	        while (1) {
		   dy = (gridy * PitchY[ds->layer]) + Ylowerbound;
	           if ((dy + EPS) > (ds->y2 + deltay)
				|| gridy >= NumChannelsY[ds->layer]) break;
		   if ((dy - EPS) > (ds->y1 - deltay) && gridy >= 0) {
		      // If it clears distance for a route layer but not
		      // vias, then block vias only.
		      deltaxy = get_route_clear(ds->layer, ds);
		      if (((dx - EPS) <= (ds->x1 - deltaxy)) ||
				((dx + EPS) >= (ds->x2 + deltaxy)) ||
				((dy - EPS) <= (ds->y1 - deltaxy)) ||
				((dy + EPS) >= (ds->y2 + deltaxy))) {
			 block_route(gridx, gridy, ds->layer, UP);
			 block_route(gridx, gridy, ds->layer, DOWN);
		      }
		      else
			 check_obstruct(gridx, gridy, ds, dx, dy);
		   }
		   gridy++;
		}
	     }
	     gridx++;
	  }
       }

       for (i = 0; i < g->nodes; i++) {
	  if (g->netnum[i] == 0) {	/* Unconnected node */
	     // Diagnostic, and power bus handling
	     if (g->node[i]) {
		// Should we flag a warning if we see something that looks
		// like a power or ground net here?
		if (Verbose > 1)
		   Fprintf(stdout, "Gate instance %s unconnected node %s\n",
			g->gatename, g->node[i]);
	     }
	     else {
		if (Verbose > 1)
	           Fprintf(stdout, "Gate instance %s unconnected node (%d)\n",
			g->gatename, i);
	     }
             for (ds = g->taps[i]; ds; ds = ds->next) {

		deltax = get_via_clear(ds->layer, 1, ds);
		gridx = (int)((ds->x1 - Xlowerbound - deltax)
			/ PitchX[ds->layer]) - 1;
		while (1) {
		   dx = (gridx * PitchX[ds->layer]) + Xlowerbound;
		   if (dx > (ds->x2 + deltax)
				|| gridx >= NumChannelsX[ds->layer]) break;
		   else if (dx >= (ds->x1 - deltax) && gridx >= 0) {
		      deltay = get_via_clear(ds->layer, 0, ds);
		      gridy = (int)((ds->y1 - Ylowerbound - deltay)
				/ PitchY[ds->layer]) - 1;
		      while (1) {
		         dy = (gridy * PitchY[ds->layer]) + Ylowerbound;
		         if ((dy + EPS) > (ds->y2 + deltay)
					|| gridy >= NumChannelsY[ds->layer]) break;
		         if ((dy - EPS) >= (ds->y1 - deltay) && gridy >= 0) {
		            // If it clears distance for a route layer but not
		            // vias, then block vias only.
		            deltaxy = get_route_clear(ds->layer, ds);
		            if (((dx - EPS) < (ds->x1 - deltaxy)) ||
					((dx + EPS) > (ds->x2 + deltaxy)) ||
					((dy - EPS) < (ds->y1 - deltaxy)) ||
					((dy + EPS) > (ds->y2 + deltaxy))) {
			       block_route(gridx, gridy, ds->layer, UP);
			       block_route(gridx, gridy, ds->layer, DOWN);
		            }
		            else
			       check_obstruct(gridx, gridy, ds, dx, dy);
			 }
		         gridy++;
		      }
		   }
		   gridx++;
		}
	     }
	  }
       }
    }

    // Create additional obstructions from the UserObs list
    // These obstructions are not considered to be metal layers,
    // so we don't compute a distance measure.  However, we need
    // to compute a boundary of 1/2 route width to avoid having
    // the route overlapping the obstruction area.

    for (i = 0; i < Num_layers; i++) {
	delta[i] = LefGetRouteWidth(i) / 2.0;
    }

    for (ds = UserObs; ds; ds = ds->next) {
	gridx = (int)((ds->x1 - Xlowerbound - delta[ds->layer])
			/ PitchX[ds->layer]) - 1;
	while (1) {
	    dx = (gridx * PitchX[ds->layer]) + Xlowerbound;
	    if (dx > (ds->x2 + delta[ds->layer])
			|| gridx >= NumChannelsX[ds->layer]) break;
	    else if (dx >= (ds->x1 - delta[ds->layer]) && gridx >= 0) {
		gridy = (int)((ds->y1 - Ylowerbound - delta[ds->layer])
				/ PitchY[ds->layer]) - 1;
		while (1) {
		    dy = (gridy * PitchY[ds->layer]) + Ylowerbound;
		    if (dy > (ds->y2 + delta[ds->layer])
				|| gridy >= NumChannelsY[ds->layer]) break;
		    if (dy >= (ds->y1 - delta[ds->layer]) && gridy >= 0)
		       check_obstruct(gridx, gridy, ds, dx, dy);

		    gridy++;
		}
	    }
	    gridx++;
	}
    }
}

/*--------------------------------------------------------------*/
/* expand_tap_geometry()					*/
/*								*/
/*  For each rectangle defining part of a gate terminal,	*/
/*  search the surrounding terminal geometry.  If the rectangle	*/
/*  can expand in any direction, then allow it to grow to the	*/
/*  maximum size.  This generates overlapping geometry for each	*/
/*  terminal, but avoids bad results for determining how to	*/
/*  route to a terminal point if the terminal is broken up into	*/
/*  numerous nonoverlapping rectangles.				*/
/*								*/
/*  Note that this is not foolproof.  It also needs a number of	*/
/*  enhancements.  For example, to expand east, other geometry	*/
/*  should be looked at in order of increasing left edge X	*/
/*  value, and so forth.					*/
/*--------------------------------------------------------------*/

void expand_tap_geometry()
{
    DSEG ds, ds2;
    GATE g;
    int i;
    u_char expanded;

    for (g = Nlgates; g; g = g->next) {
	for (i = 0; i < g->nodes; i++) {
	    if (g->netnum[i] == 0) continue;

	    for (ds = g->taps[i]; ds; ds = ds->next) {
		expanded = TRUE;
		while (expanded == TRUE) {
		    expanded = FALSE;

		    for (ds2 = g->taps[i]; ds2; ds2 = ds2->next) {
		        if (ds == ds2) continue;
			if (ds->layer != ds2->layer) continue;
		    
		        if ((ds2->y1 <= ds->y1) && (ds2->y2 >= ds->y2)) {
			    // Expand east
			    if ((ds2->x1 > ds->x1) && (ds2->x1 <= ds->x2))
			        if (ds->x2 < ds2->x2) {
				    ds->x2 = ds2->x2;
				    expanded = TRUE;
			        }
		    
			    // Expand west
			    if ((ds2->x2 < ds->x2) && (ds2->x2 >= ds->x1))
			        if (ds->x1 > ds2->x1) {
				    ds->x1 = ds2->x1;
				    expanded = TRUE;
			        }
		        }
		    
		        if ((ds2->x1 <= ds->x1) && (ds2->x2 >= ds->x2)) {
			    // Expand north
			    if ((ds2->y1 > ds->y1) && (ds2->y1 <= ds->y2))
			        if (ds->y2 < ds2->y2) {
				    ds->y2 = ds2->y2;
				    expanded = TRUE;
			        }
		    
			    // Expand south
			    if ((ds2->y2 < ds->y2) && (ds2->y2 >= ds->y1))
			        if (ds->y1 > ds2->y1) {
				    ds->y1 = ds2->y1;
				    expanded = TRUE;
			        }
		        }
		    }
		}
	    }
	}
    }
}

/*--------------------------------------------------------------*/
/* create_obstructions_from_nodes()				*/
/*								*/
/*  Fills in the Obs[][] grid from the position of each node	*/
/*  (net terminal), which may have multiple unconnected		*/
/*  positions.							*/
/*								*/
/*  Also fills in the Nodeloc[] grid with the node number,	*/
/*  which causes the router to put a premium on			*/
/*  routing other nets over or under this position, to		*/
/*  discourage boxing in a pin position and making it 		*/
/*  unroutable.							*/
/*								*/
/*  ARGS: none.							*/
/*  RETURNS: nothing						*/
/*  SIDE EFFECTS: none						*/
/*  AUTHOR:  Tim Edwards, June 2011, based on code by Steve	*/
/*	Beccue.							*/
/*--------------------------------------------------------------*/

void create_obstructions_from_nodes()
{
    NODE node, n2;
    GATE g;
    DPOINT dp;
    DSEG ds;
    u_int dir, k;
    int i, gx, gy, gridx, gridy, net;
    double dx, dy, deltax, deltay;
    float dist, xdist;
    double offmaxx[MAX_LAYERS], offmaxy[MAX_LAYERS];

    // Use a more conservative definition of keepout, to include via
    // widths, which may be bigger than route widths.

    for (i = 0; i < Num_layers; i++) {
	offmaxx[i] = PitchX[i] - LefGetRouteSpacing(i)
		- 0.5 * (LefGetRouteWidth(i) + LefGetViaWidth(i, i, 0));
	offmaxy[i] = PitchY[i] - LefGetRouteSpacing(i)
		- 0.5 * (LefGetRouteWidth(i) + LefGetViaWidth(i, i, 1));
    }

    // When we place vias at an offset, they have to satisfy the spacing
    // requirements for the via's top layer, as well.  So take the least
    // maximum offset of each layer and the layer above it.

    for (i = 0; i < Num_layers - 1; i++) {
       offmaxx[i] = MIN(offmaxx[i], offmaxx[i + 1]);
       offmaxy[i] = MIN(offmaxy[i], offmaxy[i + 1]);
    }

    // For each node terminal (gate pin), mark each grid position with the
    // net number.  This overrides any obstruction that may be placed at that
    // point.

    // For each pin position, we also find the "keepout" area around the
    // pin where we may not place an unrelated route.  For this, we use a
    // flag bit, so that the position can be ignored when routing the net
    // associated with the pin.  Normal obstructions take precedence.

    for (g = Nlgates; g; g = g->next) {
       for (i = 0; i < g->nodes; i++) {
	  if (g->netnum[i] != 0) {

	     // Get the node record associated with this pin.
	     node = g->noderec[i];
	     if (node == NULL) continue;

	     // First mark all areas inside node geometry boundary.

             for (ds = g->taps[i]; ds; ds = ds->next) {
		gridx = (int)((ds->x1 - Xlowerbound) / PitchX[ds->layer]) - 1;
		while (1) {
		   dx = (gridx * PitchX[ds->layer]) + Xlowerbound;
		   if (dx > ds->x2 || gridx >= NumChannelsX[ds->layer]) break;
		   else if (dx >= ds->x1 && gridx >= 0) {
		      gridy = (int)((ds->y1 - Ylowerbound) / PitchY[ds->layer]) - 1;
		      while (1) {
		         dy = (gridy * PitchY[ds->layer]) + Ylowerbound;
		         if (dy > ds->y2 || gridy >= NumChannelsY[ds->layer]) break;

			 // Area inside defined pin geometry

			 if (dy > ds->y1 && gridy >= 0) {
			     int orignet = Obs[ds->layer][OGRID(gridx,
					gridy, ds->layer)];

			     if ((orignet & ROUTED_NET_MASK) == (u_int)node->netnum) {

				// Duplicate tap point.   Don't re-process it. (?)
				gridy++;
				continue;
			     }

			     if (!(orignet & NO_NET) &&
					((orignet & ROUTED_NET_MASK) != (u_int)0)) {

				// Net was assigned to other net, but is inside
				// this pin's geometry.  Declare point to be
				// unroutable, as it is too close to both pins.
				// NOTE:  This is a conservative rule and could
				// potentially make a pin unroutable.
				// Another note:  By setting Obs[] to
				// OBSTRUCT_MASK as well as NO_NET, we ensure
				// that it falls through on all subsequent
				// processing.

				disable_gridpos(gridx, gridy, ds->layer);
			     }
			     else if (!(orignet & NO_NET)) {

				// A grid point that is within 1/2 route width
				// of a tap rectangle corner can violate metal
				// width rules, and so should declare a stub.
				
				dir = 0;
				dist = 0.0;
			        xdist = 0.5 * LefGetRouteWidth(ds->layer);

				if (dx >= ds->x2 - xdist) {
				   if (dy > ds->y2 - xdist + EPS) {
				      // Check northeast corner

				      if ((ds->x2 - dx) > (ds->y2 - dy)) {
					 // West-pointing stub
					 dir = STUBROUTE_EW;
					 dist = ds->x2 - dx - 2.0 * xdist;
				      }
				      else {
					 // South-pointing stub
					 dir = STUBROUTE_NS;
					 dist = ds->y2 - dy - 2.0 * xdist;
				      }

				   }
				   else if (dy < ds->y1 + xdist - EPS) {
				      // Check southeast corner

				      if ((ds->x2 - dx) > (dy - ds->y1)) {
					 // West-pointing stub
					 dir = STUBROUTE_EW;
					 dist = ds->x2 - dx - 2.0 * xdist;
				      }
				      else {
					 // North-pointing stub
					 dir = STUBROUTE_NS;
					 dist = ds->y1 - dy + 2.0 * xdist;
				      }
				   }
				}
				else if (dx <= ds->x1 + xdist) {
				   if (dy > ds->y2 - xdist + EPS) {
				      // Check northwest corner

				      if ((dx - ds->x1) > (ds->y2 - dy)) {
					 // East-pointing stub
					 dir = STUBROUTE_EW;
					 dist = ds->x1 - dx + 2.0 * xdist;
				      }
				      else {
					 // South-pointing stub
					 dir = STUBROUTE_NS;
					 dist = ds->y2 - dy - 2.0 * xdist;
				      }

				   }
				   else if (dy < ds->y1 + xdist - EPS) {
				      // Check southwest corner

				      if ((dx - ds->x2) > (dy - ds->y1)) {
					 // East-pointing stub
					 dir = STUBROUTE_EW;
					 dist = ds->x1 - dx + 2.0 * xdist;
				      }
				      else {
					 // North-pointing stub
					 dir = STUBROUTE_NS;
					 dist = ds->y1 - dy + 2.0 * xdist;
				      }
				   }
				}

			        Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
			        	= (Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
					   & BLOCKED_MASK) | (u_int)node->netnum | dir;
			        Nodeloc[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= node;
			        Nodesav[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= node;
			        Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= dist;

			     }
			     else if ((orignet & NO_NET) && ((orignet & OBSTRUCT_MASK)
					!= OBSTRUCT_MASK)) {
				double sdistx = LefGetViaWidth(ds->layer, ds->layer, 0)
					/ 2.0 + LefGetRouteSpacing(ds->layer);
				double sdisty = LefGetViaWidth(ds->layer, ds->layer, 1)
					/ 2.0 + LefGetRouteSpacing(ds->layer);
				double offd;

				// Define a maximum offset we can have in X or
				// Y above which the placement of a via will
				// cause a DRC violation with a wire in the
				// adjacent route track in the direction of the
				// offset.

				int maxerr = 0;

				// If a cell is positioned off-grid, then a grid
				// point may be inside a pin and still be unroutable.
				// The Obsinfo[] array tells where an obstruction is,
				// if there was only one obstruction in one direction
				// blocking the grid point.  If so, then we set the
				// Stub[] distance to move the tap away from the
				// obstruction to resolve the DRC error.

				// Make sure we have marked this as a node.
			        Nodeloc[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= node;
			        Nodesav[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= node;
			        Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
			        	= (Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
					   & BLOCKED_MASK) | (u_int)node->netnum;

				if (orignet & OBSTRUCT_N) {
			           offd = -(sdisty - Obsinfo[ds->layer]
					[OGRID(gridx, gridy, ds->layer)]);
				   if (offd >= -offmaxy[ds->layer]) {
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= offd;
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						|= (STUBROUTE_NS | OFFSET_TAP);
				   }
				   else maxerr = 1;
				}
				else if (orignet & OBSTRUCT_S) {
				   offd = sdisty - Obsinfo[ds->layer]
					[OGRID(gridx, gridy, ds->layer)];
				   if (offd <= offmaxy[ds->layer]) {
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= offd;
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						|= (STUBROUTE_NS | OFFSET_TAP);
				   }
				   else maxerr = 1;
				}
				else if (orignet & OBSTRUCT_E) {
				   offd = -(sdistx - Obsinfo[ds->layer]
					[OGRID(gridx, gridy, ds->layer)]);
				   if (offd >= -offmaxx[ds->layer]) {
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= offd;
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						|= (STUBROUTE_EW | OFFSET_TAP);
				   }
				   else maxerr = 1;
				}
				else if (orignet & OBSTRUCT_W) {
				   offd = sdistx - Obsinfo[ds->layer]
					[OGRID(gridx, gridy, ds->layer)];
				   if (offd <= offmaxx[ds->layer]) {
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= offd;
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						|= (STUBROUTE_EW | OFFSET_TAP);
				   }
				   else maxerr = 1;
				}

			        if (maxerr == 1)
				   disable_gridpos(gridx, gridy, ds->layer);

				// Diagnostic
				else if (Verbose > 3)
				   Fprintf(stderr, "Port overlaps obstruction"
					" at grid %d %d, position %g %g\n",
					gridx, gridy, dx, dy);
			     }
			     else if (orignet & NO_NET) {
				// The code in create_obstructions_from_gates
				// does manhattan, not eucliean, checks.  If
				// an obstruction is inside the manhattan
				// distance, it will be marked NO_NET.  To
				// work around this, if rect ds completely
				// surrounds a via centered on the grid point,
				// then allow it unconditionally.

				deltax = 0.5 * LefGetViaWidth(ds->layer, ds->layer, 0);
				deltay = 0.5 * LefGetViaWidth(ds->layer, ds->layer, 0);
				if (((dx - ds->x1 + EPS) > deltax) &&
					((ds->x2 - dx + EPS) > deltax) &&
					((dy - ds->y1 + EPS) > deltay) &&
					((ds->y2 - dy + EPS) > deltay)) {
			           Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
			        	= (Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
					   & BLOCKED_MASK) | (u_int)node->netnum;
			           Nodeloc[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= node;
			           Nodesav[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= node;
				}
			     }

			     // Check that we have not created a PINOBSTRUCT
			     // route directly over this point.
			     if (ds->layer < Num_layers - 1) {
			        k = Obs[ds->layer + 1][OGRID(gridx, gridy,
					ds->layer + 1)];
			        if (k & PINOBSTRUCTMASK) {
			           if ((k & ROUTED_NET_MASK) != (u_int)node->netnum) {
				       Obs[ds->layer + 1][OGRID(gridx, gridy,
						ds->layer + 1)] = NO_NET;
				       Nodeloc[ds->layer + 1][OGRID(gridx, gridy,
						ds->layer + 1)] = (NODE)NULL;
				       Nodesav[ds->layer + 1][OGRID(gridx, gridy,
						ds->layer + 1)] = (NODE)NULL;
				       Stub[ds->layer + 1][OGRID(gridx, gridy,
						ds->layer + 1)] = (float)0.0;
				   }
				}
			     }
			 }
		         gridy++;
		      }
		   }
		   gridx++;
		}
	     }

	     // Repeat this whole exercise for areas in the halo outside
	     // the node geometry.  We have to do this after enumerating
	     // all inside areas because the tap rectangles often overlap,
	     // and one rectangle's halo may be inside another tap.

             for (ds = g->taps[i]; ds; ds = ds->next) {

		// Note:  Should be handling get_route_clear as a less
		// restrictive case, as was done above.
 
		deltax = get_via_clear(ds->layer, 1, ds);
		gridx = (int)((ds->x1 - Xlowerbound - deltax)
			/ PitchX[ds->layer]) - 1;

		while (1) {
		   dx = (gridx * PitchX[ds->layer]) + Xlowerbound;

		   if ((dx + EPS) > (ds->x2 + deltax) ||
				gridx >= NumChannelsX[ds->layer])
		      break;

		   else if ((dx - EPS) > (ds->x1 - deltax) && gridx >= 0) {
		      deltay = get_via_clear(ds->layer, 0, ds);
		      gridy = (int)((ds->y1 - Ylowerbound - deltay)
				/ PitchY[ds->layer]) - 1;

		      while (1) {
		         dy = (gridy * PitchY[ds->layer]) + Ylowerbound;
		         if ((dy + EPS) > (ds->y2 + deltay) ||
				gridy >= NumChannelsY[ds->layer])
			    break;

		         if ((dy - EPS) > (ds->y1 - deltay) && gridy >= 0) {
			    xdist = 0.5 * LefGetRouteWidth(ds->layer);

			    // Area inside halo around defined pin geometry.
			    // Exclude areas already processed (areas inside
			    // some pin geometry have been marked with netnum)

			    // Also check that we are not about to define a
			    // route position for a pin on a layer above 0 that
			    // blocks a pin underneath it.

			    n2 = NULL;
			    if (ds->layer > 0)
			       n2 = Nodeloc[ds->layer - 1][OGRID(gridx, gridy,
					ds->layer - 1)];
			    if (n2 == NULL)
			       n2 = Nodeloc[ds->layer][OGRID(gridx, gridy, ds->layer)];

			    else {
			       // Watch out for the case where a tap crosses
			       // over a different tap.  Don't treat the tap
			       // on top as if it is not there!

			       NODE n3;
			       n3 = Nodeloc[ds->layer][OGRID(gridx, gridy, ds->layer)];
			       if (n3 != NULL && n3 != node) n2 = n3;
			    }

			    // Ignore my own node.
			    if (n2 == node) n2 = NULL;

			    k = Obs[ds->layer][OGRID(gridx, gridy, ds->layer)];

			    // In case of a port that is inaccessible from a grid
			    // point, or not completely overlapping it, the
			    // stub information will show how to adjust the
			    // route position to cleanly attach to the port.

			    dir = STUBROUTE_X;
			    dist = 0.0;

			    if (((k & ROUTED_NET_MASK) != (u_int)node->netnum)
					&& (n2 == NULL)) {

				if ((k & OBSTRUCT_MASK) != 0) {
				   float sdist = Obsinfo[ds->layer][OGRID(gridx,
						gridy, ds->layer)];

				   // If the point is marked as close to an
				   // obstruction, we can declare this an
				   // offset tap if we are not on a corner.
				   // Because we cannot define both an offset
				   // and a stub simultaneously, if the distance
				   // to clear the obstruction does not make the
				   // route reach the tap, then we mark the grid
				   // position as unroutable.

				   if (dy >= (ds->y1 - xdist) &&
						dy <= (ds->y2 + xdist)) {
				      if ((dx >= ds->x2) &&
						((k & OBSTRUCT_MASK) == OBSTRUCT_E)) {
				         dist = sdist - LefGetRouteKeepout(ds->layer);
					 if ((dx - ds->x2 + dist) < xdist)
				 	    dir = STUBROUTE_EW | OFFSET_TAP;
				      }
				      else if ((dx <= ds->x1) &&
						((k & OBSTRUCT_MASK) == OBSTRUCT_W)) {
				         dist = LefGetRouteKeepout(ds->layer) - sdist;
					 if ((ds->x1 - dx - dist) < xdist)
				            dir = STUBROUTE_EW | OFFSET_TAP;
				      }
			 	   }	
				   if (dx >= (ds->x1 - xdist) &&
						dx <= (ds->x2 + xdist)) {
				      if ((dy >= ds->y2) &&
						((k & OBSTRUCT_MASK) == OBSTRUCT_N)) {
				         dist = sdist - LefGetRouteKeepout(ds->layer);
					 if ((dy - ds->y2 + dist) < xdist)
				            dir = STUBROUTE_NS | OFFSET_TAP;
				      }
				      else if ((dy <= ds->y1) &&
						((k & OBSTRUCT_MASK) == OBSTRUCT_S)) {
				         dist = LefGetRouteKeepout(ds->layer) - sdist;
					 if ((ds->y1 - dy - dist) < xdist)
				            dir = STUBROUTE_NS | OFFSET_TAP;
				      }
				   }
				   // Otherwise, dir is left as STUBROUTE_X
				}
				else {

				   // Cleanly unobstructed area.  Define stub
				   // route from point to tap, with a route width
				   // overlap if necessary to avoid a DRC width
				   // violation.

				   if ((dx >= ds->x2) &&
					((dx - ds->x2) > (dy - ds->y2)) &&
					((dx - ds->x2) > (ds->y1 - dy))) {
				      // West-pointing stub
				      if ((dy - ds->y2) <= xdist &&
					  (ds->y1 - dy) <= xdist) {
					 // Within reach of tap rectangle
					 dir = STUBROUTE_EW;
					 dist = ds->x2 - dx;
					 if (dy < (ds->y2 - xdist) &&
						dy > (ds->y1 + xdist)) {
					    if (dx < ds->x2 + xdist) dist = 0.0;
					 }
					 else {
					    dist -= 2.0 * xdist;
					 }
				      }
				   }
				   else if ((dx <= ds->x1) &&
					((ds->x1 - dx) > (dy - ds->y2)) &&
					((ds->x1 - dx) > (ds->y1 - dy))) {
				      // East-pointing stub
				      if ((dy - ds->y2) <= xdist &&
					  (ds->y1 - dy) <= xdist) {
					 // Within reach of tap rectangle
					 dir = STUBROUTE_EW;
					 dist = ds->x1 - dx;
					 if (dy < (ds->y2 - xdist) &&
						dy > (ds->y1 + xdist)) {
					    if (dx > ds->x1 - xdist) dist = 0.0;
					 }
					 else {
					    dist += 2.0 * xdist;
					 }
				      }
				   }
				   else if ((dy >= ds->y2) &&
					((dy - ds->y2) > (dx - ds->x2)) &&
					((dy - ds->y2) > (ds->x1 - dx))) {
				      // South-pointing stub
				      if ((dx - ds->x2) <= xdist &&
					  (ds->x1 - dx) <= xdist) {
					 // Within reach of tap rectangle
					 dir = STUBROUTE_NS;
					 dist = ds->y2 - dy;
					 if (dx < (ds->x2 - xdist) &&
						dx > (ds->x1 + xdist)) {
					    if (dy < ds->y2 + xdist) dist = 0.0;
					 }
					 else {
					    dist -= 2.0 * xdist;
					 }
				      }
				   }
				   else if ((dy <= ds->y1) &&
					((ds->y1 - dy) > (dx - ds->x2)) &&
					((ds->y1 - dy) > (ds->x1 - dx))) {
				      // North-pointing stub
				      if ((dx - ds->x2) <= xdist &&
					  (ds->x1 - dx) <= xdist) {
					 // Within reach of tap rectangle
					 dir = STUBROUTE_NS;
					 dist = ds->y1 - dy;
					 if (dx < (ds->x2 - xdist) &&
						dx > (ds->x1 + xdist)) {
					    if (dy > ds->y1 - xdist) dist = 0.0;
					 }
					 else {
					    dist += 2.0 * xdist;
					 }
				      }
				   }

				   if (dir == STUBROUTE_X) {

				      // Outside of pin at a corner.  First, if one
				      // direction is too far away to connect to a
				      // pin, then we must route the other direction.

				      if (dx < ds->x1 - xdist || dx > ds->x2 + xdist) {
				         if (dy >= ds->y1 - xdist &&
							dy <= ds->y2 + xdist) {
				            dir = STUBROUTE_EW;
				            dist = (float)(((ds->x1 + ds->x2) / 2.0)
							- dx);
					 }
				      }
				      else if (dy < ds->y1 - xdist ||
							dy > ds->y2 + xdist) {
				         dir = STUBROUTE_NS;
				         dist = (float)(((ds->y1 + ds->y2) / 2.0) - dy);
				      }

				      // Otherwise we are too far away at a diagonal
				      // to reach the pin by moving in any single
				      // direction.  To be pedantic, we could define
				      // some jogged stub, but for now, we just call
				      // the point unroutable (leave dir = STUBROUTE_X)
				   }
				}

				// Stub distances of <= 1/2 route width are useless
				if (dir == STUBROUTE_NS || dir == STUBROUTE_EW)
				   if (fabs(dist) < (xdist + EPS)) {
				      dir = 0;
				      dist = 0.0;
				   }

				if ((k < Numnets) && (dir != STUBROUTE_X)) {
				   Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
				   	= (Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
					  & BLOCKED_MASK) | (u_int)g->netnum[i] | dir; 
				   Nodeloc[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= node;
				   Nodesav[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= node;
				}
				else if ((Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
					& NO_NET) != 0) {
				   // Keep showing an obstruction, but add the
				   // direction info and log the stub distance.
				   Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
					|= dir;
				}
				else {
				   Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
					|= (dir | (g->netnum[i] & ROUTED_NET_MASK));
				}
				Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= dist;
			    }
			    else {
			       int othernet = (k & ROUTED_NET_MASK);

			       if (othernet != 0 && othernet != (u_int)node->netnum) {

			          // This location is too close to two different
				  // node terminals and should not be used
				  // (NOTE:  This routine also disables
				  // catecorner positions that might pass
				  // euclidean distance DRC checks, so it's
				  // more restrictive than necessary.)

				  // If there is a stub, then we can't specify
				  // an offset tap, so just disable it.  If
				  // there is already an offset, then just
				  // disable it.  Otherwise, check if othernet
				  // could be routed using a tap offset.

				  // To avoid having to check all nearby
				  // geometry, place a restriction that the
				  // next grid point in the direction of the
				  // offset must be free (not a tap point of
				  // any net, including this one).  That is
				  // still "more restrictive than necessary",
				  // but since the alternative is an efficient
				  // area search for interacting geometry, this
				  // restriction will stand until an example
				  // comes along that requires the detailed
				  // search.
				
				  if ((k & (STUBROUTE_X | OFFSET_TAP)) != 0)
				     disable_gridpos(gridx, gridy, ds->layer);
				  else if (Nodesav[ds->layer][OGRID(gridx, gridy,
						ds->layer)] != NULL) {

				     u_char no_offsets = TRUE;
				     int offset_net;

				     // By how much would a tap need to be moved
				     // to clear the obstructing geometry?

				     // Check tap to right

				     if ((dx > ds->x2) && (gridx <
						NumChannelsX[ds->layer] - 1)) {
					offset_net = Obs[ds->layer][OGRID(gridx + 1,
						gridy, ds->layer)];
					if (offset_net == 0 || offset_net == othernet) {
					   xdist = 0.5 * LefGetViaWidth(ds->layer,
							ds->layer, 0);
					   dist = ds->x2 - dx + xdist +
							LefGetRouteSpacing(ds->layer);
					   dir = (STUBROUTE_EW | OFFSET_TAP);
					   Stub[ds->layer][OGRID(gridx, gridy,
							ds->layer)] = dist;
					   Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
							|= dir;
					   no_offsets = FALSE;
					}
				     }

				     // Check tap to left

				     if ((dx < ds->x1) && (gridx > 0)) {
					offset_net = Obs[ds->layer][OGRID(gridx - 1,
						gridy, ds->layer)];
					if (offset_net == 0 || offset_net == othernet) {
					   xdist = 0.5 * LefGetViaWidth(ds->layer,
							ds->layer, 0);
					   dist = ds->x1 - dx - xdist -
							LefGetRouteSpacing(ds->layer);
					   dir = (STUBROUTE_EW | OFFSET_TAP);
					   Stub[ds->layer][OGRID(gridx, gridy,
							ds->layer)] = dist;
					   Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
							|= dir;
					   no_offsets = FALSE;
					}
				     }

				     // Check tap up

				     if ((dy > ds->y2) && (gridy <
						NumChannelsY[ds->layer] - 1)) {
					offset_net = Obs[ds->layer][OGRID(gridx,
						gridy + 1, ds->layer)];
					if (offset_net == 0 || offset_net == othernet) {
					   xdist = 0.5 * LefGetViaWidth(ds->layer,
							ds->layer, 1);
					   dist = ds->y2 - dy + xdist +
							LefGetRouteSpacing(ds->layer);
					   dir = (STUBROUTE_NS | OFFSET_TAP);
					   Stub[ds->layer][OGRID(gridx, gridy,
							ds->layer)] = dist;
					   Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
							|= dir;
					   no_offsets = FALSE;
					}
				     }

				     // Check tap to left

				     if ((dy < ds->y1) && (gridy > 0)) {
					offset_net = Obs[ds->layer][OGRID(gridx,
						gridy - 1, ds->layer)];
					if (offset_net == 0 || offset_net == othernet) {
					   xdist = 0.5 * LefGetViaWidth(ds->layer,
							ds->layer, 1);
					   dist = ds->y1 - dy - xdist -
							LefGetRouteSpacing(ds->layer);
					   dir = (STUBROUTE_NS | OFFSET_TAP);
					   Stub[ds->layer][OGRID(gridx, gridy,
							ds->layer)] = dist;
					   Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
							|= dir;
					   no_offsets = FALSE;
					}
				     }

				     // No offsets were possible, so disable the
				     // position

				     if (no_offsets == TRUE)
				        disable_gridpos(gridx, gridy, ds->layer);
				  }
				  else
				     disable_gridpos(gridx, gridy, ds->layer);
			       }

			       /* If we are on a layer > 0, then this geometry	*/
			       /* may block or partially block a pin on layer	*/
			       /* zero.  Mark this point as belonging to the	*/
			       /* net with a stub route to it.			*/
			       /* NOTE:  This is possibly too restrictive.	*/
			       /* May want to force a tap offset for vias on	*/
			       /* layer zero. . .				*/

			       if ((ds->layer > 0) && (n2 != NULL) && (n2->netnum
					!= node->netnum) && ((othernet == 0) ||
					(othernet == (u_int)node->netnum))) {

				  xdist = 0.5 * LefGetViaWidth(ds->layer, ds->layer, 0);
				  if ((dy + xdist + LefGetRouteSpacing(ds->layer) >
					ds->y1) && (dy + xdist < ds->y1)) {
				     if ((dx - xdist < ds->x2) &&
						(dx + xdist > ds->x1) &&
						(Stub[ds->layer][OGRID(gridx, gridy,
						ds->layer)] == 0.0)) {
					Stub[ds->layer][OGRID(gridx, gridy,
						ds->layer)] = ds->y1 - dy;
					Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
				   		= (Obs[ds->layer][OGRID(gridx, gridy,
						ds->layer)] & BLOCKED_MASK) |
						node->netnum | STUBROUTE_NS;
					Nodeloc[ds->layer][OGRID(gridx, gridy,
						ds->layer)] = node;
					Nodesav[ds->layer][OGRID(gridx, gridy,
						ds->layer)] = node;
				     }
				  }
				  if ((dy - xdist - LefGetRouteSpacing(ds->layer) <
					ds->y2) && (dy - xdist > ds->y2)) {
				     if ((dx - xdist < ds->x2) &&
						(dx + xdist > ds->x1) &&
						(Stub[ds->layer][OGRID(gridx, gridy,
						ds->layer)] == 0.0)) {
					Stub[ds->layer][OGRID(gridx, gridy,
						ds->layer)] = ds->y2 - dy;
					Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
				   		= (Obs[ds->layer][OGRID(gridx, gridy,
						ds->layer)] & BLOCKED_MASK) |
						node->netnum | STUBROUTE_NS;
					Nodeloc[ds->layer][OGRID(gridx, gridy,
						ds->layer)] = node;
					Nodesav[ds->layer][OGRID(gridx, gridy,
						ds->layer)] = node;
				     }
				  }

				  xdist = 0.5 * LefGetViaWidth(ds->layer, ds->layer, 1);
				  if ((dx + xdist + LefGetRouteSpacing(ds->layer) >
					ds->x1) && (dx + xdist < ds->x1)) {
				     if ((dy - xdist < ds->y2) &&
						(dy + xdist > ds->y1) &&
						(Stub[ds->layer][OGRID(gridx, gridy,
						ds->layer)] == 0.0)) {
					Stub[ds->layer][OGRID(gridx, gridy,
						ds->layer)] = ds->x1 - dx;
					Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
				   		= (Obs[ds->layer][OGRID(gridx, gridy,
						ds->layer)] & BLOCKED_MASK) |
						node->netnum | STUBROUTE_EW;
					Nodeloc[ds->layer][OGRID(gridx, gridy,
						ds->layer)] = node;
					Nodesav[ds->layer][OGRID(gridx, gridy,
						ds->layer)] = node;
				     }
				  }
				  if ((dx - xdist - LefGetRouteSpacing(ds->layer) <
					ds->x2) && (dx - xdist > ds->x2)) {
				     if ((dy - xdist < ds->y2) &&
						(dy + xdist > ds->y1) &&
						(Stub[ds->layer][OGRID(gridx, gridy,
						ds->layer)] == 0.0)) {
					Stub[ds->layer][OGRID(gridx, gridy,
						ds->layer)] = ds->x2 - dx;
					Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
				   		= (Obs[ds->layer][OGRID(gridx, gridy,
						ds->layer)] & BLOCKED_MASK) |
						node->netnum | STUBROUTE_EW;
					Nodeloc[ds->layer][OGRID(gridx, gridy,
						ds->layer)] = node;
					Nodesav[ds->layer][OGRID(gridx, gridy,
						ds->layer)] = node;
				     }
				  }
			       }
			    }
		         }
		         gridy++;
		      }
		   }
		   gridx++;
		}
	     }
	  }
       }
    }

} /* void create_obstructions_from_nodes( void ) */

/*--------------------------------------------------------------*/
/* tap_to_tap_interactions()					*/
/*								*/
/*  Similar to create_obstructions_from_nodes(), but looks at	*/
/*  each node's tap geometry, looks at every grid point in a	*/
/*  wider area surrounding the tap.  If any other node has an	*/
/*  offset that would place it too close to this node's	tap	*/
/*  geometry, then we mark the other node as unroutable at that	*/
/*  grid point.							*/
/*--------------------------------------------------------------*/

void tap_to_tap_interactions()
{
    NODE node;
    GATE g;
    DSEG ds;
    struct dseg_ de;
    int mingridx, mingridy, maxgridx, maxgridy;
    int i, gridx, gridy, net, orignet, offset;
    double dx, dy;
    float dist;
    u_char errbox;

    double deltax[MAX_LAYERS];
    double deltay[MAX_LAYERS];

    for (i = 0; i < Num_layers; i++) {
	deltax[i] = 0.5 * LefGetViaWidth(i, i, 0) + LefGetRouteSpacing(i);
	// NOTE:  Extra space is how much vias get shifted relative to the
	// specified offset distance to account for the via size being larger
	// than the route width.
	// deltax[i] += 0.5 * (LefGetViaWidth(i, i, 0) - LefGetRouteSpacing(i));
	deltay[i] = 0.5 * LefGetViaWidth(i, i, 1) + LefGetRouteSpacing(i);
	// deltay[i] += 0.5 * (LefGetViaWidth(i, i, 1) - LefGetRouteSpacing(i));
    }

    for (g = Nlgates; g; g = g->next) {
       for (i = 0; i < g->nodes; i++) {
	  net = g->netnum[i];
	  if (net != 0) {

	     // Get the node record associated with this pin.
	     node = g->noderec[i];

             for (ds = g->taps[i]; ds; ds = ds->next) {

		mingridx = (int)((ds->x1 - Xlowerbound) / PitchX[ds->layer]) - 1;
		if (mingridx < 0) mingridx = 0;
		maxgridx = (int)((ds->x2 - Xlowerbound) / PitchX[ds->layer]) + 2;
		if (maxgridx >= NumChannelsX[ds->layer])
		   maxgridx = NumChannelsX[ds->layer] - 1;
		mingridy = (int)((ds->y1 - Ylowerbound) / PitchY[ds->layer]) - 1;
		if (mingridy < 0) mingridy = 0;
		maxgridy = (int)((ds->y2 - Ylowerbound) / PitchY[ds->layer]) + 2;
		if (maxgridy >= NumChannelsY[ds->layer])
		   maxgridy = NumChannelsY[ds->layer] - 1;

		for (gridx = mingridx; gridx <= maxgridx; gridx++) {
		   for (gridy = mingridy; gridy <= maxgridy; gridy++) {

		      /* Is there an offset tap at this position, and	*/
		      /* does it belong to a net that is != net?	*/

		      orignet = Obs[ds->layer][OGRID(gridx, gridy, ds->layer)];
		      if (orignet & OFFSET_TAP) {
			 offset = orignet & PINOBSTRUCTMASK;
			 orignet &= ROUTED_NET_MASK;
			 if (orignet != net) {

		            dx = (gridx * PitchX[ds->layer]) + Xlowerbound;
		            dy = (gridy * PitchY[ds->layer]) + Ylowerbound;

			    dist = Stub[ds->layer][OGRID(gridx, gridy, ds->layer)];

			    /* "de" is the bounding box of a via placed	  */
			    /* at (gridx, gridy) and offset as specified. */
			    /* Expanded by metal spacing requirement.	  */

			    de.x1 = dx - deltax[ds->layer];
			    de.x2 = dx + deltax[ds->layer];
			    de.y1 = dy - deltay[ds->layer];
			    de.y2 = dy + deltay[ds->layer];

			    if (offset == (STUBROUTE_NS | OFFSET_TAP)) {
			       de.y1 += dist;
			       de.y2 += dist;
			    }
			    else if (offset == (STUBROUTE_EW | OFFSET_TAP)) {
			       de.x1 += dist;
			       de.x2 += dist;
			    }

			    // Shrink by EPS to avoid roundoff errors
			    de.x1 += EPS;
			    de.x2 -= EPS;
			    de.y1 += EPS;
			    de.y2 -= EPS;

			    /* Does the via bounding box interact with	*/
			    /* the tap geometry?			*/

			    if ((de.x1 < ds->x2) && (ds->x1 < de.x2) &&
					(de.y1 < ds->y2) && (ds->y1 < de.y2))
			       disable_gridpos(gridx, gridy, ds->layer);
			 }
		      }
		   }
		}
	     }
	  }
       }
    }
}

/*--------------------------------------------------------------*/
/* make_routable()						*/
/*								*/
/*  In the case that a node can't be routed because it has no	*/
/*  available tap points, but there is tap geometry recorded	*/
/*  for the node, then take the first available grid location	*/
/*  near the tap.  This, of course, bypasses all of qrouter's	*/
/*  DRC checks.  But it is only meant to be a stop-gap measure	*/
/*  to get qrouter to complete all routes, and may work in	*/
/*  cases where, say, the tap passes euclidean rules but not	*/
/*  manhattan rules.						*/
/*--------------------------------------------------------------*/

void
make_routable(NODE node)
{
    GATE g;
    DSEG ds;
    int i, gridx, gridy, net;
    double dx, dy;

    /* The database is not organized to find tap points	*/
    /* from nodes, so we have to search for the node.	*/
    /* Fortunately this routine isn't normally called.	*/

    for (g = Nlgates; g; g = g->next) {
       for (i = 0; i < g->nodes; i++) {
	  if (g->noderec[i] == node) {
             for (ds = g->taps[i]; ds; ds = ds->next) {
		gridx = (int)((ds->x1 - Xlowerbound) / PitchX[ds->layer]) - 1;
		while (1) {
		   dx = (gridx * PitchX[ds->layer]) + Xlowerbound;
		   if (dx > ds->x2 || gridx >= NumChannelsX[ds->layer]) break;
		   else if (dx >= ds->x1 && gridx >= 0) {
		      gridy = (int)((ds->y1 - Ylowerbound) / PitchY[ds->layer]) - 1;
		      while (1) {
		         dy = (gridy * PitchY[ds->layer]) + Ylowerbound;
		         if (dy > ds->y2 || gridy >= NumChannelsY[ds->layer]) break;

			 // Area inside defined pin geometry

			 if (dy > ds->y1 && gridy >= 0) {
			    int orignet = Obs[ds->layer][OGRID(gridx,
					gridy, ds->layer)];

			    if (orignet & NO_NET) {
				Obs[ds->layer][OGRID(gridx, gridy, ds->layer)] =
					g->netnum[i];
				Nodeloc[ds->layer][OGRID(gridx, gridy, ds->layer)] =
					node;
				Nodesav[ds->layer][OGRID(gridx, gridy, ds->layer)] =
					node;
				return;
			    }
			 }
			 gridy++;
		      }
		   }
		   gridx++;
	        }
	     }
	  }
       }
    }
}

/*--------------------------------------------------------------*/
/* adjust_stub_lengths()					*/
/*								*/
/*  Makes an additional pass through the tap and obstruction	*/
/*  databases, checking geometry against the potential stub	*/
/*  routes for DRC spacing violations.  Adjust stub routes as	*/
/*  necessary to resolve the DRC error(s).			*/
/*								*/
/*  ARGS: none.							*/
/*  RETURNS: nothing						*/
/*  SIDE EFFECTS: none						*/
/*  AUTHOR:  Tim Edwards, April 2013				*/
/*--------------------------------------------------------------*/

void adjust_stub_lengths()
{
    NODE node, n2;
    GATE g;
    DPOINT dp;
    DSEG ds, ds2;
    struct dseg_ dt, de;
    u_int dir, k;
    int i, gx, gy, gridx, gridy, net, orignet;
    double dx, dy, wx, wy, s, dd;
    float dist;
    u_char errbox;

    // For each node terminal (gate pin), look at the surrounding grid points.
    // If any define a stub route or an offset, check if the stub geometry
    // or offset geometry would create a DRC spacing violation.  If so, adjust
    // the stub route to resolve the error.  If the error cannot be resolved,
    // mark the position as unroutable.  If it is the ONLY grid point accessible
    // to the pin, keep it as-is and flag a warning.

    // Unlike blockage-finding routines, which look in an area of a size equal
    // to the DRC interaction distance around a tap rectangle, this routine looks
    // out one grid pitch in each direction, to catch information about stubs that
    // may terminate within a DRC interaction distance of the tap rectangle.

    for (g = Nlgates; g; g = g->next) {
       for (i = 0; i < g->nodes; i++) {
	  if (g->netnum[i] != 0) {

	     // Get the node record associated with this pin.
	     node = g->noderec[i];
	     if (node == NULL) continue;

	     // Work through each rectangle in the tap geometry

             for (ds = g->taps[i]; ds; ds = ds->next) {
		wx = 0.5 * LefGetViaWidth(ds->layer, ds->layer, 0);
		wy = 0.5 * LefGetViaWidth(ds->layer, ds->layer, 1);
		s = LefGetRouteSpacing(ds->layer);
		gridx = (int)((ds->x1 - Xlowerbound - PitchX[ds->layer])
			/ PitchX[ds->layer]) - 1;
		while (1) {
		   dx = (gridx * PitchX[ds->layer]) + Xlowerbound;
		   if (dx > (ds->x2 + PitchX[ds->layer]) ||
				gridx >= NumChannelsX[ds->layer]) break;
		   else if (dx >= (ds->x1 - PitchX[ds->layer]) && gridx >= 0) {
		      gridy = (int)((ds->y1 - Ylowerbound - PitchY[ds->layer])
				/ PitchY[ds->layer]) - 1;
		      while (1) {
		         dy = (gridy * PitchY[ds->layer]) + Ylowerbound;
		         if (dy > (ds->y2 + PitchY[ds->layer]) ||
				gridy >= NumChannelsY[ds->layer]) break;
		         if (dy >= (ds->y1 - PitchY[ds->layer]) && gridy >= 0) {

			     orignet = Obs[ds->layer][OGRID(gridx, gridy, ds->layer)];

			     // Ignore this location if it is assigned to another
			     // net, or is assigned to NO_NET.

			     if ((orignet & ROUTED_NET_MASK) != node->netnum) {
				gridy++;
				continue;
			     }

			     // STUBROUTE_X are unroutable;  leave them alone
			     if ((orignet & PINOBSTRUCTMASK) == STUBROUTE_X) {
				gridy++;
				continue;
			     }

			     // define a route box around the grid point

			     errbox = FALSE;
			     dt.x1 = dx - wx;
			     dt.x2 = dx + wx;
			     dt.y1 = dy - wy;
			     dt.y2 = dy + wy;

			     dist = Stub[ds->layer][OGRID(gridx, gridy, ds->layer)];

			     // adjust the route box according to the stub
			     // or offset geometry, provided that the stub
			     // is longer than the route box.

			     if (orignet & OFFSET_TAP) {
				if (orignet & STUBROUTE_EW) {
				   dt.x1 += dist;
				   dt.x2 += dist;
				}
				else if (orignet & STUBROUTE_NS) {
				   dt.y1 += dist;
				   dt.y2 += dist;
				}
			     }
			     else if (orignet & PINOBSTRUCTMASK) {
				if (orignet & STUBROUTE_EW) {
				   if (dist > EPS) {
				      if (dx + dist > dt.x2)
				 	 dt.x2 = dx + dist;
				   }
				   else {
				      if (dx + dist < dt.x1)
					 dt.x1 = dx + dist;
				   }
				}
				else if (orignet & STUBROUTE_NS) {
				   if (dist > EPS) {
				      if (dy + dist > dt.y2)
					 dt.y2 = dy + dist;
				   }
				   else {
				      if (dy + dist < dt.y1)
					 dt.y1 = dy + dist;
				   }
				}
			     }

			     de = dt;

			     // check for DRC spacing interactions between
			     // the tap box and the route box

			     if ((dt.y1 - ds->y2) > EPS && (dt.y1 - ds->y2) < s) {
				if (ds->x2 > (dt.x1 - s) && ds->x1 < (dt.x2 + s)) {
				   de.y2 = dt.y1;
				   de.y1 = ds->y2;
				   if (ds->x2 + s < dt.x2) de.x2 = ds->x2 + s;
				   if (ds->x1 - s > dt.x1) de.x1 = ds->x1 - s;
				   errbox = TRUE;
				}
			     }
			     else if ((ds->y1 - dt.y2) > EPS && (ds->y1 - dt.y2) < s) {
				if (ds->x2 > (dt.x1 - s) && ds->x1 < (dt.x2 + s)) {
				   de.y1 = dt.y2;
				   de.y2 = ds->y1;
				   if (ds->x2 + s < dt.x2) de.x2 = ds->x2 + s;
				   if (ds->x1 - s > dt.x1) de.x1 = ds->x1 - s;
				   errbox = TRUE;
				}
			     }

			     if ((dt.x1 - ds->x2) > EPS && (dt.x1 - ds->x2) < s) {
				if (ds->y2 > (dt.y1 - s) && ds->y1 < (dt.y2 + s)) {
				   de.x2 = dt.x1;
				   de.x1 = ds->x2;
				   if (ds->y2 + s < dt.y2) de.y2 = ds->y2 + s;
				   if (ds->y1 - s > dt.y1) de.y1 = ds->y1 - s;
				   errbox = TRUE;
				}
			     }
			     else if ((ds->x1 - dt.x2) > EPS && (ds->x1 - dt.x2) < s) {
				if (ds->y2 > (dt.y1 - s) && ds->y1 < (dt.y2 + s)) {
				   de.x1 = dt.x2;
				   de.x2 = ds->x1;
				   if (ds->y2 + s < dt.y2) de.y2 = ds->y2 + s;
				   if (ds->y1 - s > dt.y1) de.y1 = ds->y1 - s;
				   errbox = TRUE;
				}
			     }

			     if (errbox == TRUE) {
	
			        // Chop areas off the error box that are covered by
			        // other taps of the same port.

			        for (ds2 = g->taps[i]; ds2; ds2 = ds2->next) {
				   if (ds2 == ds) continue;
				   if (ds2->layer != ds->layer) continue;

				   if (ds2->x1 <= de.x1 && ds2->x2 >= de.x2 &&
					ds2->y1 <= de.y1 && ds2->y2 >= de.y2) {
				      errbox = FALSE;	// Completely covered
				      break;
				   }

				   // Look for partial coverage.  Note that any
				   // change can cause a change in the original
				   // two conditionals, so we have to keep
				   // evaluating those conditionals.

				   if (ds2->x1 < de.x2 && ds2->x2 > de.x1)
				      if (ds2->y1 < de.y2 && ds2->y2 > de.y1)
					 // if (ds2->x1 < de.x1 - EPS &&
					 if (ds2->x1 < de.x1 + EPS &&
							ds2->x2 < de.x2 - EPS) {
					    de.x1 = ds2->x2;
					    if (ds2->x2 >= ds->x2) errbox = FALSE;
					 }

				   if (ds2->x1 < de.x2 && ds2->x2 > de.x1)
				      if (ds2->y1 < de.y2 && ds2->y2 > de.y1)
					 // if (ds2->x2 > de.x2 + EPS &&
					 if (ds2->x2 > de.x2 - EPS &&
							ds2->x1 > de.x1 + EPS) {
					    de.x2 = ds2->x1;
					    if (ds2->x1 <= ds->x1) errbox = FALSE;
					 }

				   if (ds2->x1 < de.x2 && ds2->x2 > de.x1)
				      if (ds2->y1 < de.y2 && ds2->y2 > de.y1)
					 // if (ds2->y1 < de.y1 - EPS &&
					 if (ds2->y1 < de.y1 + EPS &&
							ds2->y2 < de.y2 - EPS) {
					    de.y1 = ds2->y2;
					    if (ds2->y2 >= ds->y2) errbox = FALSE;
					 }

				   if (ds2->x1 < de.x2 && ds2->x2 > de.x1)
				      if (ds2->y1 < de.y2 && ds2->y2 > de.y1)
					 // if (ds2->y2 > de.y2 + EPS &&
					 if (ds2->y2 > de.y2 - EPS &&
							ds2->y1 > de.y1 + EPS) {
					    de.y2 = ds2->y1;
					    if (ds2->y1 <= ds->y1) errbox = FALSE;
					 }
				}
			     }

			     // Any area left over is a potential DRC error.

			     if ((de.x2 <= de.x1) || (de.y2 <= de.y1))
				errbox = FALSE;
		
			     if (errbox == TRUE) {

				// Create stub route to cover error box, or
				// if possible, stretch existing stub route
				// to cover error box.

				// Allow EW stubs to be changed to NS stubs and
				// vice versa if the original stub length was less
				// than a route width.  This means the grid position
				// makes contact without the stub.  Moving the stub
				// to another side should not create an error.

				// NOTE:  Changed 4/29/13;  direction of stub will
				// be changed even though it might create an error
				// in the other direction;  it can't do worse.
				// But, the case should be re-run to check (to-do)

				// NOTE 2: Changed again 1/8/14;  error box must
				// touch ds geometry.
				// Changed again 2/5/14:  error box must touch
				// ds geometry by more than just a point.  <=
				// changed to < and >= changed to >

				/* if (de.x2 > dt.x2) { */
				if ((de.x2 > dt.x2) && (de.y1 < ds->y2) &&
						(de.y2 > ds->y1)) {
				   if ((orignet & PINOBSTRUCTMASK) == 0) {
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						|= STUBROUTE_EW;
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.x2 - dx;
				      errbox = FALSE;
				   }
				   else if ((orignet & PINOBSTRUCTMASK) == STUBROUTE_EW
						&& (dist > 0)) {
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.x2 - dx;
				      errbox = FALSE;
				   }
				   else if ((orignet & PINOBSTRUCTMASK) ==
						STUBROUTE_NS) {
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						&= ~STUBROUTE_NS;
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						|= STUBROUTE_EW;
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.x2 - dx;
				      errbox = FALSE;
				   }
				}
				/* else if (de.x1 < dt.x1) { */
				else if ((de.x1 < dt.x1) && (de.y1 < ds->y2) &&
						(de.y2 > ds->y1)) {
				   if ((orignet & PINOBSTRUCTMASK) == 0) {
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						|= STUBROUTE_EW;
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.x1 - dx;
				      errbox = FALSE;
				   }
				   else if ((orignet & PINOBSTRUCTMASK) == STUBROUTE_EW
						&& (dist < 0)) {
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.x1 - dx;
				      errbox = FALSE;
				   }
				   else if ((orignet & PINOBSTRUCTMASK) ==
						STUBROUTE_NS) {
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						&= ~STUBROUTE_NS;
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						|= STUBROUTE_EW;
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.x1 - dx;
				      errbox = FALSE;
				   }
				}
				/* else if (de.y2 > dt.y2) { */
				else if ((de.y2 > dt.y2) && (de.x1 < ds->x2) &&
					(de.x2 > ds->x1)) {
				   if ((orignet & PINOBSTRUCTMASK) == 0) {
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						|= STUBROUTE_NS;
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.y2 - dy;
				      errbox = FALSE;
				   }
				   else if ((orignet & PINOBSTRUCTMASK) == STUBROUTE_NS
						&& (dist > 0)) {
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.y2 - dy;
				      errbox = FALSE;
				   }
				   else if ((orignet & PINOBSTRUCTMASK) ==
						STUBROUTE_EW) {
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						&= ~STUBROUTE_EW;
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						|= STUBROUTE_NS;
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.y2 - dy;
				      errbox = FALSE;
				   }
				}
				/* else if (de.y1 < dt.y1) { */
				else if ((de.y1 < dt.y1) && (de.x1 < ds->x2) &&
					(de.x2 > ds->x1)) {
				   if ((orignet & PINOBSTRUCTMASK) == 0) {
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						|= STUBROUTE_NS;
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.y1 - dy;
				      errbox = FALSE;
				   }
				   else if ((orignet & PINOBSTRUCTMASK) == STUBROUTE_NS
						&& (dist < 0)) {
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.y1 - dy;
				      errbox = FALSE;
				   }
				   else if ((orignet & PINOBSTRUCTMASK) ==
						STUBROUTE_EW) {
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						&= ~STUBROUTE_EW;
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						|= STUBROUTE_NS;
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.y1 - dy;
				      errbox = FALSE;
				   }
				}

				// Where the error box did not touch the stub
				// route, there is assumed to be no error.

				if (errbox == TRUE)
				   if ((de.x2 > dt.x2) || (de.x1 < dt.x1) ||
					(de.y2 > dt.y2) || (de.y1 < dt.y1))
				      errbox = FALSE;

				if (errbox == TRUE) {
				   // Unroutable position, so mark it unroutable
			           Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
					|= STUBROUTE_X;
				}
			     }
		         }
		         gridy++;
		      }
		   }
		   gridx++;
		}
	     }
	  }
       }
    }

} /* void adjust_stub_lengths() */

/*--------------------------------------------------------------*/
/* block_route()						*/
/*								*/
/*  Mark a specific length along the route tracks as unroutable	*/
/*  by finding the grid point in the direction indicated, and	*/
/*  setting the appropriate block bit in the Obs[] array for	*/
/*  that position.  The original grid point is marked as	*/
/*  unroutable in the opposite direction, for symmetry.		*/
/*--------------------------------------------------------------*/

void
block_route(int x, int y, int lay, u_char dir)
{
   int bx, by, bl, ob;

   bx = x;
   by = y;
   bl = lay;

   switch (dir) {
      case NORTH:
	 if (y == NumChannelsY[lay] - 1) return;
	 by = y + 1;
	 break;
      case SOUTH:
	 if (y == 0) return;
	 by = y - 1;
	 break;
      case EAST:
	 if (x == NumChannelsX[lay] - 1) return;
	 bx = x + 1;
	 break;
      case WEST:
	 if (x == 0) return;
	 bx = x - 1;
	 break;
      case UP:
	 if (lay == Num_layers - 1) return;
	 bl = lay + 1;
	 break;
      case DOWN:
	 if (lay == 0) return;
	 bl = lay - 1;
	 break;
   }
   
   ob = Obs[bl][OGRID(bx, by, bl)];

   if ((ob & NO_NET) != 0) return;

   switch (dir) {
      case NORTH:
	 Obs[bl][OGRID(bx, by, bl)] |= BLOCKED_S;
	 Obs[lay][OGRID(x, y, lay)] |= BLOCKED_N;
	 break;
      case SOUTH:
	 Obs[bl][OGRID(bx, by, bl)] |= BLOCKED_N;
	 Obs[lay][OGRID(x, y, lay)] |= BLOCKED_S;
	 break;
      case EAST:
	 Obs[bl][OGRID(bx, by, bl)] |= BLOCKED_W;
	 Obs[lay][OGRID(x, y, lay)] |= BLOCKED_E;
	 break;
      case WEST:
	 Obs[bl][OGRID(bx, by, bl)] |= BLOCKED_E;
	 Obs[lay][OGRID(x, y, lay)] |= BLOCKED_W;
	 break;
      case UP:
	 Obs[bl][OGRID(bx, by, bl)] |= BLOCKED_D;
	 Obs[lay][OGRID(x, y, lay)] |= BLOCKED_U;
	 break;
      case DOWN:
	 Obs[bl][OGRID(bx, by, bl)] |= BLOCKED_U;
	 Obs[lay][OGRID(x, y, lay)] |= BLOCKED_D;
	 break;
   }
}

/*--------------------------------------------------------------*/
/* find_route_blocks() ---					*/
/*								*/
/*	Search tap geometry for edges that cause DRC spacing	*/
/*	errors with route edges.  This specifically checks	*/
/*	edges of the route tracks, not the intersection points.	*/
/*	If a tap would cause an error with a route segment,	*/
/*	the grid points on either end of the segment are	*/
/*	flagged to prevent generating a route along that	*/
/*	specific segment.					*/
/*--------------------------------------------------------------*/

void
find_route_blocks()
{
   NODE node;
   GATE g;
   // DPOINT dp;
   DSEG ds, ds2;
   struct dseg_ dt, de;
   int i, gridx, gridy;
   double dx, dy, w, v, s, u;
   float dist;
   u_char errbox;

   for (g = Nlgates; g; g = g->next) {
      for (i = 0; i < g->nodes; i++) {
	 if (g->netnum[i] != 0) {

	    // Get the node record associated with this pin.
	    node = g->noderec[i];

	    // Work through each rectangle in the tap geometry

            for (ds = g->taps[i]; ds; ds = ds->next) {
	       w = 0.5 * LefGetRouteWidth(ds->layer);
	       v = 0.5 * LefGetViaWidth(ds->layer, ds->layer, 0);
	       s = LefGetRouteSpacing(ds->layer);

	       // Look west

	       gridx = (int)((ds->x1 - Xlowerbound) / PitchX[ds->layer]);
	       dx = (gridx * PitchX[ds->layer]) + Xlowerbound;
	       dist = ds->x1 - dx - w;
	       if (dist > 0 && dist < s && gridx >= 0) {
		  dt.x1 = dt.x2 = dx;
		  dt.y1 = ds->y1;
		  dt.y2 = ds->y2;

		  // Check for other taps covering this edge
		  // (to do)

		  // Find all grid points affected
	          gridy = (int)((ds->y1 - Ylowerbound - PitchY[ds->layer]) /
				PitchY[ds->layer]);
	          dy = (gridy * PitchY[ds->layer]) + Ylowerbound;
		  while (dy < ds->y1 - s) {
		     dy += PitchY[ds->layer];
		     gridy++;
		  }
		  while (dy < ds->y2 + s) {
		     u = ((Obs[ds->layer][OGRID(gridx, gridy, ds->layer)] &
				PINOBSTRUCTMASK) == STUBROUTE_EW) ? v : w;
		     if (dy + EPS < ds->y2 - u)
			block_route(gridx, gridy, ds->layer, NORTH);
		     if (dy - EPS > ds->y1 + u)
			block_route(gridx, gridy, ds->layer, SOUTH);
		     dy += PitchY[ds->layer];
		     gridy++;
		  }
	       }

	       // Look east

	       gridx = (int)(1.0 + (ds->x2 - Xlowerbound) / PitchX[ds->layer]);
	       dx = (gridx * PitchX[ds->layer]) + Xlowerbound;
	       dist = dx - ds->x2 - w;
	       if (dist > 0 && dist < s && gridx < NumChannelsX[ds->layer]) {
		  dt.x1 = dt.x2 = dx;
		  dt.y1 = ds->y1;
		  dt.y2 = ds->y2;

		  // Check for other taps covering this edge
		  // (to do)

		  // Find all grid points affected
	          gridy = (int)((ds->y1 - Ylowerbound - PitchY[ds->layer]) /
				PitchY[ds->layer]);
	          dy = (gridy * PitchY[ds->layer]) + Ylowerbound;
		  while (dy < ds->y1 - s) {
		     dy += PitchY[ds->layer];
		     gridy++;
		  }
		  while (dy < ds->y2 + s) {
		     u = ((Obs[ds->layer][OGRID(gridx, gridy, ds->layer)] &
				PINOBSTRUCTMASK) == STUBROUTE_EW) ? v : w;
		     if (dy + EPS < ds->y2 - u)
			block_route(gridx, gridy, ds->layer, NORTH);
		     if (dy - EPS > ds->y1 + u)
			block_route(gridx, gridy, ds->layer, SOUTH);
		     dy += PitchY[ds->layer];
		     gridy++;
		  }
	       }

	       // Look south

	       gridy = (int)((ds->y1 - Ylowerbound) / PitchY[ds->layer]);
	       dy = (gridy * PitchY[ds->layer]) + Ylowerbound;
	       dist = ds->y1 - dy - w;
	       if (dist > 0 && dist < s && gridy >= 0) {
		  dt.x1 = ds->x1;
		  dt.x2 = ds->x2;
		  dt.y1 = dt.y2 = dy;

		  // Check for other taps covering this edge
		  // (to do)

		  // Find all grid points affected
	          gridx = (int)((ds->x1 - Xlowerbound - PitchX[ds->layer]) /
				PitchX[ds->layer]);
	          dx = (gridx * PitchX[ds->layer]) + Xlowerbound;
		  while (dx < ds->x1 - s) {
		     dx += PitchX[ds->layer];
		     gridx++;
		  }
		  while (dx < ds->x2 + s) {
		     u = ((Obs[ds->layer][OGRID(gridx, gridy, ds->layer)] &
				PINOBSTRUCTMASK) == STUBROUTE_NS) ? v : w;
		     if (dx + EPS < ds->x2 - u)
			block_route(gridx, gridy, ds->layer, EAST);
		     if (dx - EPS > ds->x1 + u)
			block_route(gridx, gridy, ds->layer, WEST);
		     dx += PitchX[ds->layer];
		     gridx++;
		  }
	       }

	       // Look north

	       gridy = (int)(1.0 + (ds->y2 - Ylowerbound) / PitchY[ds->layer]);
	       dy = (gridy * PitchY[ds->layer]) + Ylowerbound;
	       dist = dy - ds->y2 - w;
	       if (dist > 0 && dist < s && gridy < NumChannelsY[ds->layer]) {
		  dt.x1 = ds->x1;
		  dt.x2 = ds->x2;
		  dt.y1 = dt.y2 = dy;

		  // Check for other taps covering this edge
		  // (to do)

		  // Find all grid points affected
	          gridx = (int)((ds->x1 - Xlowerbound - PitchX[ds->layer]) /
				PitchX[ds->layer]);
	          dx = (gridx * PitchX[ds->layer]) + Xlowerbound;
		  while (dx < ds->x1 - s) {
		     dx += PitchX[ds->layer];
		     gridx++;
		  }
		  while (dx < ds->x2 + s) {
		     u = ((Obs[ds->layer][OGRID(gridx, gridy, ds->layer)] &
				PINOBSTRUCTMASK) == STUBROUTE_NS) ? v : w;
		     if (dx + EPS < ds->x2 - u)
			block_route(gridx, gridy, ds->layer, EAST);
		     if (dx - EPS > ds->x1 + u)
			block_route(gridx, gridy, ds->layer, WEST);
		     dx += PitchX[ds->layer];
		     gridx++;
		  }
	       }
	    }
	 }
      }
   }
}

/* node.c */
