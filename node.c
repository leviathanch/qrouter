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

POINT get_left_lower_trunk_point(BBOX bbox)
{
	if(!bbox) return NULL;
	if(!bbox->edges) return NULL;
	POINT pt1, pt2;
	pt1 = bbox->edges->pt1;
	pt2 = bbox->edges->pt2;
	if(!pt1) return NULL;
	if(!pt2) return NULL;
	POINT retpt = create_point(pt1->x,pt1->y, 0); // creating requested point;
	for(BBOX_LINE l = bbox->edges; l; l=l->next) {
		if(!l) continue;
		pt1 = l->pt1;
		pt2 = l->pt2;
		if(!pt1) continue;
		if(!pt2) continue;
		if(pt1->x<retpt->x) retpt->x = pt1->x;
		if(pt1->y<retpt->y) retpt->y = pt1->y;
		if(pt2->x<retpt->x) retpt->x = pt2->x;
		if(pt2->y<retpt->y) retpt->y = pt2->y;
	}
	return retpt;
}

POINT get_right_upper_trunk_point(BBOX bbox)
{
	if(!bbox) return NULL;
	if(!bbox->edges) return NULL;
	POINT pt1, pt2;
	pt1 = bbox->edges->pt1;
	pt2 = bbox->edges->pt2;
	if(!pt1) return NULL;
	if(!pt2) return NULL;

	POINT retpt =  create_point(pt1->x,pt1->y,0); // creating requested point;
	for(BBOX_LINE l = bbox->edges; l; l=l->next) {
		if(!l) continue;
		pt1 = l->pt1;
		pt2 = l->pt2;
		if(!pt1) continue;
		if(!pt2) continue;
		if(pt1->x>retpt->x) retpt->x = pt1->x;
		if(pt1->y>retpt->y) retpt->y = pt1->y;
		if(pt2->x>retpt->x) retpt->x = pt2->x;
		if(pt2->y>retpt->y) retpt->y = pt2->y;
	}
	return retpt;
}

int get_bbox_area(NET net)
{
	int ret = 0;
	int xmin;
	POINT p = get_left_lower_trunk_point(net->bbox);
	BBOX_LINE hlines1 = get_horizontal_lines(net->bbox->edges);
	BBOX_LINE hlines2 = get_horizontal_lines(net->bbox->edges);
	xmin = hlines1->pt1->x;
	for(BBOX_LINE l1=hlines1;l1;l1=l1->next) if(l1->pt1->x<xmin) xmin = l1->pt1->x;
	for(BBOX_LINE l1=hlines1;l1;l1=l1->next) {
		for(BBOX_LINE l2=hlines2;l2;l2=l2->next) {
		}
	}
	return ret;
} // TODO: fix this!

int net_absolute_distance(NET net)
{
	POINT pt1, pt2;
	int distance, x, y;
	pt1 = get_right_upper_trunk_point(net->bbox);
	pt2 = get_left_lower_trunk_point(net->bbox);
	x = pt1->x - pt2->x;
	y = pt1->y - pt2->y;
	distance = sqrt((x*x)+(y*y));
	return distance;
}

POINT clone_point(POINT p)
{
	if(!p) return NULL;
	POINT pt = create_point(p->x,p->y,p->layer);
	pt->next = NULL;
	return pt;
}

POINT create_point(int x, int y, int layer)
{
	POINT pt = malloc(sizeof(struct point_)); // creating requested point
	if(!pt) {
		printf("%s: memory leak. dying!\n",__FUNCTION__);
		exit(0);
	}
	pt->x=x;
	pt->y=y;
	pt->layer=layer;
	pt->next = NULL;
	return pt;
}

BBOX create_fresh_bbox()
{
	BBOX pt = malloc(sizeof(struct bbox_)); // creating requested box
	if(!pt) {
		printf("%s: memory leak. dying!\n",__FUNCTION__);
		exit(0);
	}
	pt->edges = NULL;
	pt->num_edges = 0;
	pt->x1_exception = FALSE;
	pt->x2_exception = FALSE;
	pt->y1_exception = FALSE;
	pt->y2_exception = FALSE;
	return pt;
}

BBOX_LINE clone_line(BBOX_LINE orig)
{
	if(!orig) return NULL;
	if(!orig->pt1) return NULL;
	if(!orig->pt2) return NULL;
	BBOX_LINE r = get_fresh_line();
	r->pt1=clone_point(orig->pt1);
	r->pt2=clone_point(orig->pt2);
	r->next = NULL;
	return r;
}

BBOX_LINE get_fresh_line()
{
	BBOX_LINE r = malloc(sizeof(struct bbox_line_));
	if(!r) {
		printf("%s: memory leak. dying!\n",__FUNCTION__);
		exit(0);
	}
	r->next = NULL;
	r->pt1 = NULL;
	r->pt2 = NULL;
}

BBOX_LINE clone_line_list(BBOX_LINE orig)
{
	if(!orig) return NULL;
	BBOX_LINE ret;
	ret = clone_line(orig);
	ret->next = clone_line_list(orig->next);
	return ret;
}

BBOX shrink_bbox(BBOX orig, int num_pixels)
{
	int x, y;
	POINT pt;
	BBOX ret = clone_bbox(orig);
	pt = get_left_lower_trunk_point(orig);
	int xmin0 = pt->x;
	int ymin0 = pt->y;
	free(pt);

	for(BBOX_LINE l=ret->edges;l;l=l->next) {
		x=(l->pt1->x>xmin0)?l->pt1->x-xmin0:0;  // setting p0 as 0 point
		x=(x>(num_pixels*2))?(x-(num_pixels*2)):0; // if not zero point, shorten
		l->pt1->x=x;

		y=(l->pt1->y>ymin0)?l->pt1->y-ymin0:0;  // setting p0 as 0 point
		y=(y>(num_pixels*2))?(y-(num_pixels*2)):0; // if not zero point, shorten
		l->pt1->y=y;

		x=(l->pt2->x>xmin0)?l->pt2->x-xmin0:0;  // setting p0 as 0 point
		x=(x>(num_pixels*2))?(x-(num_pixels*2)):0; // if not zero point, shorten
		l->pt2->x=x;

		y=(l->pt2->y>ymin0)?l->pt2->y-ymin0:0;  // setting p0 as 0 point
		y=(y>(num_pixels*2))?(y-(num_pixels*2)):0; // if not zero point, shorten
		l->pt2->y=y;

		if(points_equal(l->pt1,l->pt2)) {
			free(ret);
			return NULL;
		} else {
			// shift right
			l->pt1->x+=num_pixels+xmin0;
			l->pt2->x+=num_pixels+xmin0;
			// shift up
			l->pt1->y+=num_pixels+ymin0;
			l->pt2->y+=num_pixels+ymin0;
		}
	}

	return ret;
}

BBOX clone_bbox(BBOX orig)
{
	if(!orig) return NULL;
	BBOX r = NULL;
	r = create_fresh_bbox();
	r->edges =  clone_line_list(orig->edges);
	r->num_edges = orig->num_edges;
	return r;
}

BOOL points_equal(POINT p1, POINT p2)
{
	if(!p1) return FALSE;
	if(!p2) return FALSE;
	if((p1->x==p2->x)&&(p1->y==p2->y)) return TRUE;
	return FALSE;
}

BOOL points_fully_equal(POINT p1, POINT p2)
{
	if(!p1) return FALSE;
	if(!p2) return FALSE;
	if((p1->x==p2->x)&&(p1->y==p2->y)&&(p1->layer==p2->layer)) return TRUE;
	return FALSE;
}

BOOL point_in_list(POINT list, POINT needle)
{
	BOOL ret = FALSE;
	for(POINT p=list;p;p=p->next) {
		if(points_fully_equal(p,needle)) ret=TRUE;
	}
	return ret;
}

int count_points_in_list(POINT list)
{
	int ret=0;
	for(POINT p=list;p;p=p->next) ret++;
	return ret;
}

BOOL gpoints_equal(GRIDP p1, GRIDP p2)
{
	if((p1.x==p2.x)&&(p1.y==p2.y)) return TRUE;
	return FALSE;
}

BOOL lines_equal(BBOX_LINE l1, BBOX_LINE l2)
{
	if(!l1) return FALSE;
	if(!l2) return FALSE;
	if(points_equal(l1->pt1,l2->pt1)&&points_equal(l1->pt2,l2->pt2)) return TRUE;
	if(points_equal(l1->pt1,l2->pt2)&&points_equal(l1->pt2,l2->pt1)) return TRUE;
	return FALSE;
}

BOOL edge_contains_line(BBOX_LINE list, BBOX_LINE line)
{
	if(!list) return FALSE;
	if(!line) return FALSE;
	for(BBOX_LINE l=list;l;l=l->next) if(lines_equal(line,l)) return TRUE;
	return FALSE;
}

BBOX_LINE add_line_to_edge(BBOX_LINE list, BBOX_LINE l)
{
	if(!l) return list;
	if(edge_contains_line(list,l)) {
		return list;
	}
	BBOX_LINE line = clone_line(l); // cloning line
	line->next = list;
	return line;
}

BBOX add_line_to_bbox(BBOX bbox, BBOX_LINE ol)
{
	if(!ol) return bbox;
	BBOX b = (bbox)?bbox:create_fresh_bbox();
	BBOX_LINE line = clone_line(ol);
	b->num_edges++; // incrementing line count
	b->edges=add_line_to_edge(b->edges, line);
	return b;
}

BBOX add_line_to_bbox_ints(BBOX bbox, int x1, int y1, int x2, int y2)
{
	BBOX_LINE line = NULL; // line to add
	BBOX b = (bbox==NULL)?create_fresh_bbox():bbox;
	line = get_fresh_line(); // creating requested line
	line->pt1 = create_point(x1,y1,0);
	line->pt2 = create_point(x2,y2,0);
	b->num_edges++; // incrementing line count
	b->edges=add_line_to_edge(b->edges, line);
	return b;
}

#define CHECK_POINT_ABOVE_HLINE 1
#define CHECK_POINT_UNDER_HLINE 2
#define CHECK_POINT_LEFT_VLINE 3
#define CHECK_POINT_RIGHT_VLINE 4

BOOL check_point_to_line(int mode, BBOX_LINE line, POINT pnt, BOOL with_edge, int edge_distance)
{
	if(!line) return FALSE;
	if(!pnt) return FALSE;
	if(!line->pt1) return FALSE;
	if(!line->pt2) return FALSE;
	int xmin, xmax;
	int ymin, ymax;
	int x,y;

	if((mode==CHECK_POINT_ABOVE_HLINE)||(mode==CHECK_POINT_UNDER_HLINE)) {
		if(line->pt1->y!=line->pt2->y) return FALSE; // not a horizontal line!
	}
	if((mode==CHECK_POINT_LEFT_VLINE)||(mode==CHECK_POINT_RIGHT_VLINE)) {
		if(line->pt1->x!=line->pt2->x) return FALSE; // not a vertical line!
	}

	xmin=(line->pt1->x<line->pt2->x)?line->pt1->x:line->pt2->x;
	xmax=(line->pt1->x>line->pt2->x)?line->pt1->x:line->pt2->x;
	ymin=(line->pt1->y<line->pt2->y)?line->pt1->y:line->pt2->y;
	ymax=(line->pt1->y>line->pt2->y)?line->pt1->y:line->pt2->y;

	switch(mode) {
		case CHECK_POINT_ABOVE_HLINE:
			y=(with_edge)?ymin:ymin+edge_distance;
			if(with_edge) { if(pnt->y<y) return FALSE; } // point is under hline y
			else if(pnt->y<=y) return FALSE; // point is under hline y or equal
			break;
		case CHECK_POINT_UNDER_HLINE:
			y=(with_edge)?ymin:ymin-edge_distance;
			if(with_edge) { if(pnt->y>y) return FALSE; } // point is over hline y
			else if(pnt->y>=y) return FALSE; // point is over hline y or equal
			break;
		case CHECK_POINT_LEFT_VLINE:
			x=(with_edge)?xmin:xmin-edge_distance;
			if(with_edge) { if(pnt->x>x) return FALSE; } // point is right of line
			else if(pnt->x>=x) return FALSE; // point is left of line or equal
			break;
		case CHECK_POINT_RIGHT_VLINE:
			x=(with_edge)?xmin:xmin+edge_distance;
			if(with_edge) { if(pnt->x<x) return FALSE; } // point is left of line
			else if(pnt->x<=x) return FALSE; // point is left of line or equal
			break;
	}

	if((mode==CHECK_POINT_ABOVE_HLINE)||(mode==CHECK_POINT_UNDER_HLINE))
		if((pnt->x>=xmin)&&(pnt->x<=xmax)) return TRUE;
	if((mode==CHECK_POINT_LEFT_VLINE)||(mode==CHECK_POINT_RIGHT_VLINE))
		if((pnt->y>=ymin)&&(pnt->y<=ymax)) return TRUE;

	return FALSE;
}

// check whether a line goes through another area
BOOL check_line_area(BBOX bbox, BBOX_LINE line, BOOL with_edge)
{
	BOOL ret = FALSE;
	int xmin, xmax, ymin, ymax;
	POINT vpnt = create_point(0,0,0);
	if(line->pt1->x==line->pt2->x) {
		ymin=(line->pt1->y<line->pt2->y)?line->pt1->y:line->pt2->y;
		ymax=(line->pt1->y>line->pt2->y)?line->pt1->y:line->pt2->y;
		vpnt->x=line->pt1->x;
		for(vpnt->y=ymin;vpnt->y<ymax;vpnt->y++) if(check_point_area(bbox,vpnt,with_edge,0)) ret=TRUE;
	}
	if(line->pt1->y==line->pt2->y) {
		xmin=(line->pt1->x<line->pt2->x)?line->pt1->x:line->pt2->x;
		xmax=(line->pt1->x>line->pt2->x)?line->pt1->x:line->pt2->x;
		vpnt->y=line->pt1->y;
		for(vpnt->x=xmin;vpnt->x<xmax;vpnt->x++) if(check_point_area(bbox,vpnt,with_edge,0)) ret=TRUE;
	}
	free(vpnt);
	return ret;
}

BOOL check_grid_point_area(BBOX bbox, GRIDP gpnt, BOOL with_edge, int edge_distance)
{
	if(!bbox) return FALSE;
	if(bbox->num_edges<4) return FALSE;
	BOOL ret;
	POINT pnt = create_point(gpnt.x,gpnt.y,gpnt.lay);
	ret=check_point_area(bbox, pnt, with_edge, edge_distance);
	free(pnt);
	return ret;
}

int count_line_list(BBOX_LINE list)
{
	int ret=0;
	if(!list) return ret;
	for(BBOX_LINE l=list;l;l=l->next) if(l) ret++;
	return ret;
}

BOOL is_closed_shape(BBOX box)
{
	if(!box) return FALSE;
	if(!box->edges) return FALSE;
	if(box->num_edges<4) return FALSE;
	BOOL ret = FALSE;
	BBOX_LINE obj = box->edges;
	BBOX_LINE edge = NULL;
	POINT pt = clone_point(obj->pt1);
	for(BBOX_LINE l2=obj;l2;l2=l2->next) {
		for(BBOX_LINE l=obj;l;l=l->next) {
			if(edge_contains_line(edge,l)) continue;
			if(points_equal(pt,l->pt1)) {
				free(pt);
				pt=clone_point(l->pt2);
				edge=add_line_to_edge(edge,l);
				break;
			}
			if(points_equal(pt,l->pt2)) {
				free(pt);
				pt=clone_point(l->pt1);
				edge=add_line_to_edge(edge,l);
				break;
			}
		}
	}
	free(pt);
	if(count_line_list(edge)==count_line_list(obj)) ret = TRUE;
	free_line_list(edge);
	return ret;
}

// check whether pnt of point is within borders
BOOL check_point_area(BBOX bbox, POINT pnt, BOOL with_edge, int edge_distance)
{
	if(!bbox) return FALSE;
	if(!bbox->edges) return FALSE;
	if(bbox->num_edges<4) return FALSE;
	if(!pnt) return FALSE;
	int xmin, xmax, ymin, ymax;

	// first check die area
	if((pnt->x<0)||(pnt->x>NumChannelsX[0])||(pnt->y<0)||(pnt->y>NumChannelsY[0])) return FALSE; // outside die

	// is not a closed shape then false
	if(!is_closed_shape(bbox)) return FALSE;

	// then  check trunk box
	POINT pt1 = get_left_lower_trunk_point(bbox);
	POINT pt2 = get_right_upper_trunk_point(bbox);
	if(!pt1) return FALSE;
	if(!pt2) return FALSE;
	xmin=pt1->x;
	xmax=pt2->x;
	ymin=pt1->y;
	ymax=pt2->y;
	free(pt1);
	free(pt2);
	if((pnt->x<xmin)||(pnt->x>xmax)||(pnt->y<ymin)||(pnt->y>ymax)) return FALSE; // outside trunk

	// now check structure
	BBOX_LINE hlines = get_horizontal_lines(bbox->edges);
	BBOX_LINE vlines = get_vertical_lines(bbox->edges);
	for(BBOX_LINE hll = hlines;hll;hll=hll->next) { // horizontal lower line hll
		//if(bbox->y1_exception) printf("%s lower die area violated, adapting\n",__FUNCTION__);
		if(check_point_to_line(CHECK_POINT_ABOVE_HLINE,hll,pnt,with_edge,bbox->y1_exception?0:edge_distance)) {
			for(BBOX_LINE vrl = vlines;vrl;vrl=vrl->next) { // vertical right line vrl
				//if(bbox->x2_exception) printf("%s right die area violated, adapting\n",__FUNCTION__);
				if(check_point_to_line(CHECK_POINT_LEFT_VLINE,vrl,pnt,with_edge,bbox->x2_exception?0:edge_distance)) {
					for(BBOX_LINE hul = hlines;hul;hul=hul->next) { // horizontal upper line hul
						//if(bbox->y2_exception) printf("%s upper die area violated, adapting\n",__FUNCTION__);
						if(check_point_to_line(CHECK_POINT_UNDER_HLINE,hul,pnt,with_edge,bbox->y2_exception?0:edge_distance)) {
							for(BBOX_LINE vll = vlines;vll;vll=vll->next) { // vertical left line vll
								//if(bbox->x1_exception) printf("%s left die area violated, adapting\n",__FUNCTION__);
								if(check_point_to_line(CHECK_POINT_RIGHT_VLINE,vll,pnt,with_edge,bbox->x1_exception?0:edge_distance)) {
									free_line_list(hlines);
									free_line_list(vlines);
									return TRUE;
								}
							}
						}
					}
				}
			}
		}
	}
	free_line_list(hlines);
	free_line_list(vlines);
	return FALSE;
}

// check whether b2 is totally within b1
BOOL partial_bbox_overlap(BBOX b1, BBOX b2)
{
	if(!b1) return FALSE;
	if(!b2) return FALSE;
	BOOL ret=FALSE;
	for(BBOX_LINE l=b2->edges;l;l=l->next) if(check_line_area(b1,l,TRUE)) ret=TRUE;
	return ret;
}

// check whether b2 is totally within b1
BOOL box2_inside_box1(BBOX b1, BBOX b2)
{
	if(!b1) return FALSE;
	if(!b2) return FALSE;
	POINT p1, p2;
	BOOL ret=TRUE;
	for(BBOX_LINE l=b2->edges;l;l=l->next) {
		p1=l->pt1;
		p2=l->pt2;
		if(!check_point_area(b1,p1,TRUE,0)) ret=FALSE;
		if(!check_point_area(b1,p2,TRUE,0)) ret=FALSE;
	}
	return ret;
}

BOOL check_single_bbox_collision(BBOX box1, BBOX box2)
{
	if(!box1) return TRUE;
	if(!box2) return TRUE;
	if(box1==box2) return TRUE;
	BOOL ret=FALSE;
	if(box2_inside_box1(box1,box2)) ret=TRUE;
	if(box2_inside_box1(box2,box1)) ret=TRUE;
	if(partial_bbox_overlap(box1,box2)) ret=TRUE;
	if(partial_bbox_overlap(box2,box1)) ret=TRUE;
	return ret;
}

NETLIST get_bbox_collisions(NET net, BOOL thread)
{
	NETLIST ret = NULL;
	NET n;
	if(!net) return NULL;
	if(!net->bbox) return NULL;
	if(net->bbox->num_edges<4) return NULL;
	if(thread==FOR_THREAD) {
		for(int i=0; i<MAX_NUM_THREADS; i++) {
			n = CurNet[i];
			if(n) {
				if(n!=net) {
					if(check_single_bbox_collision(net->bbox,n->bbox)) {
						ret=postpone_net(ret,n);
					}
				}
			}
		}
	}
	if(thread==NOT_FOR_THREAD) {
		for(int i=0; i<Numnets; i++) {
			n = getnettoroute(i);
			if(n) {
				if((n!=net)&&!is_gndnet(n)&&!is_vddnet(n)&&!is_clknet(n)) {
					if(check_single_bbox_collision(net->bbox,n->bbox)) {
						ret=postpone_net(ret,n);
					}
				}
			}
		}
	}
	return ret;
}

BOOL check_bbox_collisions(NET net, BOOL thread)
{
	NET n;
	BOOL ret = FALSE;
	if(!net) return TRUE;
	if(thread==FOR_THREAD) {
		for(int i=0; i<MAX_NUM_THREADS; i++) {
			n = CurNet[i];
			if(n) {
				if(n!=net) {
					if(check_single_bbox_collision(net->bbox,n->bbox)) {
						ret=TRUE;
					}
				}
			}
		}
	}
	if(thread==NOT_FOR_THREAD) {
		for(int i=0; i<Numnets; i++) {
			n = getnettoroute(i);
			if(n) {
				if((n!=net)&&!is_gndnet(n)&&!is_vddnet(n)&&!is_clknet(n)&&!n->routed) {
					if(check_single_bbox_collision(net->bbox,n->bbox)) {
						ret=TRUE;
					}
				}
			}
		}
	}
	return ret;
}

void free_bbox(BBOX t)
{
	if(!t) return;
	free_line_list(t->edges);
	free(t);
}

void free_line(BBOX_LINE t)
{
	if(!t) return;
	if(!t->pt1) return;
	if(!t->pt2) return;
	free(t->pt1);
	free(t->pt2);
	free(t);
}

BBOX_LINE delete_line_from_edge(BBOX_LINE vbox, BBOX_LINE l)
{
	if(!vbox) return NULL;
	if(!l) return vbox;
	if(edge_contains_line(vbox,l)) {
		BBOX_LINE i = vbox;
		BBOX_LINE last = NULL;
		while(!lines_equal(i,l)) {
			last=i;
			i=i->next;
		}
		if(last) last->next=i->next;
		else vbox=i->next;
		free(i);
	}
	return vbox;
}

// checks whether all taps are inside vbox
// return FALSE if not and otherwise TRUE
BOOL check_bbox_consistency(NET net, BBOX vbox)
{
	if(!net) return FALSE;
	if(!vbox) return FALSE;
	if(!vbox->edges) return FALSE;
	if(!is_closed_shape(vbox)) return FALSE;

	POINT vpnt;
	DPOINT dtap;
	BOOL ok;

	vpnt = create_point(0,0,0);
	for(NODE tn = net->netnodes; tn; tn=tn->next) {
		dtap = (tn->taps == NULL) ? tn->extend : tn->taps;
		if (dtap == NULL) continue;
		vpnt->x=dtap->gridx;
		vpnt->y=dtap->gridy;
		ok=check_point_area(vbox, vpnt,FALSE,TAP_ROOM);
		if(!ok) {
			free(vpnt);
			return FALSE;
		}
	}
	free(vpnt);

	return TRUE;
}

void free_line_list(BBOX_LINE t)
{
	if(!t) return;
	BBOX_LINE curr, head=t;
	while ((curr = head) != NULL) {
		head = head->next;
		free_line(curr);
	}
}

BBOX_LINE get_vertical_lines(BBOX_LINE box)
{
	if(!box) return NULL;
	BBOX_LINE ret = NULL;
	for(BBOX_LINE l=box;l;l=l->next)
		if((l->pt1->x==l->pt2->x)&&(l->pt1->y!=l->pt2->y))
			ret=add_line_to_edge(ret,l);
	return ret;
}

BBOX_LINE get_horizontal_lines(BBOX_LINE box)
{
	if(!box) return NULL;
	BBOX_LINE ret = NULL;
	for(BBOX_LINE l=box;l;l=l->next)
		if((l->pt1->x!=l->pt2->x)&&(l->pt1->y==l->pt2->y))
			ret=add_line_to_edge(ret,l);
	return ret;
}

POINT get_line_intersection(BBOX_LINE a, BBOX_LINE b)
{
	if(!a) return NULL;
	if(!a->pt1) return NULL;
	if(!a->pt2) return NULL;
	if(!b) return NULL;
	if(!b->pt1) return NULL;
	if(!b->pt2) return NULL;
	int axmin, axmax;
	int aymin, aymax;
	int bxmin, bxmax;
	int bymin, bymax;
	int ax, ay;
	int bx, by;
	int ax1=a->pt1->x, ax2=a->pt2->x, ay1=a->pt1->y, ay2=a->pt2->y;
	int bx1=b->pt1->x, bx2=b->pt2->x, by1=b->pt1->y, by2=b->pt2->y;
	if((ax1==ax2)&&(bx1==bx2)) { // parallel
		return NULL;
	} else if((ay1==ay2)&&(by1==by2)) { // parallel
		return NULL;
	} else if((ax1==ax2)&&(by1==by2)) { // line a is vertical, line b is horizontal
		aymin = (ay1<ay2)?ay1:ay2;
		aymax = (ay1>ay2)?ay1:ay2;
		bxmin = (bx1<bx2)?bx1:bx2;
		bxmax = (bx1>bx2)?bx1:bx2;
		ax=ax1;
		by=by1;
		if((aymin<=by)&&(aymax>=by)) // line a is vertical here
			if((bxmin<=ax)&&(bxmax>=ax))  // line b must be horizontal here
				return create_point(ax,by,0);
	} else if((ay1==ay2)&&(bx1==bx2)) { // line a is horizontal, line b is vertical
		axmin = (ax1<ax2)?ax1:ax2;
		axmax = (ax1>ax2)?ax1:ax2;
		bymin = (by1<by2)?by1:by2;
		bymax = (by1>by2)?by1:by2;
		ay=ay1;
		bx=bx1;
		if((axmin<=bx)&&(axmax>=bx)) // line a is horizontal here
			if((bymin<=ay)&&(bymax>=ay))  // line b must be vertical here
				return create_point(bx,ay,0);
	}
	return NULL;
}

BOOL lines_are_intersecting(BBOX_LINE a, BBOX_LINE b)
{
	if(!a) return FALSE;
	if(!b) return FALSE;
	BOOL ret = FALSE;
	POINT p = get_line_intersection(a,b);
	if(p) {
		ret=TRUE;
		free(p);
	}
	return ret;
}

BOOL point_on_hline(BBOX_LINE l, POINT pnt)
{
	if(!l) return FALSE;
	if(!pnt) return FALSE;
	BOOL ret = FALSE;
	int xmin, xmax, y;
	xmin=(l->pt1->x<l->pt2->x)?l->pt1->x:l->pt2->x;
	xmax=(l->pt1->x>l->pt2->x)?l->pt1->x:l->pt2->x;
	y=l->pt2->y;
	if((pnt->y==y)&&(pnt->x>=xmin)&&(pnt->x<=xmax)) ret = TRUE;
	return ret;
}

BOOL point_on_vline(BBOX_LINE l, POINT pnt)
{
	if(!l) return FALSE;
	if(!pnt) return FALSE;
	BOOL ret = FALSE;
	int ymin, ymax, x;
	ymin=(l->pt1->y<l->pt2->y)?l->pt1->y:l->pt2->y;
	ymax=(l->pt1->y>l->pt2->y)?l->pt1->y:l->pt2->y;
	x=l->pt2->x;
	if((pnt->x==x)&&(pnt->y>=ymin)&&(pnt->y<=ymax)) ret = TRUE;
	return ret;
}

BOOL point_on_edge(BBOX box, POINT pnt)
{
	BOOL ret = FALSE;
	BBOX_LINE hlines = get_horizontal_lines(box->edges);
	BBOX_LINE vlines = get_vertical_lines(box->edges);
	for(BBOX_LINE l=hlines;l;l=l->next) if(point_on_hline(l, pnt)) ret = TRUE;
	for(BBOX_LINE l=vlines;l;l=l->next) if(point_on_vline(l, pnt)) ret = TRUE;
	free_line_list(hlines);
	free_line_list(vlines);
	return ret;
}

BBOX_LINE get_intersecting_lines(BBOX box1, BBOX box2)
{
	if(!box1) return NULL;
	if(!box2) return NULL;
	BBOX_LINE ret = NULL;
	for(BBOX_LINE la=box1->edges; la; la=la->next)
		for(BBOX_LINE lb=box2->edges; lb; lb=lb->next)
			if(lines_are_intersecting(la,lb))
				ret = add_line_to_edge(ret, la);
	return ret;
}

BBOX_LINE connect_edge_gaps(BBOX box)
{
	if(!box) return NULL;
	BBOX_LINE ret = NULL;
	BBOX_LINE line = NULL;
	BBOX_LINE edge = clone_line_list(box->edges);
	if(!edge) return NULL;
	POINT pt = clone_point(edge->pt1);

	for(BBOX_LINE l1=box->edges;l1;l1=l1->next) {
		for(BBOX_LINE l2=edge;l2;l2=l2->next) {
			if(points_equal(pt,l2->pt1)) {
				free(pt);
				pt=clone_point(l2->pt2);
				edge=delete_line_from_edge(edge,l2);
				break;
			}
			if(points_equal(pt,l2->pt2)) {
				free(pt);
				pt=clone_point(l2->pt1);
				edge=delete_line_from_edge(edge,l2);
				break;
			}
		}
	}

	if(count_line_list(edge)) {
		for(BBOX_LINE la=edge;la;la=la->next) {
			for(BBOX_LINE l=edge;l;l=l->next) {
				if(((pt->x==l->pt1->x)&&(pt->y!=l->pt1->y))||((pt->x!=l->pt1->x)&&(pt->y==l->pt1->y))) {
					line=get_fresh_line();
					line->pt1=clone_point(pt);
					line->pt2=clone_point(l->pt1);
					ret=add_line_to_edge(ret,line);
					free_line(line);
					free(pt);
					pt=clone_point(l->pt2);
				}
				if(((pt->x==l->pt2->x)&&(pt->y!=l->pt2->y))||((pt->x!=l->pt2->x)&&(pt->y==l->pt2->y))) {
					line=get_fresh_line();
					line->pt1=clone_point(pt);
					line->pt2=clone_point(l->pt2);
					ret=add_line_to_edge(ret,line);
					free_line(line);
					free(pt);
					pt=clone_point(l->pt2);
				}
			}
		}
	}

	free(pt);
	free_line_list(edge);

	return ret;
}

BOOL lines_are_parallel(BBOX_LINE a, BBOX_LINE b)
{
	if(!a) return FALSE;
	if(!b) return FALSE;
	if(!a->pt1) return FALSE;
	if(!a->pt2) return FALSE;
	if(!b->pt1) return FALSE;
	if(!b->pt2) return FALSE;
	if((a->pt1->x==a->pt2->x)&&(b->pt1->x==b->pt2->x)) return TRUE;
	if((a->pt1->y==a->pt2->y)&&(b->pt1->y==b->pt2->y)) return TRUE;
	return FALSE;
}

BBOX_LINE get_cutout_edge(BBOX box1, BBOX box2)
{
	if(!box1) return NULL;
	if(!box2) return NULL;
	BBOX_LINE ret = NULL; // return value
	BBOX_LINE tl = NULL; // temporary line
	BBOX_LINE vtl = NULL; // vertical temporary line
	POINT i = NULL; // intersect point
	
	for(BBOX_LINE line=box2->edges; line; line=line->next) {
		if(point_on_edge(box1,line->pt1)&&point_on_edge(box1,line->pt2)) { // whole line on the edge
			continue;
		} else if(point_on_edge(box1,line->pt1)) { // point on the edge
			continue;
		} else if(point_on_edge(box1,line->pt2)) { // point on the edge
			continue;
		} else if(check_point_area(box1,line->pt1,FALSE,0)&&check_point_area(box1,line->pt2,FALSE,0)) { // whole line within our box (without edges)
			tl = clone_line(line);
			ret = add_line_to_edge(ret,tl);
			free_line(tl);
		} else if(check_point_area(box1,line->pt1,FALSE,0)) {
			for(vtl=box1->edges;vtl;vtl=vtl->next) {
				if(lines_are_parallel(vtl,line)) continue;
				i=get_line_intersection(line,vtl);
				if(i) {
					tl = get_fresh_line();
					tl->pt1 = clone_point(line->pt1);
					tl->pt2 = clone_point(i);
					ret=add_line_to_edge(ret,tl);
					free_line(tl);
					tl = get_fresh_line();
					tl->pt1 = clone_point(check_point_area(box2,vtl->pt1,TRUE,0)?vtl->pt2:vtl->pt1);
					tl->pt2 = clone_point(i);
					ret=add_line_to_edge(ret,tl);
					free_line(tl);
					free(i);
				}
			}
		} else if(check_point_area(box1,line->pt2,FALSE,0)) {
			for(vtl=box1->edges;vtl;vtl=vtl->next) {
				if(lines_are_parallel(vtl,line)) continue;
				i=get_line_intersection(line,vtl);
				if(i) {
					tl = get_fresh_line();
					tl->pt1 = clone_point(line->pt2);
					tl->pt2 = clone_point(i);
					ret=add_line_to_edge(ret,tl);
					free_line(tl);
					tl = get_fresh_line();
					tl->pt1 = clone_point(check_point_area(box2,vtl->pt1,TRUE,0)?vtl->pt2:vtl->pt1);
					tl->pt2 = clone_point(i);
					ret=add_line_to_edge(ret,tl);
					free_line(tl);
					free(i);
				}
			}
		}
	}
	return ret;
}

void fit_all_bboxes(NETLIST list)
{
	NET net;
	for(NETLIST li=list;li;li=li->next) {
		net=li->net;
		if(net) if(net->bbox) if(net->bbox->edges) { 
			for(BBOX_LINE l=net->bbox->edges;l;l=l->next) {
				for(int lay=0;lay<Num_layers;lay++) {
					if(l->pt1) {
						if(l->pt1->x>NumChannelsX[lay]) {
							l->pt1->x=NumChannelsX[lay];
							net->bbox->x2_exception=TRUE;
						}
						if(l->pt1->y>NumChannelsY[lay]) {
							l->pt1->y=NumChannelsY[lay];
							net->bbox->y2_exception=TRUE;
						}
					}
					if(l->pt2) {
						if(l->pt2->x>NumChannelsX[lay]) {
							l->pt2->x=NumChannelsX[lay];
							net->bbox->x2_exception=TRUE;
						}
						if(l->pt2->y>NumChannelsY[lay]) {
							l->pt2->y=NumChannelsY[lay];
							net->bbox->y2_exception=TRUE;
						}
					}
				}
				if(l->pt1) {
					if(l->pt1->x<0) {
						l->pt1->x=0;
						net->bbox->x1_exception=TRUE;
					}
					if(l->pt1->y<0) {
						l->pt1->y=0;
						net->bbox->y1_exception=TRUE;
					}
				}
				if(l->pt2) {
					if(l->pt2->x<0) {
						l->pt2->x=0;
						net->bbox->x1_exception=TRUE;
					}
					if(l->pt2->y<0) {
						l->pt2->y=0;
						net->bbox->y1_exception=TRUE;
					}
				}
			}
		}
	}
}

BBOX delete_line_from_bbox(BBOX bbox, BBOX_LINE l)
{
	if(!l) return bbox;
	if(!bbox) return bbox;
	if(!bbox->edges) return bbox;
	if(edge_contains_line(bbox->edges,l)) {
		bbox->edges=delete_line_from_edge(bbox->edges,l);
		bbox->num_edges--;
	}
	return bbox;
}

BOOL fit_competing_net_bboxes(NET n1, NET n2)
{
	if(!n1) return FALSE;
	if(!n2) return FALSE;
	if(!n1->bbox) return FALSE;
	if(!n2->bbox) return FALSE;
	if(n1->bbox->num_edges<4) return FALSE;
	if(n2->bbox->num_edges<4) return FALSE;
	if(box2_inside_box1(n1->bbox,n2->bbox)) return FALSE; // don't cut out inside

	BBOX bbox_temp = clone_bbox(n1->bbox); // copy of bbox1
	BBOX_LINE edge1 = get_intersecting_lines(bbox_temp,n2->bbox);
	BBOX_LINE edge2 = get_cutout_edge(bbox_temp,n2->bbox);

	for(BBOX_LINE l=edge1;l;l=l->next) bbox_temp=delete_line_from_bbox(bbox_temp, l);
	for(BBOX_LINE l=edge2;l;l=l->next) bbox_temp=add_line_to_bbox(bbox_temp, l);

	free_line_list(edge1);
	free_line_list(edge2);

	if(check_bbox_consistency(n1, bbox_temp)) { // check whether all taps are still within the box
		free_bbox(n1->bbox);
		n1->bbox=bbox_temp;
		return TRUE;
	} else { // roll back/restore
		free_bbox(bbox_temp);
		return FALSE;
	}
}

BOOL resolve_bbox_collisions(NET net, BOOL thread)
{
	NET n;
	BOOL ret = TRUE;
	if(!net) return FALSE;
	NETLIST pp=get_bbox_collisions(net,thread);
	for(NETLIST p=pp;p;p=p->next) {
		n = p->net;
		if((net!=n)&&n) {
			if(is_gndnet(n)||is_vddnet(n)||is_clknet(n)) {
				continue;
			} else {
				Fprintf(stdout,"fiting net %s with net %s: ",net->netname,n->netname);
				if(fit_competing_net_bboxes(net, n)) {
					Fprintf(stdout,"fit successful\n");
				} else {
					Fprintf(stdout,"fitting failed\n");
					ret=FALSE;
				}
			}
		}
	}
	free_postponed(pp);
	return ret;
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
   POINT llpt_p = get_left_lower_trunk_point(p->bbox);
   POINT rupt_p = get_right_upper_trunk_point(p->bbox);
   POINT llpt_q = get_left_lower_trunk_point(q->bbox);
   POINT rupt_q = get_right_upper_trunk_point(q->bbox);

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

   pwidth = rupt_p->x - llpt_p->x;
   pheight = rupt_p->y - llpt_p->y;
   pdim = (pwidth > pheight) ? pheight : pwidth;

   qwidth = rupt_q->x - llpt_q->x;
   qheight =  rupt_q->y - llpt_p->y;
   qdim = (qwidth > qheight) ? qheight : qwidth;

   free(llpt_p);
   free(rupt_p);
   free(llpt_q);
   free(rupt_q);

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
  int i, j;
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
   if(!net) return;

      // Use the first tap point for each node to get a rough bounding box and
      // centroid of all taps

      int x1=0, x2=0, y1=0, y2=0;
      if (net->numnodes == 0) return;	 // e.g., vdd or gnd bus

      n1 = net->netnodes;
      dtap = (n1->taps == NULL) ? n1->extend : n1->taps;
      x1=dtap->gridx;
      y1=dtap->gridy;
      for (n1 = net->netnodes; n1 != NULL; n1 = n1->next) {
         dtap = (n1->taps == NULL) ? n1->extend : n1->taps;
	 if (dtap) {
            if (dtap->gridx < x1) x1 = dtap->gridx;
            if (dtap->gridy < y1) y1 = dtap->gridy;
	 }
      }
      n1 = net->netnodes;
      dtap = (n1->taps == NULL) ? n1->extend : n1->taps;
      x2=dtap->gridx;
      y2=dtap->gridy;
      for (n1 = net->netnodes; n1 != NULL; n1 = n1->next) {
         dtap = (n1->taps == NULL) ? n1->extend : n1->taps;
	 if (dtap) {
            if (dtap->gridx > x2) x2 = dtap->gridx;
            if (dtap->gridy > y2) y2 = dtap->gridy;
	 }
      }
      
      for (n1 = net->netnodes; n1 != NULL; n1 = n1->next) {
         dtap = (n1->taps == NULL) ? n1->extend : n1->taps;
	 if (dtap) {
            if (dtap->gridx > x2) x2 = dtap->gridx;
            if (dtap->gridx < x1) x1 = dtap->gridx;
            if (dtap->gridy > y2) y2 = dtap->gridy;
            if (dtap->gridy < y1) y1 = dtap->gridy;
	 }
      }

      if(!net->bbox) net->bbox = create_fresh_bbox();

      net->bbox->x1_exception=FALSE;
      net->bbox->y1_exception=FALSE;
      net->bbox->x2_exception=FALSE;
      net->bbox->y2_exception=FALSE;

      x1-=BOX_ROOM_X;
      y1-=BOX_ROOM_Y;
      x2+=BOX_ROOM_X;
      y2+=BOX_ROOM_Y;

      net->bbox = add_line_to_bbox_ints(net->bbox, x1, y1, x1, y2); // left lower point -> left upper point
      net->bbox = add_line_to_bbox_ints(net->bbox, x1, y1, x2, y1); // left lower point -> right lower point
      net->bbox = add_line_to_bbox_ints(net->bbox, x2, y2, x2, y1); // right upper point -> right lower point
      net->bbox = add_line_to_bbox_ints(net->bbox, x2, y2, x1, y2); // right upper point -> left upper point
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

void define_route_tree(NET net)
{
    NODE n1;
    DPOINT dtap;
    int xcent, ycent, xmin, ymin, xmax, ymax;
    if(!net) return;
    if(!net->bbox) return;

    // This is called after create_bounding_box(), so bounds have
    // been calculated.
    POINT p1, p2;
    if(net->bbox->num_edges>3) {
    p1 = get_left_lower_trunk_point(net->bbox);
    p2 = get_right_upper_trunk_point(net->bbox);

    xmin = p1->x;
    xmax = p2->x;
    ymin = p1->y;
    ymax = p2->y;

    free(p1);
    free(p2);
    } else {
    xmin = -MAXRT;
    xmax = MAXRT;
    ymin = -MAXRT;
    ymax = MAXRT;
    }

    if (net->numnodes > 0) {

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
/* SetNodeinfo --						*/
/*	Allocate a NODEINFO record and put it in the Nodeinfo	*/
/*	array at position (gridx, gridy, d->layer).  Return the	*/
/* 	pointer to the location.				*/
/*--------------------------------------------------------------*/

NODEINFO
SetNodeinfo(int gridx, int gridy, int layer)
{
    NODEINFO *lnodeptr;

    lnodeptr = &NODEIPTR(gridx, gridy, layer);
    if (*lnodeptr == NULL) {
	*lnodeptr = (NODEINFO)calloc(1, sizeof(struct nodeinfo_));
    }
    return *lnodeptr;
}

/*--------------------------------------------------------------*/
/* FreeNodeinfo --						*/
/*	Free a NODEINFO record at array Nodeinfo position	*/
/*	(gridx, gridy, d->layer).  Set the position pointer to	*/
/*	NULL.  							*/
/*--------------------------------------------------------------*/

void
FreeNodeinfo(int gridx, int gridy, int layer)
{
    NODEINFO *lnodeptr;
    lnodeptr = &NODEIPTR(gridx, gridy, layer);

    if (*lnodeptr != NULL) {
        free(*lnodeptr);
	*lnodeptr = NULL;
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
  int i;
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
/* count_reachable_taps()					*/
/*								*/
/*  For each grid point in the layout, find if it corresponds	*/
/*  to a node and is unobstructed.  If so, increment the node's	*/
/*  count of reachable taps.  Then work through the list of	*/
/*  nodes and determine if any are completely unreachable.  If	*/
/*  so, then unobstruct any position that is inside tap		*/
/*  geometry that can contain a via.				*/
/*								*/
/*  NOTE:  This routine should check for tap rectangles that	*/
/*  may combine to form an area large enough to place a via;	*/
/*  also, it should check for tap points that are routable	*/
/*  by a wire and not a tap.  However, those conditions are	*/
/*  rare and are left unhandled for now.			*/
/*--------------------------------------------------------------*/

void
count_reachable_taps()
{
    NODE node;
    NODEINFO lnode;
    GATE g;
    DSEG ds;
    int l, i, j;
    int gridx, gridy;
    double deltax, deltay;
    double dx, dy;

    for (l = 0; l < Num_layers; l++) {
	for (j = 0; j < NumChannelsX[l] * NumChannelsY[l]; j++) {
	    if (Nodeinfo[l][j]) {
		node = Nodeinfo[l][j]->nodeloc;
		if (node != NULL) {

		    // Redundant check;  if Obs has NO_NET set, then
		    // Nodeinfo->nodeloc for that position should already
		    // be NULL

		    if (!(Obs[l][j] & NO_NET))
			node->numtaps++;
		}
	    }
	}
    }

    for (g = Nlgates; g; g = g->next) {
	for (i = 0; i < g->nodes; i++) {
	    node = g->noderec[i];
	    if (node == NULL) continue;
	    if (node->numnodes == 0) continue;	 // e.g., vdd or gnd bus
	    if (node->numtaps == 0) {
		Fprintf(stderr, "Error: Node %s of net \"%s\" has no taps!\n",
			print_node_name(node), node->netname);

		for (ds = g->taps[i]; ds; ds = ds->next) {
		    deltax = 0.5 * LefGetViaWidth(ds->layer, ds->layer, 0);
		    deltay = 0.5 * LefGetViaWidth(ds->layer, ds->layer, 1);

		    gridx = (int)((ds->x1 - Xlowerbound) / PitchX[ds->layer]) - 1;
		    while (1) {
			dx = (gridx * PitchX[ds->layer]) + Xlowerbound;
			if (dx > ds->x2 || gridx >= NumChannelsX[ds->layer]) break;

			if (((dx - ds->x1 + EPS) > deltax) &&
				((ds->x2 - dx + EPS) > deltax)) {
			    gridy = (int)((ds->y1 - Ylowerbound)
					/ PitchY[ds->layer]) - 1;
			    while (1) {
				dy = (gridy * PitchY[ds->layer]) + Ylowerbound;
				if (dy > ds->y2 || gridy >= NumChannelsY[ds->layer])
				    break;

				if (((dy - ds->y1 + EPS) > deltay) &&
					((ds->y2 - dy + EPS) > deltay)) {

				    if ((ds->layer == Num_layers - 1) ||
						!(OBSVAL(gridx, gridy, ds->layer + 1)
						& NO_NET)) {

					// Grid position is clear for placing a via

					Fprintf(stderr, "Tap position (%g, %g) appears"
						" to be technically routable, so it"
						" is being forced routable.\n",
						dx, dy);

					OBSVAL(gridx, gridy, ds->layer) =
						(OBSVAL(gridx, gridy, ds->layer)
						& BLOCKED_MASK)
						| (u_int)node->netnum;
					lnode = SetNodeinfo(gridx, gridy, ds->layer);
					lnode->nodeloc = node;
					lnode->nodesav = node;
					node->numtaps++;
				    }
				}
				gridy++;
			    }
			}
			gridx++;
		    }
		}
	    }
	    if (node->numtaps == 0) {
		/* Node wasn't cleanly within tap geometry when centered */
		/* on a grid point.  But if the via can be offset and is */
		/* cleanly within the tap geometry, then allow it.	 */

		double dist, mindist;
		int dir, mask, tapx, tapy, tapl;

		/* Initialize mindist to a large value */
		mask = 0;
		mindist = PitchX[Num_layers - 1] + PitchY[Num_layers - 1];
		dir = 0;	/* Indicates no solution found */

		for (ds = g->taps[i]; ds; ds = ds->next) {
		    deltax = 0.5 * LefGetViaWidth(ds->layer, ds->layer, 0);
		    deltay = 0.5 * LefGetViaWidth(ds->layer, ds->layer, 1);

		    gridx = (int)((ds->x1 - Xlowerbound) / PitchX[ds->layer]) - 1;
		    while (1) {
			dx = (gridx * PitchX[ds->layer]) + Xlowerbound;
			if (dx > ds->x2 || gridx >= NumChannelsX[ds->layer]) break;

			if (((dx - ds->x1 + EPS) > -deltax) &&
				((ds->x2 - dx + EPS) > -deltax)) {
			    gridy = (int)((ds->y1 - Ylowerbound)
					/ PitchY[ds->layer]) - 1;

			    while (1) {
				dy = (gridy * PitchY[ds->layer]) + Ylowerbound;
				if (dy > ds->y2 || gridy >= NumChannelsY[ds->layer])
				    break;

				// Check that the grid position is inside the
				// tap rectangle.  However, if the point above
				// the grid is blocked, then a via cannot be
				// placed here, so skip it.

				if (((ds->layer == Num_layers - 1) ||
					!(OBSVAL(gridx, gridy, ds->layer + 1)
					& NO_NET)) &&
					((dy - ds->y1 + EPS) > -deltay) &&
					((ds->y2 - dy + EPS) > -deltay)) {

				    // Grid point is inside tap geometry.
				    // Since it did not pass the simple insideness
				    // test previously, it can be assumed that
				    // one of the edges is closer to the grid point
				    // than 1/2 via width.  Find that edge and use
				    // it to determine the offset.

				    // Check right edge
				    if ((ds->x2 - dx + EPS) < deltax) {
					dist = deltax - ds->x2 + dx;
					// Confirm other edges
					if ((dx - dist - deltax + EPS > ds->x1) &&
					(dy - deltay + EPS > ds->y1) &&
					(dy + deltay - EPS < ds->y2)) {
					    if (dist < fabs(mindist)) {
						mindist = dist;
						mask = STUBROUTE;
						dir = NI_STUB_EW;
						tapx = gridx;
						tapy = gridy;
						tapl = ds->layer;
					    }
					}
				    }
				    // Check left edge
				    if ((dx - ds->x1 + EPS) < deltax) {
					dist = deltax - dx + ds->x1;
					// Confirm other edges
					if ((dx + dist + deltax - EPS < ds->x2) &&
					(dy - deltay + EPS > ds->y1) &&
					(dy + deltay - EPS < ds->y2)) {
					    if (dist < fabs(mindist)) {
						mindist = -dist;
						mask = STUBROUTE;
						dir = NI_STUB_EW;
						tapx = gridx;
						tapy = gridy;
						tapl = ds->layer;
					    }
					}
				    }
				    // Check top edge
				    if ((ds->y2 - dy + EPS) < deltay) {
					dist = deltay - ds->y2 + dy;
					// Confirm other edges
					if ((dx - deltax + EPS > ds->x1) &&
					(dx + deltax - EPS < ds->x2) &&
					(dy - dist - deltay + EPS > ds->y1)) {
					    if (dist < fabs(mindist)) {
						mindist = -dist;
						mask = STUBROUTE;
						dir = NI_STUB_NS;
						tapx = gridx;
						tapy = gridy;
						tapl = ds->layer;
					    }
					}
				    }
				    // Check bottom edge
				    if ((dy - ds->y1 + EPS) < deltay) {
					dist = deltay - dy + ds->y1;
					// Confirm other edges
					if ((dx - deltax + EPS > ds->x1) &&
					(dx + deltax - EPS < ds->x2) &&
					(dy + dist + deltay - EPS < ds->y2)) {
					    if (dist < fabs(mindist)) {
						mindist = dist;
						mask = STUBROUTE;
						dir = NI_STUB_NS;
						tapx = gridx;
						tapy = gridy;
						tapl = ds->layer;
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

		/* Was a solution found? */
		if (mask != 0) {
		    // Grid position is clear for placing a via

		    Fprintf(stderr, "Tap position (%d, %d) appears to be"
				" technically routable with an offset, so"
				" it is being forced routable.\n",
				tapx, tapy);

		    OBSVAL(tapx, tapy, tapl) =
				(OBSVAL(tapx, tapy, tapl) & BLOCKED_MASK)
				| mask | (u_int)node->netnum;
		    lnode = SetNodeinfo(tapx, tapy, tapl);
		    lnode->nodeloc = node;
		    lnode->nodesav = node;
		    lnode->stub = dist;
		    lnode->flags |= dir;
		    node->numtaps++;
		}
	    }
	    if (node->numtaps == 0) {
		Fprintf(stderr, "Qrouter will not be able to completely"
			" route this net.\n");
	    }
	}
    }
}

/*--------------------------------------------------------------*/
/* check_variable_pitch()					*/
/*								*/
/*  This routine is used by the routine below it to generate	*/
/*  obstructions that force routes to be placed in 1-of-N	*/
/*  tracks.  However, it is also used to determine the same	*/
/*  information for the .info file, so that the effective	*/
/*  pitch is output, not the pitch copied from the LEF file.	*/
/*  Output is the vertical and horizontal pitch multipliers,	*/
/*  passed back through pointers.				*/
/*--------------------------------------------------------------*/

void check_variable_pitch(int l, int *hptr, int *vptr)
{
   int o, hnum, vnum;
   double vpitch, hpitch, wvia;

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

   *vptr = vnum;
   *hptr = hnum;
}

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

void create_obstructions_from_variable_pitch(void)
{
   int l, vnum, hnum, x, y;
   NODEINFO lnode;

   for (l = 0; l < Num_layers; l++) {

      check_variable_pitch(l, &hnum, &vnum);

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

	       // If the grid position itself is a node, don't restrict
	       // routing based on variable pitch.
	       if (((lnode = NODEIPTR(x, y, l)) != NULL) && (lnode->nodeloc != NULL))
		  continue;

	       // If there is a node in an adjacent grid then allow
	       // routing from that direction.

	       if ((x > 0) && ((lnode = NODEIPTR(x - 1, y, l)) != NULL) &&
			(lnode->nodeloc != NULL))
		  OBSVAL(x, y, l) = BLOCKED_MASK & ~BLOCKED_W;
	       else if ((y > 0) && ((lnode = NODEIPTR(x , y - 1, l)) != NULL) &&
			(lnode->nodeloc != NULL))
		  OBSVAL(x, y, l) = BLOCKED_MASK & ~BLOCKED_S;
	       else if ((x < NumChannelsX[l] - 1)
			&& ((lnode = NODEIPTR(x + 1, y, l)) != NULL) &&
			(lnode->nodeloc != NULL))
		  OBSVAL(x, y, l) = BLOCKED_MASK & ~BLOCKED_E;
	       else if ((y < NumChannelsY[l] - 1)
			&& ((lnode = NODEIPTR(x, y + 1, l)) != NULL) &&
			(lnode->nodeloc != NULL))
		  OBSVAL(x, y, l) = BLOCKED_MASK & ~BLOCKED_N;
	       else
		  OBSVAL(x, y, l) = NO_NET;
	    }
	 }
      }
   }
}

/*--------------------------------------------------------------*/
/* disable_gridpos() ---					*/
/*	Render the position at (x, y, lay) unroutable by	*/
/*	setting its Obs[] entry to NO_NET and removing it from	*/
/*	the Nodeinfo->nodeloc and Nodeinfo->nodesav records.	*/
/*--------------------------------------------------------------*/

static void
disable_gridpos(int x, int y, int lay)
{
    int apos = OGRID(x, y, lay);

    Obs[lay][apos] = (u_int)(NO_NET | OBSTRUCT_MASK);
    if (Nodeinfo[lay][apos]) {
	free(Nodeinfo[lay][apos]);
	Nodeinfo[lay][apos] = NULL;
    }
}

/*--------------------------------------------------------------*/
/* count_pinlayers()---						*/
/*	Check which layers have non-NULL Nodeinfo entries.  	*/
/*	Then set "Pinlayers" and free all the unused layers.	*/
/* 	This saves a lot of memory, especially when the number	*/
/*	of routing layers becomes large.			*/ 
/*--------------------------------------------------------------*/

void
count_pinlayers()
{
   int j, l;

   Pinlayers = 0;
   for (l = 0; l < Num_layers; l++) {
      for (j = 0; j < NumChannelsX[l] * NumChannelsY[l]; j++) {
	 if (Nodeinfo[l][j]) {
	    Pinlayers = l + 1;
	    break;
	 }
      }
   }

   for (l = Pinlayers; l < Num_layers; l++) {
      free(Nodeinfo[l]);
      Nodeinfo[l] = NULL;
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

static void
check_obstruct(int gridx, int gridy, DSEG ds, double dx, double dy)
{
    u_int *obsptr;
    float dist;
    NODEINFO lnode;

    obsptr = &(OBSVAL(gridx, gridy, ds->layer));
    dist = OBSINFO(gridx, gridy, ds->layer);
    lnode = NODEIPTR(gridx, gridy, ds->layer);

    // Grid point is inside obstruction + halo.
    *obsptr |= NO_NET;

    // Completely inside obstruction?
    if (dy > ds->y1 && dy < ds->y2 && dx > ds->x1 && dx < ds->x2)
       *obsptr |= OBSTRUCT_MASK;

    else {

       // Make more detailed checks in each direction

       if (dy <= ds->y1) {
	  if ((*obsptr & (OBSTRUCT_MASK & ~OBSTRUCT_N)) == 0) {
	     if ((dist == 0) || ((ds->y1 - dy) < dist))
		OBSINFO(gridx, gridy, ds->layer) = ds->y1 - dy;
	     *obsptr |= OBSTRUCT_N;
	  }
	  else *obsptr |= OBSTRUCT_MASK;
       }
       else if (dy >= ds->y2) {
	  if ((*obsptr & (OBSTRUCT_MASK & ~OBSTRUCT_S)) == 0) {
	     if ((dist == 0) || ((dy - ds->y2) < dist))
		OBSINFO(gridx, gridy, ds->layer) = dy - ds->y2;
	     *obsptr |= OBSTRUCT_S;
	  }
	  else *obsptr |= OBSTRUCT_MASK;
       }
       if (dx <= ds->x1) {
	  if ((*obsptr & (OBSTRUCT_MASK & ~OBSTRUCT_E)) == 0) {
	     if ((dist == 0) || ((ds->x1 - dx) < dist))
		OBSINFO(gridx, gridy, ds->layer) = ds->x1 - dx;
             *obsptr |= OBSTRUCT_E;
	  }
	  else *obsptr |= OBSTRUCT_MASK;
       }
       else if (dx >= ds->x2) {
	  if ((*obsptr & (OBSTRUCT_MASK & ~OBSTRUCT_W)) == 0) {
	     if ((dist == 0) || ((dx - ds->x2) < dist))
		OBSINFO(gridx, gridy, ds->layer) = dx - ds->x2;
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

static double get_via_clear(int lay, int horiz, DSEG rect) {
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

static double get_route_clear(int lay, DSEG rect) {
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
/* Truncate gates to the set of tracks.  Warn about any gates	*/
/* with nodes that are clipped entirely outside the routing	*/
/* area.							*/
/*--------------------------------------------------------------*/

void clip_gate_taps(void)
{
    NET net; 
    NODE node;
    DPOINT dp, dpl;
    int i, lay;

    for (i = 0; i < Numnets; i++) {
	net = Nlnets[i];
	for (node = net->netnodes; node; node = node->next) {
	    dpl = NULL;
	    for (dp = (DPOINT)node->taps; dp; ) {

		lay = dp->layer;

		if (dp->gridx < 0 || dp->gridy < 0 ||
			dp->gridx >= NumChannelsX[lay] ||
			dp->gridy >= NumChannelsY[lay]) {
		    Fprintf(stderr, "Tap of port of node %d of net %s"
			" is outside of route area\n",
			node->nodenum, node->netname);

		    if (dpl == NULL)
			node->taps = dp->next;
		    else
			dpl->next = dp->next;

		    free(dp);
		    dp = (dpl == NULL) ? node->taps : dpl->next;
		}
		else {
		    dpl = dp;
		    dp = dp->next;
		}
	    }
	}
    }
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

void create_obstructions_from_gates(void)
{
    GATE g;
    DSEG ds;
    int i, gridx, gridy;
    double deltax, deltay, delta[MAX_LAYERS];
    double dx, dy, deltaxy;

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
		      double s, edist, xp, yp;

		      // Check Euclidean distance measure
		      s = LefGetRouteSpacing(ds->layer);

		      if (dx < (ds->x1 + s - deltax)) {
		         xp = dx + deltax - s;
			 edist = (ds->x1 - xp) * (ds->x1 - xp);
		      }
		      else if (dx > (ds->x2 - s + deltax)) {
		         xp = dx - deltax + s;
			 edist = (xp - ds->x2) * (xp - ds->x2);
		      }
		      else edist = 0;
		      if ((edist > 0) && (dy < (ds->y1 + s - deltay))) {
		         yp = dy + deltay - s;
			 edist += (ds->y1 - yp) * (ds->y1 - yp);
		      }
		      else if ((edist > 0) && (dy > (ds->y2 - s + deltay))) {
		         yp = dy - deltay + s;
			 edist += (yp - ds->y2) * (yp - ds->y2);
		      }
		      else edist = 0;

		      if ((edist + EPS) < (s * s)) {

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
		      else {
			 edist = 0;	// diagnostic break
		      }
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

		            double s, edist = 0.0, xp, yp;

		            // Check Euclidean distance measure
		            s = LefGetRouteSpacing(ds->layer);

		            if (dx < (ds->x1 + s - deltax)) {
		               xp = dx + deltax - s;
			       edist += (ds->x1 - xp) * (ds->x1 - xp);
		            }
		            else if (dx > (ds->x2 - s + deltax)) {
		               xp = dx - deltax + s;
			       edist += (xp - ds->x2) * (xp - ds->x2);
		            }
			    else edist = 0;
		            if ((edist > 0) && (dy < (ds->y1 + s - deltay))) {
		               yp = dy + deltay - s;
			       edist += (ds->y1 - yp) * (ds->y1 - yp);
		            }
		            else if ((edist > 0) && (dy > (ds->y2 - s + deltay))) {
		               yp = dy - deltay + s;
			       edist += (yp - ds->y2) * (yp - ds->y2);
		            }
			    else edist = 0;

		            if ((edist + EPS) < (s * s)) {

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

void expand_tap_geometry(void)
{
    DSEG ds, ds2;
    GATE g;
    int i;
    u_char expanded;

    for (g = Nlgates; g; g = g->next) {
	for (i = 0; i < g->nodes; i++) {
	    if (g->netnum[i] == 0) continue;
	    if (g->taps == NULL) continue;

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
/* create_obstructions_inside_nodes()				*/
/*								*/
/*  Fills in the Obs[][] grid from the position of each node	*/
/*  (net terminal), which may have multiple unconnected		*/
/*  positions.							*/
/*								*/
/*  Also fills in the Nodeinfo.nodeloc[] grid with the node	*/
/*  number, which causes the router to put a premium on		*/
/*  routing other nets over or under this position, to		*/
/*  discourage boxing in a pin position and making it 		*/
/*  unroutable.							*/
/*								*/
/*  This routine is split into two passes.  This pass adds	*/
/*  information for points inside node regions.			*/
/*								*/
/*  ARGS: none.							*/
/*  RETURNS: nothing						*/
/*  SIDE EFFECTS: none						*/
/*  AUTHOR:  Tim Edwards, June 2011, based on code by Steve	*/
/*	Beccue.							*/
/*--------------------------------------------------------------*/

void create_obstructions_inside_nodes(void)
{
    NODE node;
    NODEINFO lnode;
    GATE g;
    DSEG ds;
    u_int dir, mask, k;
    int i, gridx, gridy;
    double dx, dy, xdist;
    float dist;

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
			     int orignet = OBSVAL(gridx, gridy, ds->layer);

			     if ((orignet & ROUTED_NET_MASK & ~ROUTED_NET) == (u_int)node->netnum) {

				// Duplicate tap point, or pre-existing
				// route.   Don't re-process it if it is
				// a duplicate.
				if ((lnode = NODEIPTR(gridx, gridy, ds->layer))) if(lnode->nodeloc) {
				    gridy++;
				    continue;
				}
			     }

			     else if (!(orignet & NO_NET) &&
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
				gridy++;
				continue;
			     }

			     if (!(orignet & NO_NET)) {

				// A grid point that is within 1/2 route width
				// of a tap rectangle corner can violate metal
				// width rules, and so should declare a stub.
				
				mask = 0;
				dir = 0;
				dist = 0.0;
			        xdist = 0.5 * LefGetRouteWidth(ds->layer);

				if (dx >= ds->x2 - xdist) {
				   if (dy > ds->y2 - xdist + EPS) {
				      // Check northeast corner

				      if ((ds->x2 - dx) > (ds->y2 - dy)) {
					 // West-pointing stub
					 mask = STUBROUTE;
					 dir = NI_STUB_EW;
					 dist = ds->x2 - dx - 2.0 * xdist;
				      }
				      else {
					 // South-pointing stub
					 mask = STUBROUTE;
					 dir = NI_STUB_NS;
					 dist = ds->y2 - dy - 2.0 * xdist;
				      }

				   }
				   else if (dy < ds->y1 + xdist - EPS) {
				      // Check southeast corner

				      if ((ds->x2 - dx) > (dy - ds->y1)) {
					 // West-pointing stub
					 mask = STUBROUTE;
					 dir = NI_STUB_EW;
					 dist = ds->x2 - dx - 2.0 * xdist;
				      }
				      else {
					 // North-pointing stub
					 mask = STUBROUTE;
					 dir = NI_STUB_NS;
					 dist = ds->y1 - dy + 2.0 * xdist;
				      }
				   }
				}
				else if (dx <= ds->x1 + xdist) {
				   if (dy > ds->y2 - xdist + EPS) {
				      // Check northwest corner

				      if ((dx - ds->x1) > (ds->y2 - dy)) {
					 // East-pointing stub
					 mask = STUBROUTE;
					 dir = NI_STUB_EW;
					 dist = ds->x1 - dx + 2.0 * xdist;
				      }
				      else {
					 // South-pointing stub
					 mask = STUBROUTE;
					 dir = NI_STUB_NS;
					 dist = ds->y2 - dy - 2.0 * xdist;
				      }

				   }
				   else if (dy < ds->y1 + xdist - EPS) {
				      // Check southwest corner

				      if ((dx - ds->x2) > (dy - ds->y1)) {
					 // East-pointing stub
					 mask = STUBROUTE;
					 dir = NI_STUB_EW;
					 dist = ds->x1 - dx + 2.0 * xdist;
				      }
				      else {
					 // North-pointing stub
					 mask = STUBROUTE;
					 dir = NI_STUB_NS;
					 dist = ds->y1 - dy + 2.0 * xdist;
				      }
				   }
				}

			        OBSVAL(gridx, gridy, ds->layer)
			        	= (OBSVAL(gridx, gridy, ds->layer)
					   & BLOCKED_MASK) | (u_int)node->netnum | mask;
				lnode = SetNodeinfo(gridx, gridy, ds->layer);
				lnode->nodeloc = node;
				lnode->nodesav = node;
				lnode->stub = dist;
				lnode->flags |= dir;
			     }
			     else if ((orignet & NO_NET) && ((orignet & OBSTRUCT_MASK)
					!= OBSTRUCT_MASK)) {
				/* Handled on next pass */
			     }

			     // Check that we have not created a PINOBSTRUCT
			     // route directly over this point.
			     if (ds->layer < Num_layers - 1) {
			        k = OBSVAL(gridx, gridy, ds->layer + 1);
			        if (k & PINOBSTRUCTMASK) {
			           if ((k & ROUTED_NET_MASK) != (u_int)node->netnum) {
				       OBSVAL(gridx, gridy, ds->layer + 1) = NO_NET;
				       FreeNodeinfo(gridx, gridy, ds->layer + 1);
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

} /* void create_obstructions_inside_nodes( void ) */

/*--------------------------------------------------------------*/
/* create_obstructions_outside_nodes()				*/
/*								*/
/*  Fills in the Obs[][] grid from the position of each node	*/
/*  (net terminal), which may have multiple unconnected		*/
/*  positions.							*/
/*								*/
/*  Also fills in the Nodeinfo.nodeloc[] grid with the node	*/
/*  number, which causes the router to put a premium on		*/
/*  routing other nets over or under this position, to		*/
/*  discourage boxing in a pin position and making it 		*/
/*  unroutable.							*/
/*								*/
/*  This routine is split into two passes.  This pass adds	*/
/*  information for points outside node regions but close	*/
/*  enough to interact with the node.				*/
/*								*/
/*  ARGS: none.							*/
/*  RETURNS: nothing						*/
/*  SIDE EFFECTS: none						*/
/*  AUTHOR:  Tim Edwards, June 2011, based on code by Steve	*/
/*	Beccue.							*/
/*--------------------------------------------------------------*/


void create_obstructions_outside_nodes(void)
{
    NODE node, n2;
    NODEINFO lnode;
    GATE g;
    DSEG ds;
    u_int dir, mask, k;
    int i, gridx, gridy;
    double dx, dy, deltax, deltay, xdist;
    float dist;
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

			 // 2nd pass on area inside defined pin geometry,
			 // allowing terminal connections to be made using
			 // an offset tap, where possible.

			 if ((dy >= ds->y1 && gridy >= 0) && (dx >= ds->x1)
					&& (dy <= ds->y2) && (dx <= ds->x2)) {
			     int orignet = OBSVAL(gridx, gridy, ds->layer);

			     if ((orignet & ROUTED_NET_MASK) == (u_int)node->netnum) {

				// Duplicate tap point.   Don't re-process it.
				gridy++;
				continue;
			     }

			     if (!(orignet & NO_NET) &&
					((orignet & ROUTED_NET_MASK) != (u_int)0)) {
				/* Do nothing;  previously handled */
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
				// Nodeinfo.stub[] distance to move the tap away from
				// the obstruction to resolve the DRC error.

				// Make sure we have marked this as a node.
				lnode = SetNodeinfo(gridx, gridy, ds->layer);
				lnode->nodeloc = node;
				lnode->nodesav = node;
			        OBSVAL(gridx, gridy, ds->layer)
			        	= (OBSVAL(gridx, gridy, ds->layer)
					   & BLOCKED_MASK) | (u_int)node->netnum;

				if (orignet & OBSTRUCT_N) {
			           offd = -(sdisty - OBSINFO(gridx, gridy, ds->layer));
				   if (offd >= -offmaxy[ds->layer]) {
			              OBSVAL(gridx, gridy, ds->layer) |= OFFSET_TAP;
				      lnode->offset = offd;
				      lnode->flags |= NI_OFFSET_NS;

				      /* If position above has obstruction, then */
				      /* add up/down block to prevent vias.	 */

				      if ((ds->layer < Num_layers - 1) &&
						(gridy > 0) &&
						(OBSVAL(gridx, gridy - 1, ds->layer + 1)
						& OBSTRUCT_MASK))
					block_route(gridx, gridy, ds->layer, UP);
				   }
				   else maxerr = 1;
				}
				else if (orignet & OBSTRUCT_S) {
				   offd = sdisty - OBSINFO(gridx, gridy, ds->layer);
				   if (offd <= offmaxy[ds->layer]) {
			              OBSVAL(gridx, gridy, ds->layer) |= OFFSET_TAP;
				      lnode->offset = offd;
				      lnode->flags |= NI_OFFSET_NS;

				      /* If position above has obstruction, then */
				      /* add up/down block to prevent vias.	 */

				      if ((ds->layer < Num_layers - 1) &&
						(gridy < NumChannelsY[ds->layer + 1]
						- 1) &&
						(OBSVAL(gridx, gridy + 1, ds->layer + 1)
						& OBSTRUCT_MASK))
					block_route(gridx, gridy, ds->layer, UP);
				   }
				   else maxerr = 1;
				}
				else if (orignet & OBSTRUCT_E) {
				   offd = -(sdistx - OBSINFO(gridx, gridy, ds->layer));
				   if (offd >= -offmaxx[ds->layer]) {
			              OBSVAL(gridx, gridy, ds->layer) |= OFFSET_TAP;
				      lnode->offset = offd;
				      lnode->flags |= NI_OFFSET_EW;

				      /* If position above has obstruction, then */
				      /* add up/down block to prevent vias.	 */

				      if ((ds->layer < Num_layers - 1) &&
						(gridx > 0) &&
						(OBSVAL(gridx - 1, gridy, ds->layer + 1)
						& OBSTRUCT_MASK))
					block_route(gridx, gridy, ds->layer, UP);
				   }
				   else maxerr = 1;
				}
				else if (orignet & OBSTRUCT_W) {
				   offd = sdistx - OBSINFO(gridx, gridy, ds->layer);
				   if (offd <= offmaxx[ds->layer]) {
			              OBSVAL(gridx, gridy, ds->layer) |= OFFSET_TAP;
				      lnode->offset = offd;
				      lnode->flags |= NI_OFFSET_EW;

				      /* If position above has obstruction, then */
				      /* add up/down block to prevent vias.	 */

				      if ((ds->layer < Num_layers - 1) &&
						(gridx < NumChannelsX[ds->layer]
						- 1) &&
						(OBSVAL(gridx + 1, gridy, ds->layer + 1)
						& OBSTRUCT_MASK))
					block_route(gridx, gridy, ds->layer, UP);
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
			 }

		         if ((dy - EPS) > (ds->y1 - deltay) && gridy >= 0) {

			    double s, edist, xp, yp;
			    unsigned char epass = 0;

			    // Area inside halo around defined pin geometry.
			    // Exclude areas already processed (areas inside
			    // some pin geometry have been marked with netnum)

			    // Also check that we are not about to define a
			    // route position for a pin on a layer above 0 that
			    // blocks a pin underneath it.

			    // Flag positions that pass a Euclidean distance check.
			    // epass = 1 indicates that position clears a
			    // Euclidean distance measurement.

			    s = LefGetRouteSpacing(ds->layer);

			    if (dx < (ds->x1 + s - deltax)) {
				xp = dx + deltax - s;
				edist = (ds->x1 - xp) * (ds->x1 - xp);
			    }
			    else if (dx > (ds->x2 - s + deltax)) {
				xp = dx - deltax + s;
				edist = (xp - ds->x2) * (xp - ds->x2);
			    }
			    else edist = 0;
			    if ((edist > 0) && (dy < (ds->y1 + s - deltay))) {
				yp = dy + deltay - s;
				edist += (ds->y1 - yp) * (ds->y1 - yp);
			    }
			    else if ((edist > 0) && (dy > (ds->y2 - s + deltay))) {
				yp = dy - deltay + s;
				edist += (yp - ds->y2) * (yp - ds->y2);
			    }
			    else edist = 0;
			    if ((edist + EPS) > (s * s)) epass = 1;

			    xdist = 0.5 * LefGetRouteWidth(ds->layer);

			    n2 = NULL;
			    if (ds->layer > 0) {
			       lnode = NODEIPTR(gridx, gridy, ds->layer - 1);
			       n2 = (lnode) ? lnode->nodeloc : NULL;
			    }
			    if (n2 == NULL) {
			       lnode = NODEIPTR(gridx, gridy, ds->layer);
			       n2 = (lnode) ? lnode->nodeloc : NULL;
			    }
			    else {
			       // Watch out for the case where a tap crosses
			       // over a different tap.  Don't treat the tap
			       // on top as if it is not there!

			       NODE n3;
			       lnode = NODEIPTR(gridx, gridy, ds->layer);
			       n3 = (lnode) ? lnode->nodeloc : NULL;
			       if (n3 != NULL && n3 != node) n2 = n3;
			    }

			    // Ignore my own node.
			    if (n2 == node) n2 = NULL;

			    k = OBSVAL(gridx, gridy, ds->layer);

			    // In case of a port that is inaccessible from a grid
			    // point, or not completely overlapping it, the
			    // stub information will show how to adjust the
			    // route position to cleanly attach to the port.

			    mask = STUBROUTE;
			    dir = NI_STUB_NS | NI_STUB_EW;
			    dist = 0.0;

			    if (((k & ROUTED_NET_MASK) != (u_int)node->netnum)
					&& (n2 == NULL)) {

				if ((k & OBSTRUCT_MASK) != 0) {
				   float sdist = OBSINFO(gridx, gridy, ds->layer);

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
					 if ((dx - ds->x2 + dist) < xdist) {
				 	    mask = OFFSET_TAP;
				 	    dir = NI_OFFSET_EW;

				            if ((ds->layer < Num_layers - 1) &&
							(gridx > 0) &&
							(OBSVAL(gridx - 1, gridy,
							ds->layer + 1)
							& OBSTRUCT_MASK))
					       block_route(gridx, gridy, ds->layer, UP);
					 }
				      }
				      else if ((dx <= ds->x1) &&
						((k & OBSTRUCT_MASK) == OBSTRUCT_W)) {
				         dist = LefGetRouteKeepout(ds->layer) - sdist;
					 if ((ds->x1 - dx - dist) < xdist) {
				 	    mask = OFFSET_TAP;
				            dir = NI_OFFSET_EW;

				            if ((ds->layer < Num_layers - 1) &&
							gridx <
							(NumChannelsX[ds->layer] - 1)
							&& (OBSVAL(gridx + 1, gridy,
							ds->layer + 1)
							& OBSTRUCT_MASK))
					       block_route(gridx, gridy, ds->layer, UP);
					 }
				      }
			 	   }	
				   if (dx >= (ds->x1 - xdist) &&
						dx <= (ds->x2 + xdist)) {
				      if ((dy >= ds->y2) &&
						((k & OBSTRUCT_MASK) == OBSTRUCT_N)) {
				         dist = sdist - LefGetRouteKeepout(ds->layer);
					 if ((dy - ds->y2 + dist) < xdist) {
				 	    mask = OFFSET_TAP;
				            dir = NI_OFFSET_NS;

				            if ((ds->layer < Num_layers - 1) &&
							gridy < 
							(NumChannelsY[ds->layer] - 1)
							&& (OBSVAL(gridx, gridy - 1,
							ds->layer + 1)
							& OBSTRUCT_MASK))
					       block_route(gridx, gridy, ds->layer, UP);
					 }
				      }
				      else if ((dy <= ds->y1) &&
						((k & OBSTRUCT_MASK) == OBSTRUCT_S)) {
				         dist = LefGetRouteKeepout(ds->layer) - sdist;
					 if ((ds->y1 - dy - dist) < xdist) {
				 	    mask = OFFSET_TAP;
				            dir = NI_OFFSET_NS;

				            if ((ds->layer < Num_layers - 1) &&
							(gridy > 0) &&
							(OBSVAL(gridx, gridy + 1,
							ds->layer + 1)
							& OBSTRUCT_MASK))
					       block_route(gridx, gridy, ds->layer, UP);
					 }
				      }
				   }
				   // Otherwise, dir is left as NI_STUB_MASK
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
					 mask = STUBROUTE;
					 dir = NI_STUB_EW;
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
					 mask = STUBROUTE;
					 dir = NI_STUB_EW;
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
					 mask = STUBROUTE;
					 dir = NI_STUB_NS;
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
					 mask = STUBROUTE;
					 dir = NI_STUB_NS;
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

				   if ((mask == STUBROUTE) && (dir == NI_STUB_MASK)) {

				      // Outside of pin at a corner.  First, if one
				      // direction is too far away to connect to a
				      // pin, then we must route the other direction.

				      if (dx < ds->x1 - xdist || dx > ds->x2 + xdist) {
				         if (dy >= ds->y1 - xdist &&
							dy <= ds->y2 + xdist) {
					    mask = STUBROUTE;
				            dir = NI_STUB_EW;
				            dist = (float)(((ds->x1 + ds->x2) / 2.0)
							- dx);
					 }
				      }
				      else if (dy < ds->y1 - xdist ||
							dy > ds->y2 + xdist) {
					 mask = STUBROUTE;
				         dir = NI_STUB_NS;
				         dist = (float)(((ds->y1 + ds->y2) / 2.0) - dy);
				      }

				      // Otherwise we are too far away at a diagonal
				      // to reach the pin by moving in any single
				      // direction.  To be pedantic, we could define
				      // some jogged stub, but for now, we just call
				      // the point unroutable (leave dir = NI_STUB_MASK)

				      // To do:  Apply offset + stub
				   }
				}

				// Additional checks on stub routes

				// Stub distances of <= 1/2 route width are
				// unnecessary, so don't create them.

				if (mask == STUBROUTE && (dir == NI_STUB_NS
					|| dir == NI_STUB_EW) &&
					(fabs(dist) < (xdist + EPS))) {
				    mask = 0;
				    dir = 0;
				    dist = 0.0;
				}
				else if (mask == STUBROUTE && (dir == NI_STUB_NS
					|| dir == NI_STUB_EW)) {
				   struct dseg_ de;
				   DSEG ds2;
				   u_char errbox = TRUE;

				   // Additional check:  Sometimes the above
				   // checks put stub routes where they are
				   // not needed because the stub is completely
				   // covered by other tap geometry.  Take the
				   // stub area and remove parts covered by
				   // other tap rectangles.  If the entire
				   // stub is gone, then don't put a stub here.

				   if (dir == NI_STUB_NS) {
				       de.x1 = dx - xdist;
				       de.x2 = dx + xdist;
				       if (dist > 0) {
					  de.y1 = dy + xdist;
					  de.y2 = dy + dist;
				       }
				       else {
					  de.y1 = dy + dist;
					  de.y2 = dy - xdist;
				       }
				   }
				   if (dir == NI_STUB_EW) {
				       de.y1 = dy - xdist;
				       de.y2 = dy + xdist;
				       if (dist > 0) {
					  de.x1 = dx + xdist;
					  de.x2 = dx + dist;
				       }
				       else {
					  de.x1 = dx + dist;
					  de.x2 = dx - xdist;
				       }
				   }

				   // For any tap that overlaps the
				   // stub extension box, remove that
				   // part of the box.

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

				      // ds2 covers left side of de
				      if (ds2->x1 < de.x2 && ds2->x2 > de.x1 + EPS)
				         if (ds2->y1 < de.y2 && ds2->y2 > de.y1)
					    if (ds2->x1 < de.x1 + EPS &&
							ds2->x2 < de.x2 - EPS) {
					       de.x1 = ds2->x2;
					       if (de.x1 > de.x2 - EPS) errbox = FALSE;
					    }

				      // ds2 covers right side of de
				      if (ds2->x1 < de.x2 - EPS && ds2->x2 > de.x1)
				         if (ds2->y1 < de.y2 && ds2->y2 > de.y1)
					    if (ds2->x2 > de.x2 - EPS &&
							ds2->x1 > de.x1 + EPS) {
					       de.x2 = ds2->x1;
					       if (de.x2 < de.x1 + EPS) errbox = FALSE;
					    }

				      // ds2 covers bottom side of de
				      if (ds2->y1 < de.y2 && ds2->y2 > de.y1 + EPS)
				         if (ds2->x1 < de.x2 && ds2->x2 > de.x1)
					    if (ds2->y1 < de.y1 + EPS &&
							ds2->y2 < de.y2 - EPS) {
					       de.y1 = ds2->y2;
					       if (de.y1 > de.y2 - EPS) errbox = FALSE;
					    }

				      // ds2 covers top side of de
				      if (ds2->y1 < de.y2 - EPS && ds2->y2 > de.y1)
				         if (ds2->x1 < de.x2 && ds2->x2 > de.x1)
					    if (ds2->y2 > de.y2 - EPS &&
							ds2->y1 > de.y1 + EPS) {
					       de.y2 = ds2->y1;
					       if (de.y2 < de.y1 + EPS) errbox = FALSE;
					    }
			           }

				   // If nothing is left of the stub box,
				   // then remove the stub.

				   if (errbox == FALSE) {
				      mask = 0;
				      dir = 0;
				      dist = 0;
				   }
                                }

				lnode = SetNodeinfo(gridx, gridy, ds->layer);

				if ((k < Numnets) && (dir != NI_STUB_MASK)) {
				   OBSVAL(gridx, gridy, ds->layer)
				   	= (OBSVAL(gridx, gridy, ds->layer)
					  & BLOCKED_MASK) | (u_int)g->netnum[i] | mask; 
				   lnode->nodeloc = node;
				   lnode->nodesav = node;
				   lnode->flags |= dir;
				}
				else if ((OBSVAL(gridx, gridy, ds->layer)
					& NO_NET) != 0) {
				   // Keep showing an obstruction, but add the
				   // direction info and log the stub distance.
				   OBSVAL(gridx, gridy, ds->layer) |= mask;
				   lnode->flags |= dir;
				}
				else {
				   OBSVAL(gridx, gridy, ds->layer)
					|= (mask | (g->netnum[i] & ROUTED_NET_MASK));
				   lnode->flags |= dir;
				}
				if ((mask & STUBROUTE) != 0) {
				   lnode->stub = dist;
				}
				else if (((mask & OFFSET_TAP) != 0) || (dist != 0.0)) {
				   lnode->offset = dist;
				}
				
				// Remove entries with NI_STUB_MASK---these
				// are blocked-in taps that are not routable
				// without causing DRC violations (formerly
				// called STUBROUTE_X).

				if (dir == NI_STUB_MASK) {
				   disable_gridpos(gridx, gridy, ds->layer);
				}
			    }
			    else if (epass == 0) {

			       // Position fails euclidean distance check

			       int othernet = (k & ROUTED_NET_MASK);

			       if (othernet != 0 && othernet != (u_int)node->netnum) {

			          // This location is too close to two different
				  // node terminals and should not be used

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

				  // Such an example has come along, leading to
				  // an additional relaxation allowing an offset
				  // if the neighboring channel does not have a
				  // node record.  This will probably need
				  // revisiting.
				
				  if ((k & PINOBSTRUCTMASK) != 0)
				     disable_gridpos(gridx, gridy, ds->layer);
				  else if ((lnode = NODEIPTR(gridx, gridy, ds->layer))
					!= NULL && (lnode->nodesav != NULL)) {

				     u_char no_offsets = TRUE;
				     int offset_net;

				     // By how much would a tap need to be moved
				     // to clear the obstructing geometry?

				     // Check tap to right

				     if ((dx > ds->x2) && (gridx <
						NumChannelsX[ds->layer] - 1)) {
					offset_net = OBSVAL(gridx + 1, gridy, ds->layer);
					if (offset_net == 0 || offset_net == othernet) {
					   xdist = 0.5 * LefGetViaWidth(ds->layer,
							ds->layer, 0);
					   dist = ds->x2 - dx + xdist +
							LefGetRouteSpacing(ds->layer);
					   mask = OFFSET_TAP;
					   dir = NI_OFFSET_EW;
					   OBSVAL(gridx, gridy, ds->layer) |= mask;
					   lnode->offset = dist;
					   lnode->flags |= dir;
					   no_offsets = FALSE;

				           if ((ds->layer < Num_layers - 1) &&
						(gridx > 0) &&
						(OBSVAL(gridx + 1, gridy, ds->layer + 1)
						& OBSTRUCT_MASK))
					      block_route(gridx, gridy, ds->layer, UP);
					}
				     }

				     // Check tap to left

				     if ((dx < ds->x1) && (gridx > 0)) {
					offset_net = OBSVAL(gridx - 1, gridy, ds->layer);
					if (offset_net == 0 || offset_net == othernet) {
					   xdist = 0.5 * LefGetViaWidth(ds->layer,
							ds->layer, 0);
					   dist = ds->x1 - dx - xdist -
							LefGetRouteSpacing(ds->layer);
					   mask = OFFSET_TAP;
					   dir = NI_OFFSET_EW;
					   OBSVAL(gridx, gridy, ds->layer) |= mask;
					   lnode->offset = dist;
					   lnode->flags |= dir;
					   no_offsets = FALSE;

				           if ((ds->layer < Num_layers - 1) &&
						gridx < (NumChannelsX[ds->layer] - 1) &&
						(OBSVAL(gridx - 1, gridy, ds->layer + 1)
						& OBSTRUCT_MASK))
					      block_route(gridx, gridy, ds->layer, UP);
					}
				     }

				     // Check tap up

				     if ((dy > ds->y2) && (gridy <
						NumChannelsY[ds->layer] - 1)) {
					offset_net = OBSVAL(gridx, gridy + 1, ds->layer);
					if (offset_net == 0 || offset_net == othernet) {
					   xdist = 0.5 * LefGetViaWidth(ds->layer,
							ds->layer, 1);
					   dist = ds->y2 - dy + xdist +
							LefGetRouteSpacing(ds->layer);
					   mask = OFFSET_TAP;
					   dir = NI_OFFSET_NS;
					   OBSVAL(gridx, gridy, ds->layer) |= mask;
					   lnode->offset = dist;
					   lnode->flags |= dir;
					   no_offsets = FALSE;

				           if ((ds->layer < Num_layers - 1) &&
						(gridy > 0) &&
						(OBSVAL(gridx, gridy + 1, ds->layer + 1)
						& OBSTRUCT_MASK))
					      block_route(gridx, gridy, ds->layer, UP);
					}
				     }

				     // Check tap down

				     if ((dy < ds->y1) && (gridy > 0)) {
					offset_net = OBSVAL(gridx, gridy - 1, ds->layer);
					if (offset_net == 0 || offset_net == othernet) {
					   xdist = 0.5 * LefGetViaWidth(ds->layer,
							ds->layer, 1);
					   dist = ds->y1 - dy - xdist -
							LefGetRouteSpacing(ds->layer);
					   mask = OFFSET_TAP;
					   dir = NI_OFFSET_NS;
					   OBSVAL(gridx, gridy, ds->layer) |= mask;
					   lnode->offset = dist;
					   lnode->flags |= dir;
					   no_offsets = FALSE;

				           if ((ds->layer < Num_layers - 1) &&
						gridx < (NumChannelsX[ds->layer] - 1) &&
						(OBSVAL(gridx, gridy - 1, ds->layer + 1)
						& OBSTRUCT_MASK))
					      block_route(gridx, gridy, ds->layer, UP);
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

				  lnode = NODEIPTR(gridx, gridy, ds->layer);
				  xdist = 0.5 * LefGetViaWidth(ds->layer, ds->layer, 0);
				  if ((dy + xdist + LefGetRouteSpacing(ds->layer) >
					ds->y1) && (dy + xdist < ds->y1)) {
				     if ((dx - xdist < ds->x2) &&
						(dx + xdist > ds->x1) &&
						(lnode == NULL || lnode->stub
						== 0.0)) {
					OBSVAL(gridx, gridy, ds->layer)
				   		= (OBSVAL(gridx, gridy, ds->layer)
						& BLOCKED_MASK) |
						node->netnum | STUBROUTE;
				        lnode = SetNodeinfo(gridx, gridy, ds->layer);
					lnode->nodeloc = node;
					lnode->nodesav = node;
					lnode->stub = ds->y1 - dy;
					lnode->flags |= NI_STUB_NS;
				     }
				  }
				  if ((dy - xdist - LefGetRouteSpacing(ds->layer) <
					ds->y2) && (dy - xdist > ds->y2)) {
				     if ((dx - xdist < ds->x2) &&
						(dx + xdist > ds->x1) &&
						(lnode == NULL || lnode->stub
						== 0.0)) {
					OBSVAL(gridx, gridy, ds->layer)
				   		= (OBSVAL(gridx, gridy, ds->layer)
						& BLOCKED_MASK) |
						node->netnum | STUBROUTE;
				        lnode = SetNodeinfo(gridx, gridy, ds->layer);
					lnode->nodeloc = node;
					lnode->nodesav = node;
					lnode->stub = ds->y2 - dy;
					lnode->flags |= NI_STUB_NS;
				     }
				  }

				  xdist = 0.5 * LefGetViaWidth(ds->layer, ds->layer, 1);
				  if ((dx + xdist + LefGetRouteSpacing(ds->layer) >
					ds->x1) && (dx + xdist < ds->x1)) {
				     if ((dy - xdist < ds->y2) &&
						(dy + xdist > ds->y1) &&
						(lnode == NULL || lnode->stub
						 == 0.0)) {
					OBSVAL(gridx, gridy, ds->layer)
				   		= (OBSVAL(gridx, gridy, ds->layer)
						& BLOCKED_MASK) |
						node->netnum | STUBROUTE;
				        lnode = SetNodeinfo(gridx, gridy, ds->layer);
					lnode->nodeloc = node;
					lnode->nodesav = node;
					lnode->stub = ds->x1 - dx;
					lnode->flags |= NI_STUB_EW;
				     }
				  }
				  if ((dx - xdist - LefGetRouteSpacing(ds->layer) <
					ds->x2) && (dx - xdist > ds->x2)) {
				     if ((dy - xdist < ds->y2) &&
						(dy + xdist > ds->y1) &&
						(lnode == NULL || lnode->stub
						== 0.0)) {
					OBSVAL(gridx, gridy, ds->layer)
				   		= (OBSVAL(gridx, gridy, ds->layer)
						& BLOCKED_MASK) |
						node->netnum | STUBROUTE;
				        lnode = SetNodeinfo(gridx, gridy, ds->layer);
					lnode->nodeloc = node;
					lnode->nodesav = node;
					lnode->stub = ds->x2 - dx;
					lnode->flags |= NI_STUB_EW;
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

} /* void create_obstructions_outside_nodes( void ) */

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

void tap_to_tap_interactions(void)
{
    NODEINFO lnode;
    GATE g;
    DSEG ds;
    struct dseg_ de;
    int mingridx, mingridy, maxgridx, maxgridy;
    int i, gridx, gridy, net, orignet;
    double dx, dy;
    float dist;

    double deltax[MAX_LAYERS];
    double deltay[MAX_LAYERS];

    for (i = 0; i < Num_layers; i++) {
	deltax[i] = 0.5 * LefGetViaWidth(i, i, 0) + LefGetRouteSpacing(i);
	deltay[i] = 0.5 * LefGetViaWidth(i, i, 1) + LefGetRouteSpacing(i);
    }

    for (g = Nlgates; g; g = g->next) {
       for (i = 0; i < g->nodes; i++) {
	  net = g->netnum[i];
	  if (net != 0) {

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

		      orignet = OBSVAL(gridx, gridy, ds->layer);
		      if (orignet & OFFSET_TAP) {
			 orignet &= ROUTED_NET_MASK;
			 if (orignet != net) {

		            dx = (gridx * PitchX[ds->layer]) + Xlowerbound;
		            dy = (gridy * PitchY[ds->layer]) + Ylowerbound;

			    lnode = NODEIPTR(gridx, gridy, ds->layer);
			    dist = (lnode) ? lnode->offset : 0.0;

			    /* "de" is the bounding box of a via placed	  */
			    /* at (gridx, gridy) and offset as specified. */
			    /* Expanded by metal spacing requirement.	  */

			    de.x1 = dx - deltax[ds->layer];
			    de.x2 = dx + deltax[ds->layer];
			    de.y1 = dy - deltay[ds->layer];
			    de.y2 = dy + deltay[ds->layer];

			    if (lnode->flags & NI_OFFSET_NS) {
			       de.y1 += dist;
			       de.y2 += dist;
			    }
			    else if (lnode->flags & NI_OFFSET_EW) {
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
    NODEINFO lnode;
    GATE g;
    DSEG ds;
    int i, gridx, gridy;
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
			    int orignet = OBSVAL(gridx, gridy, ds->layer);

			    if (orignet & NO_NET) {
				OBSVAL(gridx, gridy, ds->layer) = g->netnum[i];
				lnode = SetNodeinfo(gridx, gridy, ds->layer);
				lnode->nodeloc = node;
				lnode->nodesav = node;
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

void adjust_stub_lengths(void)
{
    NODE node;
    NODEINFO lnode;
    GATE g;
    DSEG ds, ds2;
    struct dseg_ dt, de;
    int i, gridx, gridy, orignet, o;
    double dx, dy, wx, wy, s;
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

			     orignet = OBSVAL(gridx, gridy, ds->layer);

			     // Ignore this location if it is assigned to another
			     // net, or is assigned to NO_NET.

			     if ((orignet & ROUTED_NET_MASK) != node->netnum) {
				gridy++;
				continue;
			     }
			     lnode = NODEIPTR(gridx, gridy, ds->layer);

			     // Even if it's on the same net, we need to check
			     // if the stub is to this node, otherwise it is not
			     // an issue.
			     if (lnode->nodesav != node) {
				gridy++;
				continue;
			     }

			     // NI_STUB_MASK are unroutable;  leave them alone
			     if (orignet & STUBROUTE) {
				if ((lnode->flags & NI_OFFSET_MASK) == NI_OFFSET_MASK) {
				   gridy++;
				   continue;
				}
			     }

			     // define a route box around the grid point

			     errbox = FALSE;
			     dt.x1 = dx - wx;
			     dt.x2 = dx + wx;
			     dt.y1 = dy - wy;
			     dt.y2 = dy + wy;

			     // adjust the route box according to the stub
			     // or offset geometry, provided that the stub
			     // is longer than the route box.

			     if (orignet & OFFSET_TAP) {
			        dist = lnode->offset;
				if (lnode->flags & NI_OFFSET_EW) {
				   dt.x1 += dist;
				   dt.x2 += dist;
				}
				else if (lnode->flags & NI_OFFSET_NS) {
				   dt.y1 += dist;
				   dt.y2 += dist;
				}
			     }
			     else if (orignet & STUBROUTE) {
			        dist = (double)lnode->stub;
				if (lnode->flags & NI_STUB_EW) {
				   if (dist > EPS) {
				      if (dx + dist > dt.x2)
				 	 dt.x2 = dx + dist;
				   }
				   else {
				      if (dx + dist < dt.x1)
					 dt.x1 = dx + dist;
				   }
				}
				else if (lnode->flags & NI_STUB_NS) {
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

			     if ((dt.y1 - ds->y2) > EPS && (dt.y1 - ds->y2) + EPS < s) {
				if (ds->x2 > (dt.x1 - s) && ds->x1 < (dt.x2 + s)) {
				   de.y2 = dt.y1;
				   de.y1 = ds->y2;
				   if (ds->x2 + s < dt.x2) de.x2 = ds->x2 + s;
				   if (ds->x1 - s > dt.x1) de.x1 = ds->x1 - s;
				   errbox = TRUE;
				}
			     }
			     else if ((ds->y1 - dt.y2) > EPS && (ds->y1 - dt.y2) + EPS < s) {
				if (ds->x2 > (dt.x1 - s) && ds->x1 < (dt.x2 + s)) {
				   de.y1 = dt.y2;
				   de.y2 = ds->y1;
				   if (ds->x2 + s < dt.x2) de.x2 = ds->x2 + s;
				   if (ds->x1 - s > dt.x1) de.x1 = ds->x1 - s;
				   errbox = TRUE;
				}
			     }

			     if ((dt.x1 - ds->x2) > EPS && (dt.x1 - ds->x2) + EPS < s) {
				if (ds->y2 > (dt.y1 - s) && ds->y1 < (dt.y2 + s)) {
				   de.x2 = dt.x1;
				   de.x1 = ds->x2;
				   if (ds->y2 + s < dt.y2) de.y2 = ds->y2 + s;
				   if (ds->y1 - s > dt.y1) de.y1 = ds->y1 - s;
				   errbox = TRUE;
				}
			     }
			     else if ((ds->x1 - dt.x2) > EPS && (ds->x1 - dt.x2) + EPS < s) {
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

				// NOTE:  error box must touch ds geometry, and by
				// more than just a point.

				// 8/31/2016:
				// If DRC violations are created on two adjacent
				// sides, then create both a stub route and a tap
				// offset.  Put the stub route in the preferred
				// metal direction of the layer, and set the tap
				// offset to prevent the DRC error in the other
				// direction.
				// 10/3/2016:  The tap offset can be set either
				// by moving toward the obstructing edge to
				// remove the gap, or moving away from it to
				// avoid the DRC spacing error.  Choose the one
				// that offsets by the smaller distance.

				if ((de.x2 > dt.x2) && (de.y1 < ds->y2) &&
						(de.y2 > ds->y1)) {
				   if ((orignet & STUBROUTE) == 0) {
			              OBSVAL(gridx, gridy, ds->layer) |= STUBROUTE;
				      lnode->stub = de.x2 - dx;
				      lnode->flags |= NI_STUB_EW;
				      errbox = FALSE;
				   }
				   else if ((orignet & STUBROUTE)
						&& (lnode->flags & NI_STUB_EW)) {
				      // Beware, if dist > 0 then this reverses
				      // the stub.  For U-shaped ports may need
				      // to have separate E and W stubs.
				      lnode->stub = de.x2 - dx;
				      errbox = FALSE;
				   }
				   else if ((orignet & STUBROUTE)
						&& (lnode->flags & NI_STUB_NS)) {

				      // If preferred route direction is
				      // horizontal, then change the stub

			              OBSVAL(gridx, gridy, ds->layer) |= OFFSET_TAP;
				      if (LefGetRouteOrientation(ds->layer) == 1) {
					 lnode->flags = NI_OFFSET_NS | NI_STUB_EW;
					 // lnode->offset = lnode->stub;  // ?
					 if (lnode->stub > 0) {
					    lnode->offset = de.y2 - dy - wy;
					    if (lnode->offset > s - lnode->offset)
					         lnode->offset -= s;
					 }
					 else {
					    lnode->offset = de.y1 - dy + wy;
					    if (-lnode->offset > s + lnode->offset)
					        lnode->offset += s;
					 }
				         lnode->stub = de.x2 - dx;
				         errbox = FALSE;
				      }
				      else {
					 // Add the offset
				         lnode->offset = de.x2 - dx - wx;
					 if (lnode->offset > s - lnode->offset)
					     lnode->offset -= s;
					 lnode->flags |= NI_OFFSET_EW;
				         errbox = FALSE;
				      }
				   }
				}
				else if ((de.x1 < dt.x1) && (de.y1 < ds->y2) &&
						(de.y2 > ds->y1)) {
				   if ((orignet & STUBROUTE) == 0) {
			              OBSVAL(gridx, gridy, ds->layer) |= STUBROUTE;
				      lnode->stub = de.x1 - dx;
				      lnode->flags |= NI_STUB_EW;
				      errbox = FALSE;
				   }
				   else if ((orignet & STUBROUTE)
						&& (lnode->flags & NI_STUB_EW)) {
				      // Beware, if dist > 0 then this reverses
				      // the stub.  For U-shaped ports may need
				      // to have separate E and W stubs.
				      lnode->stub = de.x1 - dx;
				      errbox = FALSE;
				   }
				   else if ((orignet & STUBROUTE)
						&& (lnode->flags & NI_STUB_NS)) {

				      // If preferred route direction is
				      // horizontal, then change the stub

			              OBSVAL(gridx, gridy, ds->layer) |= OFFSET_TAP;
				      if (LefGetRouteOrientation(ds->layer) == 1) {
					 lnode->flags = NI_OFFSET_NS | NI_STUB_EW;
					 // lnode->offset = lnode->stub;  // ?
					 if (lnode->stub > 0) {
					    lnode->offset = de.y2 - dy - wy;
					    if (lnode->offset > s - lnode->offset)
					         lnode->offset -= s;
					 }
					 else {
					    lnode->offset = de.y1 - dy + wy;
					    if (-lnode->offset > s + lnode->offset)
					        lnode->offset += s;
					 }
				         lnode->stub = de.x1 - dx;
				         errbox = FALSE;
				      }
				      else {
					 // Add the offset
				         lnode->offset = de.x1 - dx + wx;
					 if (-lnode->offset > s + lnode->offset)
					     lnode->offset += s;
					 lnode->flags |= NI_OFFSET_EW;
				         errbox = FALSE;
				      }
				   }
				}
				else if ((de.y2 > dt.y2) && (de.x1 < ds->x2) &&
					(de.x2 > ds->x1)) {
				   if ((orignet & STUBROUTE) == 0) {
			              OBSVAL(gridx, gridy, ds->layer) |= STUBROUTE;
				      lnode->stub = de.y2 - dy;
				      lnode->flags |= NI_STUB_NS;
				      errbox = FALSE;
				   }
				   else if ((orignet & STUBROUTE)
						&& (lnode->flags & NI_STUB_NS)) {
				      // Beware, if dist > 0 then this reverses
				      // the stub.  For C-shaped ports may need
				      // to have separate N and S stubs.
				      lnode->stub = de.y2 - dy;
				      errbox = FALSE;
				   }
				   else if ((orignet & STUBROUTE)
						&& (lnode->flags & NI_STUB_EW)) {

				      // If preferred route direction is
				      // vertical, then change the stub

			              OBSVAL(gridx, gridy, ds->layer) |= OFFSET_TAP;
				      if (LefGetRouteOrientation(ds->layer) == 0) {
					 lnode->flags = NI_OFFSET_EW | NI_STUB_NS;
					 // lnode->offset = lnode->stub;  // ?
					 if (lnode->stub > 0) {
					    lnode->offset = de.x2 - dx - wx;
					    if (lnode->offset > s - lnode->offset)
					        lnode->offset -= s;
					 }
					 else {
					    lnode->offset = de.x1 - dx + wx;
					    if (-lnode->offset > s + lnode->offset)
					        lnode->offset += s;
					 }
				         lnode->stub = de.y2 - dy;
				         errbox = FALSE;
				      }
				      else {
					 // Add the offset
				         lnode->offset = de.y2 - dy - wy;
					 if (lnode->offset > s - lnode->offset)
					     lnode->offset -= s;
					 lnode->flags |= NI_OFFSET_NS;
				         errbox = FALSE;
				      }
				   }
				}
				else if ((de.y1 < dt.y1) && (de.x1 < ds->x2) &&
					(de.x2 > ds->x1)) {
				   if ((orignet & STUBROUTE) == 0) {
			              OBSVAL(gridx, gridy, ds->layer) |= STUBROUTE;
				      lnode->stub = de.y1 - dy;
				      lnode->flags |= NI_STUB_NS;
				      errbox = FALSE;
				   }
				   else if ((orignet & STUBROUTE)
						&& (lnode->flags & NI_STUB_NS)) {
				      // Beware, if dist > 0 then this reverses
				      // the stub.  For C-shaped ports may need
				      // to have separate N and S stubs.
				      lnode->stub = de.y1 - dy;
				      errbox = FALSE;
				   }
				   else if ((orignet & STUBROUTE)
						&& (lnode->flags & NI_STUB_EW)) {

				      // If preferred route direction is
				      // vertical, then change the stub

			              OBSVAL(gridx, gridy, ds->layer) |= OFFSET_TAP;
				      if (LefGetRouteOrientation(ds->layer) == 0) {
					 lnode->flags = NI_OFFSET_EW | NI_STUB_NS;
					 // lnode->offset = lnode->stub;  // ?
					 if (lnode->stub > 0) {
					    lnode->offset = de.x2 - dx - wx;
					    if (lnode->offset > s - lnode->offset)
					        lnode->offset -= s;
					 }
					 else {
					    lnode->offset = de.x1 - dx + wx;
					    if (-lnode->offset > s + lnode->offset)
					        lnode->offset += s;
					 }
				         lnode->stub = de.y1 - dy + wy;
				         errbox = FALSE;
				      }
				      else {
					 // Add the offset
				         lnode->offset = de.y1 - dy + wy;
					 if (-lnode->offset > s + lnode->offset)
					     lnode->offset += s;
					 lnode->flags |= NI_OFFSET_NS;
				         errbox = FALSE;
				      }
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
			           OBSVAL(gridx, gridy, ds->layer) |= STUBROUTE;
				   lnode->flags |= NI_STUB_MASK;
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
   
   ob = OBSVAL(bx, by, bl);

   if ((ob & NO_NET) != 0) return;

   switch (dir) {
      case NORTH:
	 OBSVAL(bx, by, bl) |= BLOCKED_S;
	 OBSVAL(x, y, lay) |= BLOCKED_N;
	 break;
      case SOUTH:
	 OBSVAL(bx, by, bl) |= BLOCKED_N;
	 OBSVAL(x, y, lay) |= BLOCKED_S;
	 break;
      case EAST:
	 OBSVAL(bx, by, bl) |= BLOCKED_W;
	 OBSVAL(x, y, lay) |= BLOCKED_E;
	 break;
      case WEST:
	 OBSVAL(bx, by, bl) |= BLOCKED_E;
	 OBSVAL(x, y, lay) |= BLOCKED_W;
	 break;
      case UP:
	 OBSVAL(bx, by, bl) |= BLOCKED_D;
	 OBSVAL(x, y, lay) |= BLOCKED_U;
	 break;
      case DOWN:
	 OBSVAL(bx, by, bl) |= BLOCKED_U;
	 OBSVAL(x, y, lay) |= BLOCKED_D;
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
   GATE g;
   NODEINFO lnode;
   DSEG ds;
   struct dseg_ dt;
   int i, gridx, gridy;
   double dx, dy, w, v, s, u;
   double dist;

   for (g = Nlgates; g; g = g->next) {
      for (i = 0; i < g->nodes; i++) {
	 if (g->netnum[i] != 0) {

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
		     lnode = NODEIPTR(gridx, gridy, ds->layer);
		     u = ((OBSVAL(gridx, gridy, ds->layer) & STUBROUTE)
				&& (lnode->flags & NI_STUB_EW)) ? v : w;
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
		     lnode = NODEIPTR(gridx, gridy, ds->layer);
		     u = ((OBSVAL(gridx, gridy, ds->layer) & STUBROUTE)
				&& (lnode->flags & NI_STUB_EW)) ? v : w;
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
		     lnode = NODEIPTR(gridx, gridy, ds->layer);
		     u = ((OBSVAL(gridx, gridy, ds->layer) & STUBROUTE)
				&& (lnode->flags & NI_STUB_NS)) ? v : w;
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
		     lnode = NODEIPTR(gridx, gridy, ds->layer);
		     u = ((OBSVAL(gridx, gridy, ds->layer) & STUBROUTE)
				&& (lnode->flags & NI_STUB_NS)) ? v : w;
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
