/*--------------------------------------------------------------*/
/* node.h -- general purpose autorouter                      	*/
/*--------------------------------------------------------------*/
/* Written by Tim Edwards, based on code by Steve Beccue, 2003  */
/*--------------------------------------------------------------*/

#ifndef NODE_H

#define UNDEF_NET	 0
#define GND_NET		 1
#define VDD_NET		 2
#define CLK_NET		 3
#define MIN_NET_NUMBER   4

void create_netorder(u_char method);
void find_bounding_box(NET net);
void define_route_tree(NET);
void print_nodes(char *filename);
void print_nlnets(char *filename);
void count_reachable_taps();
void check_variable_pitch(int, int *, int *);
void create_obstructions_from_variable_pitch(void);
void count_pinlayers();
void create_obstructions_from_gates(void);
void expand_tap_geometry(void);
void create_obstructions_inside_nodes(void);
void create_obstructions_outside_nodes(void);
void tap_to_tap_interactions(void);
void make_routable(NODE node);
void adjust_stub_lengths(void);
void block_route(int x, int y, int lay, u_char dir);
void find_route_blocks();
void clip_gate_taps(void);
BBOX create_fresh_bbox();
BBOX_POINT create_bbox_point(int x, int y);
BBOX add_line_to_bbox(BBOX bbox, BBOX_LINE line);
BBOX_LINE clone_line(BBOX_LINE orig);
BBOX_POINT clone_bbox_point(BBOX_POINT p);
BOOL check_contains_point(BBOX bbox, BBOX_POINT pnt);
BBOX clone_bbox(BBOX orig);
BOOL check_bbox_consistency(NET net, BBOX vbox);
BOOL is_vddnet(NET net);
BOOL is_gndnet(NET net);
BOOL is_clknet(NET net);
BBOX_POINT add_to_edge(BBOX_POINT edge, BBOX_POINT point);
BBOX_LINE get_horizontal_lines(BBOX box);
BBOX_LINE get_vertical_lines(BBOX box);
BOOL check_bbox_collisions(NET net);
POSTPONED_NET get_bbox_collisions(NET net);
BOOL resolve_bbox_collisions(NET net);
void add_clknet(NET net);
void add_gndnet(NET net);
void add_vddnet(NET net);
void free_postponed(POSTPONED_NET postponed);
void free_bbox(BBOX t);
void free_bbox_points(BBOX_POINT t);
#define NODE_H
#endif 


/* end of node.h */

