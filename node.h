/*--------------------------------------------------------------*/
/* node.h -- general purpose autorouter                      	*/
/*--------------------------------------------------------------*/
/* Written by Tim Edwards, based on code by Steve Beccue, 2003  */
/*--------------------------------------------------------------*/

#ifndef NODE_H

#define CHECK_POINT_ABOVE_HLINE 0x1001
#define CHECK_POINT_UNDER_HLINE 0x1002
#define CHECK_POINT_HORIZONTAL CHECK_POINT_ABOVE_HLINE&CHECK_POINT_UNDER_HLINE

#define CHECK_POINT_LEFT_VLINE 0x2001
#define CHECK_POINT_RIGHT_VLINE 0x2002
#define CHECK_POINT_VERTICAL CHECK_POINT_LEFT_VLINE&CHECK_POINT_RIGHT_VLINE

#define UNDEF_NET	 0
#define GND_NET		 1
#define VDD_NET		 2
#define CLK_NET		 3
#define MIN_NET_NUMBER   4

#define COORDS(l) l->pt1->x,l->pt1->y,l->pt2->x,l->pt2->y

void create_netorder(u_char method);
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
BBOX new_bbox();
POINT create_point(int x, int y, int layer);
BOOL points_equal(POINT p1, POINT p2);
BOOL points_fully_equal(POINT p1, POINT p2);
BOOL gpoints_equal(GRIDP p1, GRIDP p2);
BBOX add_line_to_bbox(BBOX bbox, BBOX_LINE line);
BBOX delete_line_from_bbox(BBOX bbox, BBOX_LINE l);
POINT get_left_lower_trunk_point(BBOX bbox);
POINT get_right_upper_trunk_point(BBOX bbox);
int get_bbox_area(NET net);
int net_absolute_distance(NET net);
void free_line_list(BBOX_LINE t);
void fit_all_bboxes(NETLIST l);
BBOX shrink_bbox(BBOX orig, int num_pixels);
BBOX_LINE new_line();
BBOX_LINE clone_line(BBOX_LINE orig);
POINT clone_point(POINT p);
BOOL check_point_area(BBOX bbox, POINT pnt, BOOL with_edge, int edge_distance);
BOOL check_grid_point_area(BBOX bbox, GRIDP pnt, BOOL with_edge, int edge_distance);
BBOX clone_bbox(BBOX orig);
BOOL check_bbox_consistency(NET net, BBOX vbox);
BOOL is_vddnet(NET net);
BOOL is_gndnet(NET net);
BOOL is_clknet(NET net);
BBOX_LINE get_horizontal_lines(BBOX_LINE box);
BBOX_LINE get_vertical_lines(BBOX_LINE box);
BOOL check_bbox_collisions(NET net, BOOL thread);
NETLIST get_bbox_collisions(NET net, BOOL thread);
BOOL resolve_bbox_collisions(NET net, BOOL thread);
BOOL check_single_bbox_collision(BBOX box1, BBOX box2);
BOOL point_on_edge(BBOX box, POINT pnt);
BBOX_LINE add_line_to_edge(BBOX_LINE list, BBOX_LINE l);

#define NODE_H
#endif 


/* end of node.h */

