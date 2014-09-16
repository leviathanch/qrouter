/*--------------------------------------------------------------*/
/* node.h -- general purpose autorouter                      	*/
/*--------------------------------------------------------------*/
/* Written by Tim Edwards, based on code by Steve Beccue, 2003  */
/*--------------------------------------------------------------*/

#ifndef NODE_H

#define GND_NET		 1
#define VDD_NET		 2
#define MIN_NET_NUMBER   3

void create_netorder(u_char method);
void find_bounding_box(NET net);
void create_obstructions_from_nodes();
void create_obstructions_from_gates();
void create_obstructions_from_variable_pitch();
void tap_to_tap_interactions();
void make_routable(NODE node);
void adjust_stub_lengths();
void expand_tap_geometry();

void print_nodes(char *filename);
void print_nlnets(char *filename);

#define NODE_H
#endif 


/* end of node.h */

