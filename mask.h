/*
 * mask.h --
 *
 * This file includes the route mask functions
 *
 */

#ifndef _MASKINT_H
#define _MASKINT_H

u_char *RMask;                // mask out best area to route

void createMask(NET net, u_char slack, u_char halo);
void initMask(void);
void fillMask(NET net, u_char value);
void setBboxCurrent(NET net);
void createBboxMask(NET net, u_char halo);
void find_bounding_box(NET net);
void create_hbranch_mask(NET net, int y, int x1, int x2, u_char slack, u_char halo);
void create_vbranch_mask(NET net, int x, int y1, int y2, u_char slack, u_char halo);

#endif /* _MASKINT_H */
