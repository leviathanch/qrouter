/*
 * mask.h --
 *
 * This file includes the route mask functions
 *
 */

#ifndef _MASKINT_H
#define _MASKINT_H

extern u_char *RMask;                // mask out best area to route

extern void initMask(void);
extern void fillMask(NET net, u_char value);
extern void setBboxCurrent(NET net);
void find_bounding_box(NET net);

#endif /* _MASKINT_H */
