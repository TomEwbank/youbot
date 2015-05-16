/*-------------------------------------------
 * Mex function for the robot to choose his next destination in the process 
 * of exploring the area in order to build a map. 
 * 
 * arguments : - x & y, coordinates of a map cell
 *             - the map (a cell = 0 for an explored cell, 
 *                                 1 for an obstacle, 
 *                                 -1 for an unexplored cell)
 * 
 * This function looks (approximately) for one of the closest unexplored 
 * cells to the one passed as argument. It assumes that x and y  are valid 
 * coordinates, that it exists at least 1 unexplored (and explorable) cell 
 * on the map, and also that the direct 8 neighbours of the cell passed as 
 * argument aren't obstacles. 
 * -------------------------------------------------*/    


#include <math.h>
#include "mex.h"

typedef enum {RIGHT, TOP, LEFT, BOT} direction_t;

typedef struct loop_nodes
{
    int x;
    int y;
    direction_t direction;
    loop_nodes *next;
} loop_node_t;   

typedef struct loops
{
    loop_node_t *start;
    loop_node_t *end;
    size_t length;
} loop_t;


void add_node(int i, int j, direction_t d, loop_t *loop);
loop_t *init_loop(int i, int j, direction_t d);
void free_loop(loop_t *loop);
void expand(int i, int j, direction_t d1, direction_t d2, double *map, loop_t *loop);

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	#define X_IN prhs[0]
	#define Y_IN prhs[1]
	#define MAP_IN prhs[2]

	#define P_OUT plhs[0]
    
    int i = mxGetScalar(X_IN);
    int j = mxGetScalar(Y_IN)};
    int M = mxGetM(MAP_IN);
    
    double *map, *p_out;
    
    loop_t *loop, *prev_loop;
    loop_node_t *node;
    direction_t prev_direction;
    
    bool pointFound = 0;
    
    map = mxGetPr(MAP_IN);
    P_OUT = mxCreateDoubleMatrix(1, 2, mxREAL);
    p_out = mxGetPr(P_OUT);
    
    /* initializing the loop by the ones surrounding the starting point */
    prev_loop = init_loop(i+1, j, TOP);
    add_node(i+1,j+1,LEFT);
    add_node(i,j+1,LEFT);
    add_node(i-1,j+1,BOT);
    add_node(i-1,j,BOT);
    add_node(i-1,j-1,RIGHT);
    add_node(i,j-1,RIGHT);
    add_node(i+1,j-1,TOP);
    add_node(i+1,j,TOP);
    
    while(!pointFound)
    {    
        node = prev_loop->start;

        /* Setting  the first nodes of the expanded loop */
        if(map[node->x+2 + M*node->y] == 1)
                loop = init_loop(node->x+1, node->y, TOP);
        else
        {
            if(map[node->x+2 + M*(node->y+1)] == 1)
            {   
                loop = init_loop(node->x+2, node->y, LEFT);
                add_node(node->x+1, node->y, TOP);
            }
            else
                loop = init_loop(node->x+2, node->y, TOP);
        }
        
        /* Adding nodes to the expanded loop */
        prev_direction = node->direction;
        node = node->next;
        while(node != null)
        {
            expand(node->x, node->y, prev_direction, node->direction, map, loop);
            prev_direction = node->direction;
            node = node->next;
        }
    
        /* Searching in the expanded loop for an unexplored points */
        node = loop->start;
        while(node != null)
        {
            if(map[node->x + M*node->y] == -1)
            {
                p_out[0] = node->x;
                p_out[1] = node->y;
                pointFound = 1;
            }
            node = node->next;
        }
        
        free_loop(prev_loop);
        prev_loop = loop;
    }
    
    free_loop(prev_loop);
    return;
}

void add_node(int i, int j, direction_t d, loop_t *loop)
{
    loop_node_t *node = malloc(sizeof(loop_node_t));
    node->x = i;
    node->y = j;
    node->direction = d;
    node->next = NULL;
    
    loop->end->next = node;
    loop->end = node;
    loop->length++; 
}

loop_t *init_loop(int i, int j, direction_t d)
{
    loop_t *l = malloc(sizeof(loop_t));
    l->length = 2;
    l->start = malloc(sizeof(loop_node_t));
    l->start->x = i;
    l->start->y = j;
    l->start->direction = d;
    l->start->next = NULL;
    l->end = l->start;
}
    
void free_loop(loop_t *loop)
{
    loop_node_t *current = loop->start;
    loop_node_t *next;
    
    while(current->next != NULL)
    {
        next = current->next;
        free(current);
        current = next;
    }
    
    free(current);
    free(loop);
}
    
void expand(int i, int j, direction_t d1, direction_t d2, double *map, loop_t *loop)
{
    int m, n, back_m, back_n, front_m, front_n;
    direction_t next_d;
    m = i;
    n = j;
      
    if(d1 == d2)
    {
        switch(d2)
        {
            case TOP: 
                ++m; 
                back_m = m;
                front_m = m;
                back_n = n-1;
                front_n = n+1;
                next_d = LEFT;
                break;
            
            case BOT: 
                --m; 
                back_m = m;
                front_m = m;
                back_n = n+1;
                front_n = n-1;
                next_d = RIGHT;
                break;
                
            case LEFT: 
                ++n; 
                back_m = m+1;
                front_m = m-1;
                back_n = n;
                front_n = n;
                next_d = BOT;
                break;
                
            case RIGHT: 
                --n;
                back_m = m-1;
                front_m = m+1;
                back_n = n;
                front_n = n;
                next_d = TOP;
                break;
        }
        
//         if(map[m+ M*n] != 1)
//         {
//             if(map[front_m + M*front_n] == 1)
//             {
//                 add_node(m,n,next_d,loop);
//                 add_node(i,j,d2,loop);
//             }
//             else
//                 add_node(m,n,d2,loop);
//         }
//         else
        
        if(map[m+ M*n] != 1)
            add_node(m,n,d2,loop);
        else
            add_node(i,j,d2,loop);
        
    }
    else
    {
        
            
            
        
            
    
    
    
    
    
    
    
    
    
    
    
    
    
    