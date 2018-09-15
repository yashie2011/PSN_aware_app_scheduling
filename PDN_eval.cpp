// ******* # of power pins still remain 16/core -- but # of cores=60 now ************
#include "Constants.h"

void PDN_eval ( grid_node*** m, int mesh_dim_x, int mesh_dim_y, int num_tiers)
{	//double t_window = 1000.0; // For now, lets fix the window (secs) during which the the PDN (and computation) profiles remains constant --- update later --
	int i, j, k, l, m1, count; // ijk tile co-ords and l,m1 are the PDN input co-ords within each core (from 0-15)
	int num_cores;
	num_cores = mesh_dim_x*mesh_dim_y*num_tiers;
	if (num_cores != DIM_X * DIM_Y * DIM_Z)
	{
		cout << " num_cores != 60 -- check " << endl;
		exit(1);
	}
	double voltages[DIM_X][DIM_Y][DIM_Z]; //manintains sum of all Vdds of individual cores-- then we divide by sqr_num_nodes to get the avg. Vdd per core-- then, push into Vdds_vec of m --
	// initialize voltages --
	for (i=0; i<mesh_dim_x; i++) {
		for (j=0; j<mesh_dim_y; j++) {
			for (k=0; k<num_tiers; k++) {
				voltages[i][j][k] = m[i][j][k].current_Vdd;
			}
		}
	}


} // end PDN_eval
