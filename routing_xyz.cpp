// For now, lets not consider comm_power (no use of ORION necessary) and assume the core power to be the tile-power (avg. power from SNIPER tool includes both router and core power) -- 
// -- We are not optimizing power or energy, power only determines temperature (which in turn determines aging) which is same for the entire tile -- 
//    -- router and core aging differentiated by their individual active times only --
// -- Also, block-placement is determined by considering just the Vt values of cores, while router Vt values are only used as a constraint -- so, as to optimize leakage in cores -- 
//     -- Aging-retardation (Vt profile) in routers only considered during routing -- 
// Thus, PC-satisfaction would not be required to be re-checked after routing -- 
//*****************************************************************************************************
// With 3D mesh, Up links are towards k=0 and Dn links are towards k=(num_tiers-1) ----
//*****************************************************************************************************

#include "Constants.h"

void link_BW_paths (app_DoP_pair &app_element, grid_node*** m1);
void build_paths(node* node_pointers[], grid_node*** m1, vector <int> path, int source_ID, int dest_ID, double volume, const int mesh_dim_x, const int mesh_dim_y, const int num_tiers);

void routing_xyz(app_DoP_pair &app_element, grid_node*** m1,
		node* node_pointers[], const int mesh_dim_x, const int mesh_dim_y,
		const int num_tiers) {
	int i, j, k; // loop variables	
	int freq;  freq = m1[app_element.x_cord][app_element.y_cord][app_element.z_cord].frequency;
	link_BW_paths (app_element, m1);		// func. to compute the link BWs (not volumes) and store in the grid

	int source_ID, dest_ID;  vector <int> path; double volume;  int x_diff, y_diff, z_diff;
	for (i=0; i<app_element.DoP; i++) {
		for (j=0; j<node_pointers[i]->out_degree; j++) {
			volume = node_pointers[i]->out_v_volume[j];
			source_ID = node_pointers[i]->ID;	dest_ID = node_pointers[i]->out_v_connect[j];
			x_diff = abs(node_pointers[source_ID-1]->x_coord - node_pointers[dest_ID-1]->x_coord);
			y_diff = abs(node_pointers[source_ID-1]->y_coord - node_pointers[dest_ID-1]->y_coord);
			z_diff = abs(node_pointers[source_ID-1]->z_coord - node_pointers[dest_ID-1]->z_coord);
			path.clear();	
			for (k=0; k<x_diff; k++) path.push_back(1);
			for (k=0; k<y_diff; k++)  path.push_back(0);
			for (k=0; k<z_diff; k++)  path.push_back(2);
			if (path.size() > 8) { cout << " ERROR: routing -- path too long for even a 4x4x2 box " << endl; exit(1); }  // -- verify that any path should be shorter than 7 hops ---
			build_paths(node_pointers, m1, path, source_ID, dest_ID, volume, mesh_dim_x, mesh_dim_y, num_tiers);		
		}
	} // end for i

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

	/*cout << " link comm distribution --  ";
	cout << " print out the link volumes on the mesh " << endl;
	for (j=0; j<mesh_dim_y; j++){
		for (i=0; i<mesh_dim_x; i++) {
			for (k=0; k<num_tiers; k++) {
			cout << m1[i][j][k].E_out_link << " " << m1[i][j][k].W_out_link << " " << m1[i][j][k].N_out_link << " " << m1[i][j][k].S_out_link << "	" << m1[i][j][k].Up_out_link << "	" << m1[i][j][k].Dn_out_link;
		} }
	}
	*/
	//*******************************************************************************************************************************
	
	// some final outputs
	long int total_traffic = 0;
	double AT, AT_max=0;  // lets assume AT in secs, power in Watts and energy in joules

	for (i=0; i<app_element.DoP; i++)
	{   AT_max = 0;  
		//cout << " ID is " << m1[node_pointers[i]->x_coord-1][node_pointers[i]->y_coord-1].ID << " " << node_pointers[i]->ID << " coords " << node_pointers[i]->x_coord << " " << node_pointers[i]->y_coord << endl;
		//cout << " E_out_link " << m1[node_pointers[i]->x_coord-1][node_pointers[i]->y_coord-1].E_out_link << endl;
		
	//****************************************************************************************************************************************************************************************
	
		// *************** EAST LINK **************************
		if (node_pointers[i]->x_coord < (app_element.x_cord + app_element.dim_x - 1)) {
		if ((m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].E_out_link > 0))  // non-zero BW, then count the link
		{
			if (freq != m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].frequency) { cout << " ERROR: routing freq. mismatch " << endl; exit(1); }
			// find the AT - (volume/BW)-- (Mb/(Mb/sec))
			AT = double ((double)m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].E_out_link/(double)m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].E_out_BW);
		//	cout << endl << "EAST-link:  AT (in secs)" << AT << " volume and BW in Mb are " << m1[node_pointers[i]->x_coord-1][node_pointers[i]->y_coord-1][node_pointers[i]->z_coord-1].E_out_link << " " << m1[node_pointers[i]->x_coord-1][node_pointers[i]->y_coord-1][node_pointers[i]->z_coord-1].E_out_BW << endl;
			m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].E_out_AT = AT;
			if (AT>AT_max) AT_max = AT;
			
			total_traffic = total_traffic + m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].E_out_link;

		}// end if
		} // end if EAST
		
// ****************** WEST LINK ************************************************************************
	if (node_pointers[i]->x_coord > app_element.x_cord) {	
		if ((m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].W_out_link > 0))  // non-zero BW, then count the link for power-energy
		{
			if (freq != m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].frequency) { cout << " ERROR: routing freq. mismatch " << endl; exit(1); }
			// find the AT - (volume/BW)-- 
			AT = double ((double)m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].W_out_link/(double)m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].W_out_BW);
		//	cout << endl << "WEST-link:  AT (in secs)" << AT << " volume and BW in Mb are " << m1[node_pointers[i]->x_coord-1][node_pointers[i]->y_coord-1][node_pointers[i]->z_coord-1].W_out_link << " " << m1[node_pointers[i]->x_coord-1][node_pointers[i]->y_coord-1][node_pointers[i]->z_coord-1].W_out_BW << endl;
			m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].W_out_AT = AT;
			if (AT>AT_max) AT_max = AT;
				
			total_traffic = total_traffic + m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].W_out_link;

		}// end if
	} // end if WEST

		// ****************** SOUTH LINK ************************************************************************
	if (node_pointers[i]->y_coord < (app_element.y_cord + app_element.dim_y - 1)) {
		if ((m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].S_out_link > 0))  // non-zero BW, then count the link for power-energy
		{
			if (freq != m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].frequency) { cout << " ERROR: routing freq. mismatch " << endl; exit(1); }
			// find the AT - (volume/BW)-- 
			AT = double ((double)m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].S_out_link/(double)m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].S_out_BW);
			//cout << endl << "SOUTH-link:  AT (in secs)" << AT << " volume and BW in Mb are " << m1[node_pointers[i]->x_coord-1][node_pointers[i]->y_coord-1][node_pointers[i]->z_coord-1].S_out_link << " " << m1[node_pointers[i]->x_coord-1][node_pointers[i]->y_coord-1][node_pointers[i]->z_coord-1].S_out_BW << endl;
			m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].S_out_AT = AT;
			if (AT>AT_max) AT_max = AT;
		
		total_traffic = total_traffic + m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].S_out_link;
		} // end if
	} // end if SOUTH
	
	// ****************** NORTH LINK ************************************************************************
	if (node_pointers[i]->y_coord > app_element.y_cord) {	
	
		if (m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].N_out_link > 0) // non-zero BW, then count the link for power-energy
		{
			if (freq != m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].frequency) { cout << " ERROR: routing freq. mismatch " << endl; exit(1); }
			// find the AT - (volume/BW)-- 
			AT = double ((double)m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].N_out_link/(double)m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].N_out_BW);
			m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].N_out_AT = AT;
			//cout << endl << "NORTH-link:  AT (in secs)" << AT << " volume and BW in Mb are " << m1[node_pointers[i]->x_coord-1][node_pointers[i]->y_coord-1][node_pointers[i]->z_coord-1].N_out_link << " " << m1[node_pointers[i]->x_coord-1][node_pointers[i]->y_coord-1][node_pointers[i]->z_coord-1].N_out_BW << endl;
			if (AT>AT_max) AT_max = AT;
		
			total_traffic = total_traffic + m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].N_out_link;

		} // end if

	} // end if NORTH
		
	// ****************** Up LINK ************************************************************************
	if (node_pointers[i]->z_coord > app_element.z_cord) {	
		if (m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].Up_out_link > 0) // non-zero BW, then count the link for power-energy
		{
			if (freq != m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].frequency) { cout << " ERROR: routing freq. mismatch " << endl; exit(1); }
			// find the AT - (volume/BW)-- 
			AT = double ((double)m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].Up_out_link/(double)m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].Up_out_BW);
			m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].Up_out_AT = AT;
			if (AT>AT_max) AT_max = AT;
			total_traffic = total_traffic + m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].Up_out_link;
		} // end if
	} // end if Up

		// ****************** Dn LINK ************************************************************************
	if (node_pointers[i]->z_coord < (app_element.z_cord + app_element.dim_z - 1)) {
		if ((m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].Dn_out_link > 0))  // non-zero BW, then count the link for power-energy
		{
			if (freq != m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].frequency) { cout << " ERROR: routing freq. mismatch " << endl; exit(1); }
			// find the AT - (volume/BW)-- 
			AT = double ((double)m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].Dn_out_link/(double)m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].Dn_out_BW);
			m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].Dn_out_AT = AT;
			if (AT>AT_max) AT_max = AT;
			total_traffic = total_traffic + m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].Dn_out_link;
		} // end if
	} // end if Dn
	// **************************Router delays **********************************************
	m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].AT_max = AT_max;
 } // end for i
// **************************** link delays done ************************************

	cout << endl << " total_traffic (Mbps) = " << total_traffic << endl;

	// -- from AT_max, calculate the run-times and finish-times of each core --
	// -- then synchronize finish times of all cores to the latest finish-time for the app --- so as to terminate this app later --
	double max_finish = 0;	double max_run_time = 0;
	for (i=app_element.x_cord; i<app_element.x_cord+app_element.dim_x; i++) {
	  for (j=app_element.y_cord; j<app_element.y_cord+app_element.dim_y; j++) {
		  for (k=app_element.z_cord; k<app_element.z_cord+app_element.dim_z; k++) {
			  m1[i][j][k].run_time = m1[i][j][k].compute_time + m1[i][j][k].AT_max; // run-time (core specific)
			  if (max_run_time < m1[i][j][k].run_time) max_run_time = m1[i][j][k].run_time;
			  if (m1[i][j][k].run_time > app_element.run_time_constraint) { cout << " app_run-time constraint violation " << endl;  exit(1); }
			  m1[i][j][k].finish_time = app_element.start_time + m1[i][j][k].run_time;
			  if (m1[i][j][k].finish_time > max_finish) max_finish = m1[i][j][k].finish_time;
		  }
	  }
	}
	// although, finish_times are common for all cores of the app -- AT_max and runtimes are distinct for every core -----
	for (i=app_element.x_cord; i<app_element.x_cord+app_element.dim_x; i++) {
	  for (j=app_element.y_cord; j<app_element.y_cord+app_element.dim_y; j++) {
		 for (k=app_element.z_cord; k<app_element.z_cord+app_element.dim_z; k++) {
			 if (m1[i][j][k].finish_time > max_finish) { cout << " ERROR: finish-times in routing func. " << endl; exit(1); } 
			 m1[i][j][k].finish_time = max_finish;
		 }
	  }
	}
	cout << " max_run_time is: " << max_run_time << endl;
} // end routing_xyz

// note that the inter-core communication is in terms of volume (Mb)
void link_BW_paths (app_DoP_pair &app_element, grid_node*** m1) { // assign the max. BWs to all the links in the grid
	int i, j, k;
	for (i=app_element.y_cord; i<app_element.y_cord + app_element.dim_y; i++)
	{
		for (j=app_element.x_cord; j<app_element.x_cord + app_element.dim_x; j++)
		{
			for (k=app_element.z_cord; k<app_element.z_cord + app_element.dim_z; k++)
			{
				if (j!=app_element.x_cord) {
					m1[j][i][k].W_out_BW = 32.0*(double)(m1[j][i][k].frequency);  if (m1[j][i][k].W_out_BW < 1) { cout << " BW is 0 ?? " << endl; exit(1);} }
				if (j != (app_element.x_cord+app_element.dim_x-1)) {
					m1[j][i][k].E_out_BW = 32.0*(double)m1[j][i][k].frequency; if (m1[j][i][k].E_out_BW < 1) { cout << " BW is 0 ?? " << endl; exit(1);} }
				if (i!=app_element.y_cord) {
					m1[j][i][k].N_out_BW = 32.0*(double)m1[j][i][k].frequency; if (m1[j][i][k].N_out_BW < 1) { cout << " BW is 0 ?? " << endl; exit(1);} }
				if (i != app_element.y_cord+app_element.dim_y-1) {
					m1[j][i][k].S_out_BW = 32.0*(double)m1[j][i][k].frequency; if (m1[j][i][k].S_out_BW < 1) { cout << " BW is 0 ?? " << endl; exit(1);} }
				if (k!=app_element.z_cord) {
					m1[j][i][k].Up_out_BW = 32.0*(double)m1[j][i][k].frequency; if (m1[j][i][k].Up_out_BW < 1) { cout << " BW is 0 ?? " << endl; exit(1);} }
				if (k != app_element.z_cord+app_element.dim_z-1) {
					m1[j][i][k].Dn_out_BW = 32.0*(double)m1[j][i][k].frequency; if (m1[j][i][k].Dn_out_BW < 1) { cout << " BW is 0 ?? " << endl; exit(1);} }
			}
		}
	}
}//end link_BW
					
void build_paths(node* node_pointers[], grid_node*** m1, vector <int> path, int source_ID, int dest_ID, double volume, const int mesh_dim_x, const int mesh_dim_y, const int num_tiers)
{
	int i, j, k, c; //loop counters
	int source_x, source_y, source_z, dest_x, dest_y, dest_z, x_direc, y_direc, z_direc;
	//given the path array in 0's and 1's-- 1's are for x-hops
	//build the links for the given path

	source_x = node_pointers[source_ID-1]->x_coord;
	source_y = node_pointers[source_ID-1]->y_coord;
	source_z = node_pointers[source_ID-1]->z_coord;
	dest_x = node_pointers[dest_ID-1]->x_coord;
	dest_y = node_pointers[dest_ID-1]->y_coord;
	dest_z = node_pointers[dest_ID-1]->z_coord;
	
	if((source_x - dest_x) > 0) x_direc = -1;	// west
	else if ((source_x - dest_x) < 0) x_direc = 1;	// east
	else if ((source_x - dest_x) == 0) x_direc = 0;

	if ((source_y - dest_y) >0) y_direc = -1;	// north
	else if ((source_y - dest_y) < 0) y_direc = 1;	// south
	else if ((source_y - dest_y) == 0) y_direc = 0;

	if((source_z - dest_z) > 0) z_direc = -1;	// Up
	else if ((source_z - dest_z) < 0) z_direc = 1;	// Dn
	else if ((source_z - dest_z) == 0) z_direc = 0;

	if ((x_direc == 0) && (y_direc == 0) && (z_direc == 0)) { cout << " ERROR: All directions are 0 -- source_ID  dest_ID " << source_ID << " " << dest_ID << endl;
	cout << " source " << source_x << " " << source_y << " dest " << dest_x << " " << dest_y << endl;  exit(1); }

	j = source_x;		i = source_y;    c = source_z;    int path_length;  path_length = path.size();
	//-- in addition to allocating volumes to path links, now we also need to calculate the path-delays (flow_times) according to the assumptions given at the top of this program 
	for (k=0; k<path_length; k++)
	{
		if (path[k] == 0) // y-hop
		{
			if(y_direc == -1)	// go north
			{   if (i == 0) { cout << " ERROR: routing problem " << endl; exit(1); }
				m1[j][i][c].N_out_link = m1[j][i][c].N_out_link + volume;		
				i--; if (i < 0) { cout << " ERROR " << i << endl; exit(1); }
			}
			else if (y_direc == 1)	// go south
			{   if (i == mesh_dim_y-1) { cout << " ERROR: routing problem " << endl; exit(1); }
				m1[j][i][c].S_out_link = m1[j][i][c].S_out_link + volume;		
				i++; if (i > mesh_dim_y-1) { cout << " ERROR " << i << endl; exit(1); }
			}
		}
		else if (path[k] == 1) // x-hop
		{
			if(x_direc == -1)	// go west
			{   if (j == 0) { cout << " ERROR: routing problem " << endl; exit(1); }
				m1[j][i][c].W_out_link = m1[j][i][c].W_out_link + volume;		
				j--; if (j < 0) { cout << " ERROR " << j << endl; exit(1); }
			}
			else if (x_direc == 1)	// go east
			{   if (j == mesh_dim_x-1) { cout << " ERROR: routing problem " << endl; exit(1); }
				m1[j][i][c].E_out_link = m1[j][i][c].E_out_link + volume;		
				j++; if (j > mesh_dim_x-1) { cout << " ERROR " << j << endl; exit(1); }
			}
		} // end else if 
		else if (path[k] == 2) // z-hop
		{
			if(z_direc == -1)	// go Up
			{   if (c == 0) { cout << " ERROR: routing problem " << endl; exit(1); }
				m1[j][i][c].Up_out_link = m1[j][i][c].Up_out_link + volume;		
				c--; if (c < 0) { cout << " ERROR " << c << endl; exit(1); }
			}
			else if (z_direc == 1)	// go Dn
			{   if (c == num_tiers-1) { cout << " ERROR: routing problem " << endl; exit(1); }
				m1[j][i][c].Dn_out_link = m1[j][i][c].Dn_out_link + volume;		
				c++; if (c > mesh_dim_y-1) { cout << " ERROR " << i << endl; exit(1); }
			}
		} // end else if
	}// end--for over the path

} // end -- build paths
