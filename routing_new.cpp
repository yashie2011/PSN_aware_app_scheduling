// For now, lets not consider comm_power (no use of ORION necessary) and assume the core power to be the tile-power (avg. power from SNIPER tool includes both router and core power) -- 
// -- We are not optimizing power or energy, power only determines temperature (which in turn determines aging) which is same for the entire tile -- 
//    -- router and core aging differentiated by their individual active times only --
// -- Also, block-placement is determined by considering just the Vt values of cores, while router Vt values are only used as a constraint -- so, as to optimize leakage in cores -- 
//     -- Aging-retardation (Vt profile) in routers only considered during routing -- 
// Thus, PC-satisfaction would not be required to be re-checked after routing -- 
//*****************************************************************************************************
// With 3D mesh, Up links are towards k=0 and Dn links are towards k=(num_tiers-1) ----
//*****************************************************************************************************
// ----- this is a hop-by-hop version of our former exhaustive aging-aware routing -------------------
// ---------------------------------------------------------------------------------------------------
#include "Constants.h"

void link_BW_paths (grid_node*** m1, const int mesh_dim_x, const int mesh_dim_y, const int num_tiers);
//void build_paths(node* node_pointers[], grid_node*** m1, vector <int> path, int source_ID, int dest_ID, double volume, const int mesh_dim_x, const int mesh_dim_y, const int num_tiers);
void build_paths(node* node_pointers[], grid_node*** m1, vector <int> path, int source_ID, int dest_ID, const int mesh_dim_x, const int mesh_dim_y, const int num_tiers, double &max_delay);
// ------- new funcs -----------------------------
void hop_cost (node* node_pointers[],grid_node*** m1, int hop, tile_coord hop_src, tile_coord hop_dest, double &aging_element, double &volume_element, bool & marker_element);
int choose_hop (vector <double> aging_vec, vector <double> volume_vec, double alpha, double beta); 
void update_a_b (app_DoP_pair app_element, double max_delay, double &alpha, double &beta, double compute_time);
void unbuild_paths(node* node_pointers[], grid_node*** m1, vector <int> path, int source_ID, int dest_ID, const int mesh_dim_x, const int mesh_dim_y, const int num_tiers, double &max_delay);
int choose_hop_icon(vector <bool> marker_vec, vector<double> volume_vec);

// -----------------------------------------------
void routing_new(app_DoP_pair &app_element, grid_node*** m1,
		node* node_pointers[], const int mesh_dim_x, const int mesh_dim_y,
		const int num_tiers, int routing_sel) {

	vector<double> psn_vec;
	vector<double> volume_vec;
	vector <bool> marked_vec;
	vector<int> hop_vec; // max_len is 3 -- contents: 0, 1, or 3.
	int i, j, k, count; // loop variables	
	ifstream instream;  ifstream instream1;  char next;
	link_BW_paths (m1, DIM_X, DIM_Y, DIM_Z);		// func. to compute the link BWs (not volumes) and store in the grid
	int path_length;  double max_delay = 0;
	int indx;
	double alpha = 0.4;
	double beta = 0.6;
	int hop;
	int hop_source_ID, hop_dest_ID;
	int x_direc, y_direc, z_direc, src_x, src_y, src_z; // ---- new ----
	double psn_element; double volume_element; bool marker_element; // --- new ------
	tile_coord hop_src, hop_dest;
	int source_ID, dest_ID;  vector <int> path; double volume;  int x_diff, y_diff, z_diff;
	for (i=0; i<app_element.DoP; i++) {
		for (j=0; j<node_pointers[i]->out_degree; j++) {
			volume = node_pointers[i]->out_v_volume[j];
			source_ID = node_pointers[i]->ID;	dest_ID = node_pointers[i]->out_v_connect[j];
			hop_source_ID = source_ID;   hop_dest_ID = -1; // --- new ---

			if (node_pointers[i]->paths.size() > j){
				path = node_pointers[i]->paths[j];
				unbuild_paths(node_pointers, m1, path, source_ID, dest_ID, mesh_dim_x, mesh_dim_y, num_tiers, max_delay);
			}
			x_diff = abs(node_pointers[source_ID-1]->x_coord - node_pointers[dest_ID-1]->x_coord);
			y_diff = abs(node_pointers[source_ID-1]->y_coord - node_pointers[dest_ID-1]->y_coord);
			z_diff = abs(node_pointers[source_ID-1]->z_coord - node_pointers[dest_ID-1]->z_coord);

			// ------- new -------
			if((node_pointers[source_ID-1]->x_coord - node_pointers[dest_ID-1]->x_coord) > 0) x_direc = -1;	// west
			else if ((node_pointers[source_ID-1]->x_coord - node_pointers[dest_ID-1]->x_coord) < 0) x_direc = 1;	// east
			else if ((node_pointers[source_ID-1]->x_coord - node_pointers[dest_ID-1]->x_coord) == 0) x_direc = 0;

			if ((node_pointers[source_ID-1]->y_coord - node_pointers[dest_ID-1]->y_coord) >0) y_direc = -1;	// north
			else if ((node_pointers[source_ID-1]->y_coord - node_pointers[dest_ID-1]->y_coord) < 0) y_direc = 1;	// south
			else if ((node_pointers[source_ID-1]->y_coord - node_pointers[dest_ID-1]->y_coord) == 0) y_direc = 0;

			if ((node_pointers[source_ID-1]->z_coord - node_pointers[dest_ID-1]->z_coord) > 0) z_direc = -1;	// up
			else if ((node_pointers[source_ID-1]->z_coord - node_pointers[dest_ID-1]->z_coord) < 0) z_direc = 1;	// down
			else if ((node_pointers[source_ID-1]->z_coord - node_pointers[dest_ID-1]->z_coord) == 0) z_direc = 0;
			// ---------------------
			
			src_x = node_pointers[hop_source_ID-1]->x_coord;
			src_y = node_pointers[hop_source_ID-1]->y_coord;
			src_z = node_pointers[hop_source_ID-1]->z_coord;

			path.clear();	
			path_length = x_diff + y_diff + z_diff;  k = 0;
			/*for (k=0; k<x_diff; k++) path.push_back(1);
			for (k=0; k<y_diff; k++)  path.push_back(0);
			for (k=0; k<z_diff; k++)  path.push_back(2);*/
			//cout <<"Routing new: coord diffs "<<x_diff<<" "<<y_diff<<" "<<z_diff<<endl;
			if (path.size() > 16) { cout << " ERROR: routing -- path too long for even a 4x4x2 box " << endl; exit(1); }  // -- verify that any path should be shorter than 7 hops ---
			
			for (k=0; k<path_length; k++) {
			    psn_vec.clear();  volume_vec.clear(); hop_vec.clear(); marked_vec.clear(); hop = -1;
			    for (count = 0; count < 2; count++) {
			    	// loop over hop 0, 1, and 2.. i.e. y-, x-, or z-hop resp.------
					if (((count == 0) && (y_diff > 0)) || ((count == 1) && (x_diff > 0))) {
						hop = count;	 
						if (count == 0) {
							//y_diff--;
							if (y_direc == -1) {
								//hop_dest_ID = m1[node_pointers[hop_source_ID-1]->x_coord][node_pointers[hop_source_ID-1]->y_coord-1][node_pointers[hop_source_ID-1]->z_coord].ID;
								hop_dest.x = src_x;
								hop_dest.y = src_y-1;
								hop_dest.z = src_z;
							}
							else if (y_direc == 1){
								//hop_dest_ID = m1[node_pointers[hop_source_ID-1]->x_coord][node_pointers[hop_source_ID-1]->y_coord+1][node_pointers[hop_source_ID-1]->z_coord].ID;
								hop_dest.x = src_x;
								hop_dest.y = src_y+1;
								hop_dest.z = src_z;
							}
							else { cout << " ERROR: y_direc cannot be zero " << endl; exit(1); }
						}
						else if (count == 1) {
							//x_diff--;
							if (x_direc == -1) {
								//hop_dest_ID = m1[node_pointers[hop_source_ID-1]->x_coord-1][node_pointers[hop_source_ID-1]->y_coord][node_pointers[hop_source_ID-1]->z_coord].ID;
								hop_dest.x = src_x -1;
								hop_dest.y = src_y;
								hop_dest.z = src_z;
							}
							else if (x_direc == 1) {
								//hop_dest_ID = m1[node_pointers[hop_source_ID-1]->x_coord+1][node_pointers[hop_source_ID-1]->y_coord][node_pointers[hop_source_ID-1]->z_coord].ID;
								hop_dest.x = src_x+1;
								hop_dest.y = src_y;
								hop_dest.z = src_z;
							}
							else { cout << " ERROR: x_direc cannot be zero " << endl; exit(1); } 
						}
						else if (count == 2) {
							//z_diff--;
							if (z_direc == -1) {
								//hop_dest_ID = m1[node_pointers[hop_source_ID-1]->x_coord][node_pointers[hop_source_ID-1]->y_coord][node_pointers[hop_source_ID-1]->z_coord-1].ID;
								hop_dest.x = src_x;
								hop_dest.y = src_y;
								hop_dest.z = src_z-1;
							}
							else if (z_direc == 1) {
								//hop_dest_ID = m1[node_pointers[hop_source_ID-1]->x_coord][node_pointers[hop_source_ID-1]->y_coord][node_pointers[hop_source_ID-1]->z_coord+1].ID;
								hop_dest.x = src_x;
								hop_dest.y = src_y;
								hop_dest.z = src_z+1;
							}
							else { cout << " ERROR: z_direc cannot be zero " << endl; exit(1); }
						}
					} 
					else continue;
					hop_src.x = src_x; hop_src.y = src_y; hop_src.z = src_z;
					if (hop < 0) { cout << " ERROR: hop not valid " << endl; exit(1); }
					hop_cost (node_pointers, m1, hop, hop_src, hop_dest, psn_element, volume_element, marker_element);
					psn_vec.push_back(psn_element);
					volume_vec.push_back(volume_element);
					marked_vec.push_back(marker_element);
					hop_vec.push_back(hop);
                } // end for count
				if ((hop_vec.size() != psn_vec.size()) || (hop_vec.size() > 2)
						|| (hop_vec.size() == 0)) {
					cout << " ERROR: hop_vec inconsistency " << endl;
					exit(1);
				}
				indx = -1;

				//cout<<"Routing type: "<<routing_sel<<endl;
				// Random XYZ algorithm
				if (routing_sel == 2)
					indx = choose_hop (psn_vec, volume_vec, alpha, beta); // returns the index of the best hop from the hop_vec
				else if (routing_sel == 1)
					indx = choose_hop_icon(marked_vec, volume_vec);
				else if (routing_sel == 0)
					indx = 0;
				if (indx < 0)
					indx= rand()% hop_vec.size();


				if (indx < 0) { cout << " ERROR: no hop chosen ?? "<<indx << endl; exit(1); }
				//else cout<<"indx "<<indx<<endl;
				path.push_back(hop_vec[indx]); // direction of next hop -- 0, 1, or 2...
				if ((hop_vec[indx] == 0) && (y_direc == -1)) {
					y_diff--;
					src_y = src_y-1;
				}
				if ((hop_vec[indx] == 0) && (y_direc == 1)) {
					y_diff--;
					src_y = src_y+1;
				}
				if ((hop_vec[indx] == 1) && (x_direc == -1)) {
					x_diff--;
					src_x = src_x-1;
				}
				if ((hop_vec[indx] == 1) && (x_direc == 1)) {
					x_diff--;
					src_x = src_x+1;;
				}
				if ((hop_vec[indx] == 2) && (z_direc == -1)) {
					z_diff--;
					src_z = src_z-1;
				}
				if ((hop_vec[indx] == 2) && (z_direc == 1)) {
					z_diff--;
					src_z = src_z+1;
				}
			} // end for k
			
			if (path.size() != path_length) { cout << "ERROR: path length is off ?? in routing func. " << endl; exit(1); }
			//update_a_b (app_element, max_delay, alpha, beta, compute_time); // -- check
			build_paths(node_pointers, m1, path, source_ID, dest_ID, mesh_dim_x, mesh_dim_y, num_tiers, max_delay);

			if (node_pointers[i]->paths.size() > j)
			node_pointers[i]->paths[j] = path;
			else if (node_pointers[i]->paths.size() == j){
				node_pointers[i]->paths.push_back(path);
			}
			else
			{
				cout<<"Routing new: ERROR, this should not be executed"<<endl;
				vector<vector<int>>::iterator it = node_pointers[i]->paths.begin();
				node_pointers[i]->paths.insert(it+j, path);
			}

		}
	} // end for i

} // end routing_xyz


// function to calculate the node pointer task comm bandwidths.
// should be called after all the apps are mapped and routed
void update_task_bw(app_DoP_pair &app_element, grid_node*** m1,
		node* node_pointers[], const int mesh_dim_x, const int mesh_dim_y,
		const int num_tiers){
	int source_ID, dest_ID;
	int x_diff, y_diff, z_diff, source_x, source_y, source_z, dest_x, dest_y, dest_z, x_direc, y_direc, z_direc;

	for (int i=0; i < app_element.DoP; i++){
		for (int j=0; j<node_pointers[i]->out_degree; j++){
			source_ID = node_pointers[i]->ID;	dest_ID = node_pointers[i]->out_v_connect[j];
			x_diff = abs(node_pointers[source_ID-1]->x_coord - node_pointers[dest_ID-1]->x_coord);
			y_diff = abs(node_pointers[source_ID-1]->y_coord - node_pointers[dest_ID-1]->y_coord);
			z_diff = abs(node_pointers[source_ID-1]->z_coord - node_pointers[dest_ID-1]->z_coord);

			// ------- new -------
			if((node_pointers[source_ID-1]->x_coord - node_pointers[dest_ID-1]->x_coord) > 0) x_direc = -1;	// west
			else if ((node_pointers[source_ID-1]->x_coord - node_pointers[dest_ID-1]->x_coord) < 0) x_direc = 1;	// east
			else if ((node_pointers[source_ID-1]->x_coord - node_pointers[dest_ID-1]->x_coord) == 0) x_direc = 0;

			if ((node_pointers[source_ID-1]->y_coord - node_pointers[dest_ID-1]->y_coord) >0) y_direc = -1;	// north
			else if ((node_pointers[source_ID-1]->y_coord - node_pointers[dest_ID-1]->y_coord) < 0) y_direc = 1;	// south
			else if ((node_pointers[source_ID-1]->y_coord - node_pointers[dest_ID-1]->y_coord) == 0) y_direc = 0;

			if ((node_pointers[source_ID-1]->z_coord - node_pointers[dest_ID-1]->z_coord) > 0) z_direc = -1;	// up
			else if ((node_pointers[source_ID-1]->z_coord - node_pointers[dest_ID-1]->z_coord) < 0) z_direc = 1;	// down
			else if ((node_pointers[source_ID-1]->z_coord - node_pointers[dest_ID-1]->z_coord) == 0) z_direc = 0;

			if ((x_direc == 0) && (y_direc == 0) && (z_direc == 0)) {
				cout << " All directions are 0 -- source_ID  dest_ID " << source_ID << " " << dest_ID << endl;
			    cout << " source " << source_x << " " << source_y << " dest " << dest_x << " " << dest_y << endl;  exit(1);
			}

			source_x = node_pointers[source_ID-1]->x_coord;
			source_y = node_pointers[source_ID-1]->y_coord;
			source_z = node_pointers[source_ID-1]->z_coord;
			dest_x = node_pointers[dest_ID-1]->x_coord;
			dest_y = node_pointers[dest_ID-1]->y_coord;
			dest_z = node_pointers[dest_ID-1]->z_coord;

			int p,q,r;
			p = source_x; q= source_y; r = source_z;
			int max_comms = 1;
			double min_bw = 9999;

			int path_length;   path_length = node_pointers[i]->paths[j].size();
			vector<int> path = node_pointers[i]->paths[j];
			//-- in addition to allocating volumes to path links, now we also need to calculate the path-delays (flow_times) according to the assumptions given at the top of this program
			for (int k=0; k<path_length; k++)
			{
				if (path[k] == 0) // y-hop
				{
					if(y_direc == -1)	// go north
					{   if (q == 0) { cout << " ERROR: routing problem " << endl; exit(1); }
						if (max_comms < m1[p][q][r].N_out_comms) max_comms = m1[p][q][r].N_out_comms; // m1[j][i][c].N_out_link + volume;
						if (min_bw > m1[p][q][r].N_out_BW) min_bw = m1[p][q][r].N_out_BW;
						q--;
						if (q < 0) {
							cout << " ERROR " << q << endl;
							exit(1);
						}
					}
					else if (y_direc == 1)	// go south
					{   if (q == mesh_dim_y-1) { cout << " ERROR: routing problem " << endl; exit(1); }
					if (max_comms < m1[p][q][r].S_out_comms) max_comms = m1[p][q][r].S_out_comms; //m1[j][i][c].S_out_link + volume;
					if (min_bw > m1[p][q][r].S_out_BW) min_bw = m1[p][q][r].S_out_BW;
						q++; if (q > mesh_dim_y-1) { cout << " ERROR " << q << endl; exit(1); }
					}
				}
				else if (path[k] == 1) // x-hop
				{
					if(x_direc == -1)	// go west
					{
						if (p == 0) { cout << " ERROR: routing problem " << endl; exit(1); }
						if (max_comms < m1[p][q][r].W_out_comms) max_comms = m1[p][q][r].W_out_comms; // m1[j][i][c].W_out_link + volume;
						if (min_bw > m1[p][q][r].W_out_BW) min_bw = m1[p][q][r].W_out_BW;
						p--; if (p < 0) { cout << " ERROR " << p << endl; exit(1); }
					}
					else if (x_direc == 1)	// go east
					{   if (p == mesh_dim_x-1) { cout << " ERROR: routing problem " << endl; exit(1); }
					if (max_comms < m1[p][q][r].E_out_comms) max_comms = m1[p][q][r].E_out_comms; //m1[j][i][c].E_out_link + volume;
					if (min_bw > m1[p][q][r].E_out_BW) min_bw = m1[p][q][r].E_out_BW;
						p++; if (p > mesh_dim_x-1) { cout << " ERROR " << p << endl; exit(1); }
					}
				} // end else if
				else if (path[k] == 2) // z-hop
				{
					cout<<"Why are we here we don't have z -axis at all"<<endl; exit(1);
					if(z_direc == -1)	// go Up
					{   if (r == 0) { cout << " ERROR: routing problem " << endl; exit(1); }
					if (max_comms < m1[p][q][r].U_out_comms) max_comms = m1[p][q][r].U_out_comms; // m1[j][i][c].Up_out_link + volume;
					if (min_bw > m1[p][q][r].Up_out_BW) min_bw = m1[p][q][r].Up_out_BW;
						r--; if (r < 0) { cout << " ERROR " << r << endl; exit(1); }
					}
					else if (z_direc == 1)	// go Dn
					{   if (r == num_tiers-1) { cout << " ERROR: routing problem " << endl; exit(1); }
					if (max_comms < m1[p][q][r].D_out_comms) max_comms = m1[p][q][r].D_out_comms; //m1[j][i][c].Dn_out_link + volume;
					if (min_bw > m1[p][q][r].Dn_out_BW) min_bw = m1[p][q][r].Dn_out_BW;
						r++; if (r > mesh_dim_y-1) { cout << " ERROR " << i << endl; exit(1); }
					}
				} // end else if
			}// end--for over the path
			if(node_pointers[i]->out_v_bw.size() > j)
				node_pointers[i]->out_v_bw[j] = double(min_bw / max_comms);
			else if (node_pointers[i]->out_v_bw.size() == j){
				node_pointers[i]->out_v_bw.push_back(double(min_bw / max_comms));
			}
			//cout<<"Router new: BW assigned to the path is "<<node_pointers[i]->out_v_bw[j] <<endl;
		}
	}
}


// function to update the node pointer volumes at the end of every time interval
// should be called at the end of every time interval
bool update_completed_vols(app_DoP_pair &app_element, grid_node*** m1,
		node* node_pointers[], const int mesh_dim_x, const int mesh_dim_y,
		const int num_tiers){
	bool ret_flag = true;
	for(int i=0; i < app_element.DoP; i++){
		for(int j=0; j< node_pointers[i]->out_degree; j++){
			if (node_pointers[i]->out_v_volume[j] > 0){
				node_pointers[i]->out_v_volume[j] = node_pointers[i]->out_v_volume[j] - (node_pointers[i]->out_v_bw[j] * TIME_STEP);
				ret_flag = ret_flag && false;
				//cout<<"Routing_new: Remaining volume b/n tasks: "<<node_pointers[i]->out_v_volume[j]<<endl;
			}
		}
	}
	return ret_flag;
}


// note that the inter-core communication is in terms of volume (Mb)
void link_BW_paths (grid_node*** m1, const int mesh_dim_x, const int mesh_dim_y, const int num_tiers) { // assign the max. BWs to all the links in the grid
	int i, j, k;
	for (i=0; i<mesh_dim_x; i++)
	{
		for (j=0; j<mesh_dim_y; j++)
		{
			for (k=0; k<num_tiers; k++)
			{
				if (i != 0) {
					m1[i][j][k].W_out_BW = 64.0 * (double) (m1[i][j][k].frequency);
				}
				if (i != mesh_dim_x-1) {
					m1[i][j][k].E_out_BW = 64.0 * (double) m1[i][j][k].frequency;
				}
				if (j != 0) {
					m1[i][j][k].N_out_BW = 64.0 * (double) m1[i][j][k].frequency;
				}
				if (j != mesh_dim_y - 1) {
					m1[i][j][k].S_out_BW = 64.0 * (double) m1[i][j][k].frequency;
				}
				if (k != 0) {
					m1[i][j][k].Up_out_BW = 32.0 * (double) m1[i][j][k].frequency;
				}
				if (k != num_tiers - 1) {
					m1[i][j][k].Dn_out_BW = 32.0 * (double) m1[i][j][k].frequency;
				}
			}
		}
	}
}//end link_BW


void build_paths(node* node_pointers[], grid_node*** m1, vector<int> path,
		int source_ID, int dest_ID, const int mesh_dim_x,
		const int mesh_dim_y, const int num_tiers, double &max_delay) {
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

	if ((x_direc == 0) && (y_direc == 0) && (z_direc == 0)) {
		cout << " All directions are 0 -- source_ID  dest_ID " << source_ID << " " << dest_ID << endl;
	    cout << " source " << source_x << " " << source_y << " dest " << dest_x << " " << dest_y << endl;  exit(1);
	}

	j = source_x;		i = source_y;    c = source_z;
	int path_length;   path_length = path.size();
	//-- in addition to allocating volumes to path links, now we also need to calculate the path-delays (flow_times) according to the assumptions given at the top of this program 
	for (k=0; k<path_length; k++)
	{
		if (path[k] == 0) // y-hop
		{
			if(y_direc == -1)	// go north
			{   if (i == 0) { cout << " ERROR: routing problem " << endl; exit(1); }
				m1[j][i][c].N_out_comms += 1; // m1[j][i][c].N_out_link + volume;
				m1[j][i][c].N_out_thru +=  node_pointers[source_ID-1]->thru;
				i--;
				if (i < 0) {
					cout << " ERROR " << i << endl;
					exit(1);
				}
			}
			else if (y_direc == 1)	// go south
			{   if (i == mesh_dim_y-1) { cout << " ERROR: routing problem " << endl; exit(1); }
				m1[j][i][c].S_out_comms += 1; //m1[j][i][c].S_out_link + volume;
				m1[j][i][c].S_out_thru +=  node_pointers[source_ID-1]->thru;
				i++; if (i > mesh_dim_y-1) { cout << " ERROR " << i << endl; exit(1); }
			}
		}
		else if (path[k] == 1) // x-hop
		{
			if(x_direc == -1)	// go west
			{   if (j == 0) { cout << " ERROR: routing problem " << endl; exit(1); }
				m1[j][i][c].W_out_comms += 1; // m1[j][i][c].W_out_link + volume;
				m1[j][i][c].W_out_thru +=  node_pointers[source_ID-1]->thru;
				j--; if (j < 0) { cout << " ERROR " << j << endl; exit(1); }
			}
			else if (x_direc == 1)	// go east
			{   if (j == mesh_dim_x-1) { cout << " ERROR: routing problem " << endl; exit(1); }
				m1[j][i][c].E_out_comms += 1; //m1[j][i][c].E_out_link + volume;
				m1[j][i][c].E_out_thru +=  node_pointers[source_ID-1]->thru;
				j++; if (j > mesh_dim_x-1) { cout << " ERROR " << j << endl; exit(1); }
			}
		} // end else if 
		else if (path[k] == 2) // z-hop
		{
			cout<<"Why are we here we don't have z -axis at all"<<endl; exit(1);
			if(z_direc == -1)	// go Up
			{   if (c == 0) { cout << " ERROR: routing problem " << endl; exit(1); }
				m1[j][i][c].U_out_comms += 1; // m1[j][i][c].Up_out_link + volume;
				m1[j][i][c].U_out_thru +=  node_pointers[source_ID-1]->thru;
				c--; if (c < 0) { cout << " ERROR " << c << endl; exit(1); }
			}
			else if (z_direc == 1)	// go Dn
			{   if (c == num_tiers-1) { cout << " ERROR: routing problem " << endl; exit(1); }
				m1[j][i][c].D_out_comms += 1; //m1[j][i][c].Dn_out_link + volume;
				m1[j][i][c].D_out_thru +=  node_pointers[source_ID-1]->thru;
				c++; if (c > mesh_dim_y-1) { cout << " ERROR " << i << endl; exit(1); }
			}
		} // end else if
	}// end--for over the path
} // end -- build paths


// This is used to clear grid of the old paths, so that when a new path is established,
// the new path will be built on the grid links
void unbuild_paths(node* node_pointers[], grid_node*** m1, vector <int> path,
		int source_ID, int dest_ID, const int mesh_dim_x,
		const int mesh_dim_y, const int num_tiers, double &max_delay){

		//cout<<"Unbuilding paths "<<endl;
	// if path vector is not empty go through each hop and delete the established comms on the grid links
		if (path.size() > 0){

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

		if ((x_direc == 0) && (y_direc == 0) && (z_direc == 0)) {
			cout << " All directions are 0 -- source_ID  dest_ID " << source_ID << " " << dest_ID << endl;
			cout << " source " << source_x << " " << source_y << " dest " << dest_x << " " << dest_y << endl;  exit(1);
		}

		j = source_x;		i = source_y;    c = source_z;
		int path_length;   path_length = path.size();
		//-- in addition to allocating volumes to path links, now we also need to calculate the path-delays (flow_times) according to the assumptions given at the top of this program
		for (k=0; k<path_length; k++)
		{
			if (path[k] == 0) // y-hop
			{
				if(y_direc == -1)	// go north
				{   if (i == 0) { cout << " ERROR: routing problem " << endl; exit(1); }
					m1[j][i][c].N_out_comms -= 1; // m1[j][i][c].N_out_link + volume;
					m1[j][i][c].N_out_thru -=  node_pointers[source_ID-1]->thru;
					if (m1[j][i][c].N_out_comms < 0) {
						cout << "Routing_new: grid comm unmapping error"<<endl;
						exit(1);
					}
					i--;
					if (i < 0) {
						cout << " ERROR " << i << endl;
						exit(1);
					}
				}
				else if (y_direc == 1)	// go south
				{   if (i == mesh_dim_y-1) { cout << " ERROR: routing problem " << endl; exit(1); }
					m1[j][i][c].S_out_comms -= 1; //m1[j][i][c].S_out_link + volume;
					m1[j][i][c].S_out_thru -=  node_pointers[source_ID-1]->thru;
					if (m1[j][i][c].S_out_comms < 0) {
						cout << "Routing_new: grid comm unmapping error"<<endl;
						exit(1);
					}
					i++; if (i > mesh_dim_y-1) { cout << " ERROR " << i << endl; exit(1); }
				}
			}
			else if (path[k] == 1) // x-hop
			{
				if(x_direc == -1)	// go west
				{   if (j == 0) { cout << " ERROR: routing problem " << endl; exit(1); }
					m1[j][i][c].W_out_comms -= 1; // m1[j][i][c].W_out_link + volume;
					m1[j][i][c].W_out_thru -=  node_pointers[source_ID-1]->thru;
					if (m1[j][i][c].W_out_comms < 0) {
						cout << "Routing_new: grid comm unmapping error"<<endl;
						exit(1);
					}
					j--; if (j < 0) { cout << " ERROR " << j << endl; exit(1); }
				}
				else if (x_direc == 1)	// go east
				{   if (j == mesh_dim_x-1) { cout << " ERROR: routing problem " << endl; exit(1); }
					m1[j][i][c].E_out_comms -= 1; //m1[j][i][c].E_out_link + volume;
					m1[j][i][c].E_out_thru -=  node_pointers[source_ID-1]->thru;
					if (m1[j][i][c].E_out_comms < 0) {
						cout << "Routing_new: grid comm unmapping error"<<endl;
						exit(1);
					}
					j++; if (j > mesh_dim_x-1) { cout << " ERROR " << j << endl; exit(1); }
				}
			} // end else if
			else if (path[k] == 2) // z-hop
			{
				cout<<"Why are we here we don't have z -axis at all"<<endl; exit(1);
				if(z_direc == -1)	// go Up
				{   if (c == 0) { cout << " ERROR: routing problem " << endl; exit(1); }
					m1[j][i][c].U_out_comms -= 1; // m1[j][i][c].Up_out_link + volume;
					m1[j][i][c].U_out_thru -=  node_pointers[source_ID-1]->thru;
					if (m1[j][i][c].U_out_comms < 0) {
						cout << "Routing_new: grid comm unmapping error"<<endl;
						exit(1);
					}
					c--; if (c < 0) { cout << " ERROR " << c << endl; exit(1); }
				}
				else if (z_direc == 1)	// go Dn
				{   if (c == num_tiers-1) { cout << " ERROR: routing problem " << endl; exit(1); }
					m1[j][i][c].D_out_comms -= 1; //m1[j][i][c].Dn_out_link + volume;
					m1[j][i][c].D_out_thru -=  node_pointers[source_ID-1]->thru;
					if (m1[j][i][c].D_out_comms < 0) {
						cout << "Routing_new: grid comm unmapping error"<<endl;
						exit(1);
					}
					c++; if (c > mesh_dim_y-1) { cout << " ERROR " << i << endl; exit(1); }
				}
			} // end else if
		}// end--for over the path
	}

}
// ------- new funcs -----------------------------
void hop_cost(node* node_pointers[], grid_node*** m1, int hop,
		tile_coord hop_src, tile_coord hop_dest, double &psn_element,
		double &volume_element, bool &marker_element)
{
	int i, j, c; //loop counters
	int source_x, source_y, source_z, dest_x, dest_y, dest_z, x_direc, y_direc, z_direc;
	if ((hop < 0) || (hop > 2)) { cout << " ERROR: hop invalid " << endl; exit(1); }
	source_z = hop_src.z;
	source_x = hop_src.x;
	source_y = hop_src.y;
	dest_z = hop_dest.z;
	dest_x = hop_dest.x;
	dest_y = hop_dest.y;
	if (((abs(source_x - dest_x) > 0) && (hop != 1))
			|| ((abs(source_y - dest_y) > 0) && (hop != 0))
			|| ((abs(source_z - dest_z) > 0) && (hop != 2))) {
		cout<<" hop: "<<hop<<endl;
		cout<<" x_diff: "<<abs(source_x - dest_x)<<endl;
		cout<<" y_diff: "<<abs(source_y - dest_y)<<endl;
		cout<<" z_diff: "<<abs(source_z - dest_z)<<endl;
		cout << " ERROR: hop not right " << endl;
		exit(1);
	} // end if --
	// ---------------------- new -----------------------
//	cout << " evaluating hop costs -------- " << endl;
//	cout << " source coords " << source_x << source_y << source_z << " and dest_coords " << dest_x << dest_y << dest_z << endl;
	// --------------------------------------------------
	
	if((source_x - dest_x) > 0) x_direc = -1;	// west
	else if ((source_x - dest_x) < 0) x_direc = 1;	// east
	else if ((source_x - dest_x) == 0) x_direc = 0;

	if ((source_y - dest_y) >0) y_direc = -1;	// north
	else if ((source_y - dest_y) < 0) y_direc = 1;	// south
	else if ((source_y - dest_y) == 0) y_direc = 0;

	if ((source_z - dest_z) >0) z_direc = -1;	// up
	else if ((source_z - dest_z) < 0) z_direc = 1;	// down
	else if ((source_z - dest_z) == 0) z_direc = 0;
	
	if ((x_direc == 0) && (y_direc == 0) && (z_direc == 0)) {
		cout << " All directions are 0 " << endl;
		cout << " coords " << source_x << " "
				<< source_y << " " << source_z << " "<< dest_x << " " << dest_y << " " << dest_z << endl;
	    exit(1);
	}
	
	j = source_x;	 i = source_y;   c = source_z;   
	 
		if (hop == 0) // y-hop
		{ 
			if(y_direc == -1)	// go north
			{  
				i--;
				volume_element = m1[j][i][c].N_out_thru + m1[j][i][c].S_out_thru + m1[j][i][c].E_out_thru + m1[j][i][c].W_out_thru;
				psn_element = (m1[j][i][c].Vt_core - m1[j][i][c].current_Vdd);
				marker_element = m1[j][i][c].r_mark;
			}
			else if (y_direc == 1)	// go south
			{  
				i++;
				volume_element = m1[j][i][c].N_out_thru + m1[j][i][c].S_out_thru + m1[j][i][c].E_out_thru + m1[j][i][c].W_out_thru;
				psn_element = (m1[j][i][c].Vt_core - m1[j][i][c].current_Vdd);
				marker_element = m1[j][i][c].r_mark;
			}
		}
		else if (hop == 1) // x-hop
		{ 
			if (x_direc == -1)	// go west
			{ 
				j--;
				volume_element = m1[j][i][c].N_out_thru + m1[j][i][c].S_out_thru + m1[j][i][c].E_out_thru + m1[j][i][c].W_out_thru;
				psn_element = (m1[j][i][c].Vt_core - m1[j][i][c].current_Vdd);
				marker_element = m1[j][i][c].r_mark;
			}
			else if (x_direc == 1)	// go east
			{ 
				j++;
				volume_element = m1[j][i][c].N_out_thru + m1[j][i][c].S_out_thru + m1[j][i][c].E_out_thru + m1[j][i][c].W_out_thru;
				psn_element = (m1[j][i][c].Vt_core - m1[j][i][c].current_Vdd);
				marker_element = m1[j][i][c].r_mark;
			} 
		}
		else if (hop == 2) // z-hop
		{ 
			if(z_direc == -1)	// go up
			{ 
				c--;
				volume_element = m1[j][i][c].N_out_thru + m1[j][i][c].S_out_thru + m1[j][i][c].E_out_thru + m1[j][i][c].W_out_thru;
				psn_element = (m1[j][i][c].Vt_core - m1[j][i][c].current_Vdd);
				marker_element = m1[j][i][c].r_mark;
			}
			else if (z_direc == 1)	// go down
			{ 
				c++;
				volume_element = m1[j][i][c].N_out_thru + m1[j][i][c].S_out_thru + m1[j][i][c].E_out_thru + m1[j][i][c].W_out_thru;
				psn_element = (m1[j][i][c].Vt_core - m1[j][i][c].current_Vdd);
				marker_element = m1[j][i][c].r_mark;
			} 
		}// end if - path[k]
	
// ------------------------- new ----------------------------
//	cout << " aging sum for this path " << aging_element << endl;
// -----------------------------------------------------------	
}// end hop_cost func

int choose_hop (vector <double> psn_vec, vector <double> volume_vec, double alpha, double beta)
{
	int i; double min, max, range, temp;
	vector <double> norm_aging_vec;  vector <double> norm_volume_vec;   vector <double> norm_cost_vec; // normalization vecs ---
	// ------------------ new -----------------------------
	//cout << " ---- alpha and beta are: " << alpha << " " << beta << endl;
	//cout << " choose hop from these vecs ------ " << endl;
	//cout << " aging vec is: ";  for (i=0; i<aging_vec.size(); i++) { cout << aging_vec[i] << " " ; } cout << endl;
	//cout << " volume vec is: ";  for (i=0; i<aging_vec.size(); i++) { cout << volume_vec[i] << " " ; } cout << endl;
	// -----------------------------------------------
	if (psn_vec.size() != volume_vec.size()) { cout << " ERROR: choose_hop func. in routing " << endl; exit(1); }
	//cout << " aging_vec.size() is: " << aging_vec.size() << endl;
	if (psn_vec.size() < 1) { cout << " we should not be here " << endl; exit(1); }
	if (psn_vec.size() < 2) { return 0; }
	int indx;
	/*
	// (1) psn normalizatuion --
	min = 10000.0;  max = 0; 
	for (i=0; i<psn_vec.size(); i++) {
		if (min > psn_vec[i]) min = psn_vec[i];
		if (max < psn_vec[i]) max = psn_vec[i];
	}// end for i
	range = max - min;

	for (i=0; i<psn_vec.size(); i++) {
		temp = (psn_vec[i] - min)/range;   if (range < 0.000001) {  temp = 0; }
		if ((temp < 0) ||(temp > 1.0)) { cout << " ERROR: normalized_cost not between 0 and 1 -- routing " << endl; exit(1); }
		norm_aging_vec.push_back(temp);
	}

	// (2) volume normalization --
	min = 100000000.0;  max = 0;
	for (i=0; i<volume_vec.size(); i++) {
		if (min > volume_vec[i]) min = volume_vec[i];
		if (max < volume_vec[i]) max = volume_vec[i];
	} // end for i
	range = max - min;

	for (i=0; i<volume_vec.size(); i++) {
		temp = (volume_vec[i] - min)/range;   if (range < 0.00001) {  temp = 0; }
		if ((temp < 0) ||(temp > 1.0)) { cout << " ERROR: normalized_cost not between 0 and 1 -- routing " << endl; exit(1); }
		norm_volume_vec.push_back(temp);
	} // end for i
	if (norm_aging_vec.size() != norm_volume_vec.size()) { cout << " ERROR: choose_path func. in routing " << endl; exit(1); }
	
	min = 1000.0;
	for (i=0; i<norm_volume_vec.size(); i++) {
		temp = alpha * norm_aging_vec[i] + beta * norm_volume_vec[i];
		if (temp > 1.0) {
			cout << " ERROR: can combined_cost be over one ? -- routing "
					<< endl;
			exit(1);
		}
		norm_cost_vec.push_back(temp);
		if (temp < min) {
			indx = i;
			min = temp;
		}
	} // end for i

	if (norm_cost_vec.size() != norm_volume_vec.size()) { cout << " ERROR: chosse_hop func. in routing " << endl; exit(1); }
	*/
	// --------------new --------------------------
/*	cout << " norm_aging_vec " << endl;
	for (i=0; i<norm_aging_vec.size(); i++) {
		cout << norm_aging_vec[i] << " ";
	} cout << endl;
	cout << " norm_volume_vec " << endl;
	for (i=0; i<norm_volume_vec.size(); i++) {
		cout << norm_volume_vec[i] << " ";
	} cout << endl;
	cout << " norm_cost_vec " << endl;
	for (i=0; i<norm_volume_vec.size(); i++) {
		cout << norm_cost_vec[i] << " ";
	} cout << endl;
	cout << " indx is: " << indx << endl; */
	// --------------------------------------------	
	min = 9999999.999;
	//cout<< "Printing Volume Vector: ";
	for (i=0; i<volume_vec.size(); i++){
		if (volume_vec[i] < min){
			min = volume_vec[i];
			//cout<<volume_vec[i];
			indx = i;
		}
	}
	//cout<<endl;
	return indx;
} // end choose hop func.

void mark_router(grid_node*** m1, bool x_even, bool y_even){
	for(int j=0; j < DIM_Y; j++){
		for(int i=0; i < DIM_X; i++){
			int k = 0;
			if (x_even && (i%2 == 0)){
				if (y_even && (j%2 == 0)){
					m1[i][j][k].r_mark = true;
				}
				else if(!y_even && j%2 ==1){
					m1[i][j][k].r_mark = true;
				}
			}
			else if (!x_even && (i%2 == 1)){
				if (y_even && (j%2 == 0)){
					m1[i][j][k].r_mark = true;
				}
				else if(!y_even && j%2 ==1){
					m1[i][j][k].r_mark = true;
				}
			}
			else{
				m1[i][j][k].r_mark = false;
			}

		}
	}
}

//choosing hop for iconoclast
int choose_hop_icon(vector <bool> marker_vec, vector<double> volume_vec){
	//cout<<"Choose_hop_icon: hop_vec size: "<<marker_vec.size() <<endl;
	if (marker_vec.size() < 1){cout<<"marker vector is not filled"<<endl;}
	else if (marker_vec.size() < 2){return 0;}
	for (int i=0; i < marker_vec.size(); i++){
		if (marker_vec[i])
			return i;
		else if (! marker_vec[i] && (volume_vec[i] < 3)){
			return i;
		}
	}

	return -1;
}

void update_a_b (app_DoP_pair app_element, double max_delay, double &alpha, double &beta, double compute_time)  //////// ------------ STILL to CHECK -------------
{
//  ----------- from build_paths, u find the max-router-delay after the latest routed flow, thus, we always keep track of the max_delay from the already routed flows
// we use a combined cost-func. {a.aging + b.congestion}.
// Where b starts with zero and a starts with a value of 1. As, the max.-delay due to congestion approaches say 90% of the deadline, a approaches zero and b approaches 1. 
// --- Eqns. For a and b: b={(current max. delay)/(90% of deadline)}  And, a = 1 - b ---
//************************************************************************************************
	beta = (max_delay-compute_time)/(0.6*(app_element.run_time_constraint-compute_time)); if (beta > 1.0) beta = 1.0;
	alpha = 1.0 - beta;
	if ((alpha > 1.001) || (beta > 1.001)) {
		cout << " alpha and beta should be under one -- routing " << alpha << " " << beta << endl;
		cout << " beta is: " << beta << " max_delay is " << max_delay
				<< " compute time " << compute_time << " run_time_constraint "
				<< app_element.run_time_constraint << endl;
	    exit(1);
	}
} // end update func.

