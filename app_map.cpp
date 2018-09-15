// Lets try this first: 
// map on slowest cores that meet the freq. constraints (for lowest leak-power and aging-fp) with the lowest Vdd.
// will need to run both thermal and PDN tools for each time-window .. so, thermal-profile (which affects aging) would consider the Vt-profile dependent leakage power..
// depending on the assumed cost-func., choose a block to map onto-note that PC-satisfaction will now be checked within the block_placement func., so map/route only valid mappings

#include "Constants.h"

void map_route (app_DoP_pair &app_element, const int mesh_dim_x, const int mesh_dim_y, const int num_tiers, grid_node*** m1);
bool block_placement (const int mesh_dimx, const int mesh_dim_y, const int num_tiers, grid_node*** m1, app_DoP_pair &app_element, double &power_slack, double &Vdd_glob); 
// find the best placement (with the required Vdd) for the 3D block for the given DoP -- with the given freq. constraint for the app -- 

// find rectangular region (for the given DoP)--map and route---
bool app_map (app_DoP_pair &app_element, const int mesh_dim_x, const int mesh_dim_y, const int num_tiers, grid_node*** m, double &power_slack, int epoch_counter) {  // returns 1 if app is successfully mapped..
//void app_map (vector <app_DoP_pair>& app_DoP_vec, const int mesh_dim, grid_node** m, double &power_slack, double &Vdd_glob, const int max_DoP) {
	//	cout << " Enter app_map -- initial power_slack is: " << power_slack << endl;
	int i, j, k; 
	bool Vdd_flag = 0; // -- newly added -- to flag the presence of app 1 or app 8, which need highest Vdd (Vdd=1.2) to satisfy SEC-constraint ---
			 
	GridPtr **m1 = new GridPtr*[mesh_dim_x];	// array of pointers
	for(i=0; i<mesh_dim_x; i++) {
		m1[i] = new GridPtr[mesh_dim_y];

		for (j=0; j<mesh_dim_y; j++)
			m1[i][j] = new grid_node [num_tiers];
	} // end for i
		
	bool flag;	
		
	// tranfer m to m1 
	for (i=0; i<mesh_dim_x; i++) { 
		for (j=0; j<mesh_dim_y; j++) {
			for (k=0; k<num_tiers; k++) {
				m1[i][j][k] = m[i][j][k];	
			}
		}
	}

	double vdd_glob = VDD_MIN;
	// pass m1 to sub_funcs and if it comes back empty, just use the old m to return back to main, else if mapping materializes, replace m with m1 and return m...
	flag = block_placement (mesh_dim_x, mesh_dim_y, num_tiers, m1, app_element, power_slack, vdd_glob);
	if (flag == 1)	{ cout << " block_placement successful, returns Vdd: "<< vdd_glob<< endl;
		map_route (app_element, mesh_dim_x, mesh_dim_y, num_tiers, m1, epoch_counter); 
		// tranfer m1 back to m -- as, mapping successful--
		for (i=0; i<mesh_dim_x; i++) { 
			for (j=0; j<mesh_dim_y; j++) {
				for (k=0; k<num_tiers; k++) {
					m[i][j][k] = m1[i][j][k];	 
				}
			}
		}
	}// end if

	// ******* will need to delete m1, return m ----
	for (i = 0; i < mesh_dim_x; ++i) {
		for (j = 0; j < mesh_dim_y; ++j) {
			delete[] m1[i][j];
		}
		delete[] m1[i];
	}
	delete[] m1;

	if (flag == 0)  return 0; 
	else return 1;

} // end app_map func.
