// -- here, just do mapping and routing -- calculate comm-times (app-run-times), comm-power/energy here ---
// -- PC-satisfaction, SEC-calculations, and finish-times can be calculated back in main or app_map ?? --

#include "Constants.h"

// For now, lets not consider comm_power (no use of ORION necessary) and assume the core power to be the tile-power (avg. power from SNIPER tool includes both router and core power) -- 
// -- We are not optimizing power or energy, power only determines temperature (which in turn determines aging) which is same for the entire tile -- 
//    -- router and core aging differentiated by their individual active times only --
// -- Also, block-placement is determined by considering just the Vt values of cores, while router Vt values are only used as a constraint -- so, as to optimize leakage in cores -- 
//     -- Aging-retardation (Vt profile) in routers only considered during routing -- 
// Thus, PC-satisfaction would not be required to be re-checked after routing -- 

bool map_route(app_DoP_pair &app_element, const int mesh_dim_x,
		const int mesh_dim_y, const int num_tiers, grid_node*** m1) {
		int j;
		double BM_bias[13] = { 5.0, 14.5, 5.0, 15.0, 5.0, 3.0, 16.0, 2.0, 4.4, 8.0, 2.0, 2.0, 1.2 };
	// -- first, check if DoP vec is trivial -- i.e., DoP greater than one -- if not, return from this func. --
	if (app_element.DoP > 1) {
		cout << "map_route: DoP of the app " << app_element.DoP << endl;
		if (!((app_element.DoP == 4) || (app_element.DoP == 8)
				|| (app_element.DoP == 16) || (app_element.DoP == 32))) {
			cout << " ERROR: map_route -- DoP not right " << endl;
			exit(1);
		}
	}
		else if (app_element.DoP == 1) { cout << " ERROR : app_map -- DoP of 1 is disallowed " << endl; exit(1); } 
		else { cout << " ERROR: map_route -- DoP not right " << endl; exit(1);  }
	// -----------------------------------------------------------------------------------------------------------------------------
		char next;  ifstream instream;	int temp, n;	double temp_double;
		node* node_pointers[MAX_THREAD_COUNT];
		//array of pointers to all the nodes-- upto 32 cores
		// node_pointers[0] = new node(0);	// -- No more dummy --
		int DoP;	char buffer [50];
				buffer[0] = 0;
				sprintf (buffer, "BM_%d_%d.txt", app_element.app_ID, app_element.DoP);
				cout<<"map_route: communication_profile input : "<<buffer<<endl;
				instream.open(buffer);
				instream >> DoP;
				if (app_element.DoP != DoP) { cout << " ERROR: map_route -- DoP mismatch " << endl; exit(1); }
				//instream >> temp1 >> temp1 >> temp1 >> temp1; // skip these fields from the file --
				instream.get(next);  // read in '/n'
		for (j = 0; j < app_element.DoP; j++) { // -- tot_vol should include in_volume as well --- correct this ?? -- may be not ---
			node_pointers[j] = new node(j);
			instream >> node_pointers[j]->ID;
			if (node_pointers[j]->ID != j + 1) {
				cout << " ERROR: map-route - I ID: " << j << " "
						<< node_pointers[j]->ID << " " << endl;
				exit(1);
			}
			instream >> node_pointers[j]->out_degree;
			n = 0;
			node_pointers[j]->total_vol = 0;
			node_pointers[j]->marked = 0;
			node_pointers[j]->x_coord = -1;
			node_pointers[j]->y_coord = -1;
			node_pointers[j]->z_coord = -1; // to be used later
			while (n < node_pointers[j]->out_degree) {
				instream >> temp;
				node_pointers[j]->out_v_connect.push_back(temp);
				instream >> temp_double;

				// Bias should vary based on the DoP size
				if (app_element.DoP > 8){
					temp_double = (BM_bias[app_element.app_ID - 1]) * temp_double/10 ; // multiplied by less bias
				}
				else{
					temp_double = (BM_bias[app_element.app_ID - 1]) * temp_double ; // multiplied by bias
				}
				node_pointers[j]->out_v_volume.push_back(temp_double);
				node_pointers[j]->total_vol = node_pointers[j]->total_vol
						+ node_pointers[j]->out_v_volume[n]; // update the total volume
				n++;
			} // end while n
			instream.get(next); // read the newline char.
			app_element.total_vol += node_pointers[j]->total_vol;
			node_pointers[j]->thru = node_pointers[j]->total_vol /app_element.run_time_constraint;
		} // end for j
				instream.close();
				cout<<"map_route.cpp: Read the comm profile success "<<endl;
				if (j != app_element.DoP) { cout << " ERROR : map-route -- num. of nodes in node_pointers ?? " << endl; exit(1); }
			

				vector <int> indices_vec;
		// Fill node_pointer coords and grid_id

		if (app_element.map_region.coords.size() != app_element.DoP){
			cout<<"Error mapping region size and DoP are not matching"<<endl;
			exit(1);
		}
		else{
			for (size_t i =0; i < app_element.map_region.coords.size(); i++){
				node_pointers[i]->x_coord = app_element.map_region.coords[i].x;
				node_pointers[i]->y_coord = app_element.map_region.coords[i].y;
				node_pointers[i]->z_coord = app_element.map_region.coords[i].z;
				m1[node_pointers[i]->x_coord][node_pointers[i]->y_coord][node_pointers[i]->z_coord].ID = i+1;
				cout<<"mapping coordinates of the task "<<endl;
				cout<<app_element.map_region.coords[i].x<<" "<<app_element.map_region.coords[i].y<<endl;
			}
		}
		for (int i= 0; i < MAX_THREAD_COUNT; i++)
		app_element.tg_nodes[i] = node_pointers[i];

		return true;
		// Call the routing function
		//routing_new (app_element, m1, node_pointers, mesh_dim_x, mesh_dim_y, num_tiers);
		//routing_xyz (app_element, m1, node_pointers, mesh_dim_x, mesh_dim_y, num_tiers);
		//cout << " comm energy and power are: " << comm_energy << " " << comm_power << endl;

	/*	for (i=0; i<mesh_dim_x; i++) {  // check if any cores remain unmapped -- ??
		  for (j=0; j<mesh_dim_y; j++) {
			  for (k=0; k<num_tiers; k++) {
			 // if ((m1[i][j].ID == 0) && (m1[i][j].compute_time > 0.001)) { cout << " ERROR: final in map_route " << endl; exit(1); }
			  if ((m1[i][j][k].app_seq_num == app_element.app_seq_num) && (m1[i][j][k].ID == 0) && (m1[i][j][k].compute_time < 0.001)) {
				  cout << " ERROR : map_route - II " << endl; exit(1); }
				}
			}
		} // end for i */

	//	for (i=0; i<app_element.DoP; i++)
		//	delete node_pointers[i];
				
} // end map_route 
