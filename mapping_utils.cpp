#include "Constants.h"

extern vector<string> apps;
extern vector<short> DoPs;
extern vector<float> Vdds;

double get_fmax(float vdd);
double get_run_time (app_pow_traces apt, float vdd, short dop, float fmax);
vector <float> get_power_trace(power_trace pt, short dop, double cycle_number, float frequency);
void unbuild_paths(node* node_pointers[], grid_node*** m1, vector <int> path,
		int source_ID, int dest_ID, const int mesh_dim_x,
		const int mesh_dim_y, const int num_tiers, double &max_delay);


// Called to map a new app {default, random mapping}
// Returns true if mapping is successful, Else false.
bool map_app (app_DoP_pair &app_element, const int mesh_dim_x,
		const int mesh_dim_y, const int num_tiers, grid_node*** m, double &power_slack, int &avail_tiles, double op_time){
	// Start with the highest DoP,
	// Else, decrease the DoP, map the app, if the lowest DoP cannot be mapped, leave.
	// if mapping found, fill the App_element with the values of Vdd, tiles IDs, and initial power values
	vector <tile_coord> mapping;
	short selected_DoP = 0;
	for (size_t index = 0; index < DoPs.size(); index++){
		// if there are enought tiles and if power slack is sufficient, map it.
		mapping.clear();
		if (avail_tiles >= DoPs[index]){
			selected_DoP = DoPs[index];
			int selected_tiles = 0;
			bool break_flag = false;
			for (int i=0; i< DIM_X; i++){
				for (int j=0; j<DIM_Y; j++){
					for (int k=0; k<DIM_Z; k++){
						if(m[i][j][k].empty_flag){
							tile_coord coord(i,j,k, false);
							mapping.push_back(coord);
							selected_tiles++;
						}
						if (selected_tiles >= selected_DoP){
							break_flag = true;
						}
						if (break_flag) break;
					}
					if (break_flag) break;
				}
				if (break_flag) break;
			}
		}
		else{
			continue;
		}
		// Space on chip found, we should check for power slack. But that can only be found after computing the power consumption.
		cout<<"Mapping selected DoP: "<<selected_DoP<<endl;
		// Decide Vdd, and power trace. how do we choose?
		// Start from the lowest Vdd, if Fmax can get us through deadline stick with it. Else move to the next Vdd
		for (size_t v=0; v<Vdds.size(); v++){
			double Fmax = get_fmax(Vdds[v]);
			double run_time = get_run_time(app_element.app_pt, Vdds[v], selected_DoP, Fmax);
			if (run_time == 0) // This DoP/Vdd trace is unavailable, so we have to chose another one
				continue;
			if(op_time+run_time > app_element.deadline){
				continue;
			}
			else{
				avail_tiles = avail_tiles - selected_DoP;
				app_element.DoP = selected_DoP;
				app_element.map_time_vdd = Vdds[v];
				app_element.compute_time = run_time;
				app_element.frequency = Fmax;
				region mapped_region;
				mapped_region.DoP = selected_DoP;
				mapped_region.Vdd = Vdds[v];
				mapped_region.coords = mapping;
				mapped_region.frequency = Fmax;
				//TODO: We dont know if this works get the first power trace from the map

				mapped_region.tile_P = get_power_trace(app_element.app_pt[selected_DoP][Vdds[v]], selected_DoP, 0, Fmax);
				if(mapped_region.tile_P.empty()){
					continue;
				}
				app_element.map_region = mapped_region;
				app_element.start_time = op_time;
				app_element.lastRecCycle = 0;
				cout<<"map_app: Mapping of app successful: "<< apps[app_element.app_ID]<<endl;

				// Fill in the power values and the reset the empty flag of the grid nodes
				for (size_t i=0; i< app_element.map_region.coords.size(); i++){
					m[app_element.map_region.coords[i].x][app_element.map_region.coords[i].y][app_element.map_region.coords[i].z].empty_flag = false;
					m[app_element.map_region.coords[i].x][app_element.map_region.coords[i].y][app_element.map_region.coords[i].z].power = app_element.map_region.tile_P[i];
				}
				return true;
			}
		}
	}
	// if mapping not found, return false
	cout<<"map_app: Mapping is not found!!!"<<endl;
	return false;
};

// Proposed region based mapping scheme
bool map_app_psn_aware(app_DoP_pair &app_element, const int mesh_dim_x,
		const int mesh_dim_y, const int num_tiers, grid_node*** m, double &power_slack, int &avail_tiles, double op_time){
	vector <tile_coord> mapping;
	double Fmax = 0;
	double run_time =0;
	short map_x; short map_y;

	// Iterate over the DoP range
	for (size_t d =0; d < DoPs.size(); d++){
		mapping.clear();
		if (avail_tiles < DoPs[d]){
			cout<<"map_app_psn: DoP "<<DoPs[d]<<" can't be mapped, reducing the dop size"<<endl;
			continue;
		}
		else{
			float sel_vdd =0;
			vector <float> task_pows;

			// Check for each vdd in increasing order
			for (size_t v=0; v<Vdds.size(); v++){
				Fmax = get_fmax(Vdds[v]);
				run_time = get_run_time(app_element.app_pt, Vdds[v], DoPs[d], Fmax);
				if (run_time == 0) // This DoP/Vdd trace is unavailable, so we have to check the next vdd
					{
						cout<<"map_app_psn: get_run_time returned 0"<<endl;
						continue;
					}
				else if(op_time+run_time > app_element.deadline){
					cout<<"map_app_psn: Deadline missed, move on to next Vdd"<<endl;
					continue;
				}
				else{
					sel_vdd = Vdds[v];
					cout<<"map_app_psn: deadline met, find mapping region"<<endl;
					break;
				}
			}
			if (sel_vdd == 0 || Fmax == 0){
				cout<<"map_app_psn: Fmax is 0"<<endl;
				continue; // Move to next DoP, as no Vdd can satisfy deadline at this DoP
			}
			else{
				int k = 0; bool break_flag = false;
				int count = 0;

				task_pows = get_power_trace(app_element.app_pt[DoPs[d]][Vdds[sel_vdd]], DoPs[d], 0, Fmax);
				if (DoPs[d] == 4 || DoPs[d] == 8){map_x = 2; map_y = 2;}
				else if (DoPs[d] == 16 ){map_x = 2; map_y = 2;}
				else if (DoPs[d] == 32){map_x = 4; map_y = 4;}
				vector <tile_coord> temp_mapping;
				count = 0;
				for (size_t i = 0; i< DIM_X; i++){
					for (size_t j = 0; j < DIM_Y; j++){
						break_flag = false;
						mapping.clear();
						if (count == DoPs[d]) break;
						int temp_count = 0;
						if(m[i][j][k].empty_flag){
							cout<<"finding space at: "<<i<<" "<<j<<" for "<<map_x<<" "<<map_y<<endl;
							if((i + map_x <= DIM_X)&&(j + map_y <= DIM_Y)){
								for (int s = 0; s < map_x; s++){
									for (int t = 0; t < map_y; t++){
										if (m[i+s][j+t][k].empty_flag){
											cout<<"space found checking tiles "<< i+s<<" "<<j+t<<" "<<m[i+s][j+t][k].empty_flag<<endl;
											mapping.push_back(tile_coord(i+s, j+t, k , false));
											temp_count ++;
										}
										else {break_flag = true; temp_count = 0; mapping.clear(); break;}
									}if (break_flag) break;
								}
							}
						}
						// push the temporary mapping and set empty flags of those tiles to false
						if(!break_flag && mapping.size() > 0){
							temp_mapping.insert(temp_mapping.end(), mapping.begin(), mapping.end());
							count += temp_count;
							for (int i=0; i< mapping.size(); i++){
								tile_coord coord = mapping[i];
								m[coord.x][coord.y][coord.z].empty_flag = false;
							}
						}
					}
					if (count == DoPs[d]) break;
				} // End for
				// if No mapping found, go to lower DoP
				if (count != DoPs[d]){
					cout<<"map_app_psn: selected tiles "<<count<<" is not sufficient"<<endl;
					continue;
				}
				// Found a mapping. Now, save the stats into app_element.
				avail_tiles = avail_tiles - DoPs[d];
				app_element.DoP = DoPs[d];
				app_element.map_time_vdd = sel_vdd;
				app_element.compute_time = run_time;
				app_element.frequency = Fmax;
				region mapped_region;
				mapped_region.DoP = DoPs[d];
				mapped_region.Vdd = sel_vdd;
				mapped_region.coords = temp_mapping;
				mapped_region.frequency = Fmax;
				//TODO: We dont know if this works get the first power trace from the map

				mapped_region.tile_P = task_pows;
				if(mapped_region.tile_P.empty()){
					cout<<"map_app_psn: Error: Tile powers returned by the mapping is empty"<<endl;
					exit(1);
				}
				app_element.map_region = mapped_region;
				app_element.start_time = op_time;
				app_element.lastRecCycle = 0;
				cout<<"map_app: Mapping of app successful: "<< apps[app_element.app_ID]<<endl;

				// Fill in the power values and the reset the empty flag of the grid nodes
				for (size_t i=0; i< app_element.map_region.coords.size(); i++){
					m[app_element.map_region.coords[i].x][app_element.map_region.coords[i].y][app_element.map_region.coords[i].z].empty_flag = false;
					m[app_element.map_region.coords[i].x][app_element.map_region.coords[i].y][app_element.map_region.coords[i].z].power = app_element.map_region.tile_P[i];
				}
				return true;

			}
		}
	}
	cout<<"map_app_psn: No mapping found for any DoP or Vdd combo."<<endl;
	return false;
}


// Comparison work, PSN aware mapping with repulsion force
bool map_app_compare_1(app_DoP_pair &app_element, const int mesh_dim_x,
		const int mesh_dim_y, const int num_tiers, grid_node*** m, double &power_slack, int &avail_tiles, double op_time){
	short dop = 8;
	float vdd = 0.8;
	if (avail_tiles < dop){
		cout<<"map_app_1: Not enough space on chip"<<endl;
		return false;
	}
	double Fmax = get_fmax(vdd);
	double run_time = get_run_time(app_element.app_pt, vdd, dop, Fmax);
	if (run_time == 0) // This DoP/Vdd trace is unavailable
	{
		cout<<"map_app_1: No trace for this dop, vdd"<<endl;
		return false;
	}
	if(op_time+run_time > app_element.deadline){
		cout<<"map_app_1: the given dop, vdd cannot meet deadline"<<endl;
		cout<<"map_app_1: run_time ends at "<<(op_time+run_time)<<" deadline: "<<app_element.deadline<<endl;
		return false;
	}
	vector <tile_coord> mapping;
	vector <float> task_pows = get_power_trace(app_element.app_pt[dop][vdd], dop, 0, Fmax);
	short count = 0;
	float act_fac = 0;
	while (count < dop){
		tile_coord n1, n2, n3, n4;
		float min_act_fac = 999999;
		tile_coord temp;
		for (size_t i = 0; i < DIM_X; i++){
			for (size_t j = 0; j < DIM_Y; j++){
				for (size_t k = 0; k < DIM_Z; k++){
					act_fac = 0;
					if (m[i][j][k].empty_flag){

						//Choose the neighboring tiles
						if (i > 0 && j > 0){
							n1.x = i-1; n1.y = j; n1.marked = true;
							n2.x = i; n2.y = j-1; n2.marked = true;
							if (i < DIM_X-1){
								n3.x = i+1; n3.y = j; n3.marked = true;
							}
							if (j < DIM_Y-1){
								n4.x = i; n4.y = j+1; n4.marked = true;
							}
						}
						else if (i <= 0 && j > 0){
							n2.x = i; n2.y = j-1; n2.marked = true;
							if (i < DIM_X-1){
								n3.x = i+1; n3.y = j; n3.marked = true;
							}
							if (j < DIM_Y-1){
								n4.x = i; n4.y = j+1; n4.marked = true;
							}
						}
						else if (i >0 && j <=0){
							n1.x = i-1; n1.y = j; n1.marked = true;
							if (i < DIM_X-1){
								n3.x = i+1; n3.y = j; n3.marked = true;
							}
							if (j < DIM_Y-1){
								n4.x = i; n4.y = j+1; n4.marked = true;
							}
						}
						else if (i <=0 && j <=0){
							if (i < DIM_X-1){
								n3.x = i+1; n3.y = j; n3.marked = true;
							}
							if (j < DIM_Y-1){
								n4.x = i; n4.y = j+1; n4.marked = true;
							}
						}

						// calculate the activity factor of the task w.r.to neighbors
						if (n1.marked)
						act_fac += m[n1.x][n1.y][n1.z].power * task_pows[count];
						if (n2.marked)
						act_fac += m[n2.x][n2.y][n2.z].power * task_pows[count];
						if (n3.marked)
						act_fac += m[n3.x][n3.y][n3.z].power * task_pows[count];
						if (n4.marked)
						act_fac += m[n4.x][n4.y][n4.z].power * task_pows[count];
						//cout<<"act_fact "<<act_fac<<endl;
						if (act_fac < min_act_fac){
							min_act_fac = act_fac;
							temp.x = i; temp.y = j; temp.z = k;
						}
					}
				}
			}
		}
		m[temp.x][temp.y][temp.z].power = task_pows[count];
		m[temp.x][temp.y][temp.z].empty_flag = false;
		mapping.push_back(temp);
		count++;
	}
	avail_tiles = avail_tiles-dop;
	app_element.DoP = dop;
	app_element.map_time_vdd = vdd;
	app_element.compute_time = run_time;
	app_element.frequency = Fmax;
	region mapped_region;
	mapped_region.DoP = dop;
	mapped_region.Vdd = vdd;
	mapped_region.coords = mapping;
	mapped_region.frequency = Fmax;
	mapped_region.tile_P = task_pows;
	if(mapped_region.tile_P.empty()){
		cout<<"mapp_app_1: Error: Tile power should not be empty"<<endl;
		exit(1);
	}
	app_element.map_region = mapped_region;
	app_element.start_time = op_time;
	app_element.lastRecCycle = 0;
	cout<<"map_app: Mapping of app successful: "<< apps[app_element.app_ID]<<endl;

	// Fill in the power values and the reset the empty flag of the grid nodes
	for (size_t i=0; i< app_element.map_region.coords.size(); i++){
		//m[app_element.map_region.coords[i].x][app_element.map_region.coords[i].y][app_element.map_region.coords[i].z].empty_flag = false;
		m[app_element.map_region.coords[i].x][app_element.map_region.coords[i].y][app_element.map_region.coords[i].z].power = app_element.map_region.tile_P[i];
	}
	return true;
}

// Called to check if any running app has exited
// Outputs true on app exit event. Else false.
bool app_exit(vector <app_DoP_pair> &app_queue, const int mesh_dim_x,
		const int mesh_dim_y, const int num_tiers, grid_node*** m, double &power_slack, int &avail_tiles, double op_time){
// Go through all the apps, and remove the apps from the queue whose completed flag is set
	cout<<"Main called app_exit function"<<endl;
	bool ret_flag = false;
	for (size_t a =0; a < app_queue.size(); a++){
		if(app_queue[a].completed){
			cout<<"app_exit: Removing app: "<< apps[app_queue[a].app_ID]<<endl;
			// Setting the grid nodes to empty
			for (size_t i=0; i < app_queue[a].map_region.coords.size(); i++){
				m[app_queue[a].map_region.coords[i].x][app_queue[a].map_region.coords[i].y][app_queue[a].map_region.coords[i].z].empty_flag = true;
				m[app_queue[a].map_region.coords[i].x][app_queue[a].map_region.coords[i].y][app_queue[a].map_region.coords[i].z].power = 0;
				m[app_queue[a].map_region.coords[i].x][app_queue[a].map_region.coords[i].y][app_queue[a].map_region.coords[i].z].frequency = 1000;
			}
			avail_tiles = avail_tiles + app_queue[a].DoP;

			// Freeing the node pointers
			// First remove the communication paths established on the links
			for (int i =0 ; i < app_queue[a].DoP; i++){
				for(int j=0; j < app_queue[a].tg_nodes[i]->out_degree; j++){
					int src_id, dest_id;
					double max_delay = 0;
					vector<int> path = app_queue[a].tg_nodes[i]->paths[j];

					src_id = app_queue[a].tg_nodes[i]->ID;
					dest_id = app_queue[a].tg_nodes[i]->out_v_connect[j];

					unbuild_paths(app_queue[a].tg_nodes, m, path, src_id, dest_id, mesh_dim_x, mesh_dim_y, num_tiers, max_delay);

				}
				delete app_queue[a].tg_nodes[i];
			}

			// erasing the app from the list of currently running apps.
			app_queue.erase(app_queue.begin()+ a);
			ret_flag = ret_flag || true;
		}

	}
	return ret_flag;
}
