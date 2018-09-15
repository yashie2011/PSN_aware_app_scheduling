// When an app is incoming, schedule it based on the DS_PB, available space budget, and execution time constraint
#include "Constants.h"

#define BUFFSIZE 100
extern char const *app_array[];
extern vector<string> apps;
extern vector<short> DoPs;
extern vector<float> Vdds_trace;
extern vector<float> Vdds;

int router_select = 0;   // global variable to select routers to throttle

void routing_new(app_DoP_pair &app_element, grid_node*** m1,
		node* node_pointers[], const int mesh_dim_x, const int mesh_dim_y,
		const int num_tiers, int routing_sel);

bool update_completed_vols(app_DoP_pair &app_element, grid_node*** m1,
		node* node_pointers[], const int mesh_dim_x, const int mesh_dim_y,
		const int num_tiers);

void update_task_bw(app_DoP_pair &app_element, grid_node*** m1,
		node* node_pointers[], const int mesh_dim_x, const int mesh_dim_y,
		const int num_tiers);

void mark_router(grid_node*** m1, bool x_even, bool y_even);

string sniper_out = "/home/yaswanth/Dropbox/Documents/Codes/PSN_project/PSN_mapping/power_traces/Power_Inst_trace_PSN/";
string power_trace_path = "/Power_trace.csv";

// Return the Fmax of a core given its operating Vdd
double get_fmax(float vdd){
	if (vdd <= 0){
		cout<<"Error: Vdd is not positive"<<endl;
		exit(1);
	}
	double fmax = (1000.0/((vdd/3.0)/(pow(vdd-0.20, 1.2))));
	return fmax;
}

// Inputs: A power_trace_file
// output: A power_trace map of the power_trace_file
power_trace read_sim_out(const char* bench_name){
	// Read all the power_traces of the particular benchmark and get the max. execution times, return a variable
	string filename = sniper_out;
	filename = filename + bench_name;
	filename = filename + power_trace_path;
	ifstream pow_trace_file;
	power_trace pow_trace = {};

	pow_trace_file.open(filename.c_str());
	if (! pow_trace_file){
		cout<<"Cannot find the benchmark "<< filename<<endl;
		return pow_trace;
	}
	else{
		//Read each line to get the Instruction cycle values and the power traces of each core
		string line = " ";
		vector <float> trace;
		while (getline(pow_trace_file, line)){
			istringstream record(line);
			string entry = "";
			short count = 0;
			double timestamp = 0;
			char delim = ' ';
			trace.clear();
			while (getline(record, entry, delim)){
				if (count == 0)
				{
					timestamp = atof(entry.c_str());
					//cout<<"timestamp: "<<timestamp;
					count ++;
				}
				else{
					trace.push_back(atof(entry.c_str()));
					//cout<<" "<<entry;
					count ++;
				}
			}
			//cout<<endl;
			pow_trace.insert(std::pair<double, vector<float> >(timestamp, trace));
		}
		return pow_trace;
	}
}

// Input: app ID
// output: A map of DoP, Vdd, Power_trace structure.
app_pow_traces get_app_data(int app_id){
	char const* cores = "_64_";
	char const* us = "_";
	char const* architecture = "gainestown";
	power_trace pt;
	app_pow_traces apt;
	pow_trace_vdd ptv;
	cout<< "Processing app_id " << app_id<<" "<<apps[app_id]<<endl;
	string benchmark = apps[app_id];
	for (size_t dop = 0; dop< DoPs.size(); dop++){
		ptv.clear();
		for (size_t vdd = 0; vdd< Vdds_trace.size(); vdd++){
			//prepare the bench_name
			int vdd_name = 0;
			if (Vdds_trace[vdd] == 0.75) vdd_name = Vdds_trace[vdd]*100;
			else vdd_name = Vdds_trace[vdd]*10;

			char bench_name[50];
			char DoP[4] = {0};
			char Vdd[4] = {0};
			sprintf(DoP, "%d", DoPs[dop]);
			sprintf(Vdd, "%d", vdd_name);

			strcpy(bench_name, benchmark.c_str());
			strcat(bench_name, cores);
			strcat(bench_name, DoP);
			strcat(bench_name, us);
			strcat(bench_name, architecture);

			//get the power_trace
			if (Vdds_trace[vdd] == 1.0){
				cout<<"benchmark_name is "<< bench_name<<endl;
				pt = read_sim_out(bench_name);
			}
			else{
				strcat(bench_name, us);
				strcat(bench_name, Vdd);
				cout<<"benchmark_name is "<< bench_name<<endl;
				pt = read_sim_out(bench_name);
			}
			if (pt.size() > 0){
				//add to the app_pow_traces
				ptv.insert(std::pair<float, power_trace >(Vdds[vdd], pt));
			}
		}
		if (ptv.size() > 0)
			apt.insert(std::pair<short, pow_trace_vdd >(DoPs[dop], ptv));
	}
	return apt;
}


// driver function to check these functions.
int driver_func(){
	int id[] = {11};
	for (size_t i = 0; i< arraysize(id); i++){
		app_pow_traces apt;
		apt = get_app_data(id[i]);
		for (app_pow_traces::iterator it = apt.begin(); it != apt.end(); ++it){
			cout<<"DoP: "<<it->first<<" size of the vdd map "<< it->second.size();
			for (pow_trace_vdd::iterator itv = it->second.begin(); itv != it->second.end(); ++itv){
				cout<<" Vdd of this app "<< itv->first<<endl;
				for(power_trace::iterator itp = itv->second.begin(); itp != itv->second.end(); ++itp){
					cout<<"timestamp: "<< itp->first;
					for(size_t j=0; j<itp->second.size(); j++){
						cout<<j<<" "<<itp->second.at(j)<<" ";
					}
					cout<<endl;
				}
			}
		}
	}
	return 0;
}

// input takes app_pow_trace
// output gives runtime_constraint
double get_max_run_time(app_pow_traces apt){
	double max_run_time = 0;
	if (apt.size() == 0){
		cout<<"get_max_run_time: app does not have traces"<<endl;
		exit(1);
	}
	for (app_pow_traces::iterator it = apt.begin(); it != apt.end(); ++it){
		cout<<"DoP: "<<it->first<<" size of the vdd map "<< it->second.size()<<endl;
		for (pow_trace_vdd::iterator itv = it->second.begin(); itv != it->second.end(); ++itv){
			cout<<" Vdd of this app "<< itv->first<<endl;
			if (max_run_time < itv->second.rbegin()->first){
				max_run_time = itv->second.rbegin()->first;
			}
		}
	}
	// But the max run time is for 2.66 GHz. We want it for less frequency
	max_run_time = (double)(max_run_time/SIM_MIN_FREQ);
	return max_run_time;
}

double get_run_time (app_pow_traces apt, float vdd, short dop, float fmax){
	double run_time = 0;
	cout<<"get_run_time inputs: "<< vdd<<" "<< dop<<" "<<fmax<<" "<<endl;
	if (apt.size() == 0){
		cout<<"app does not have traces"<<endl;
		exit(1);
	}
	app_pow_traces::iterator it = apt.find(dop);
	if (it == apt.end()){
		cout<<"get_run_time: Trace does not exist for this DoP"<< endl;
		return run_time;
	}
	else{
		pow_trace_vdd::iterator itv = it->second.find(vdd);
		if (itv == it->second.end()){
			cout<<"get_run_time: Trace does not exist for this Vdd"<< endl;
			return run_time;
		}
		else {
			run_time = itv->second.rbegin()->first;
		}
	}
	/*for (app_pow_traces::iterator it = apt.begin(); it != apt.end(); ++it){
		if(it->first == dop){
		cout<<"DoP: "<<it->first<<" size of the vdd map "<< it->second.size();
		for (pow_trace_vdd::iterator itv = it->second.begin(); itv != it->second.end(); ++itv){
			cout<<" Vdd of this app "<< itv->first<<endl;
			if (run_time < itv->second.rbegin()->first){
				run_time = itv->second.rbegin()->first;
			}
		}
	}
	}*/
	// Get the run time from cycles
	run_time = (double)(run_time/(fmax*pow(10.0,6)));
	return run_time;

}
// TODO: this needs major revamping
// input takes cycle number, and Fmax
// output returns a vector of powers consumed at each core/ Currents drawn at each core
// returns an empty trace if the cycle
vector <float> get_power_trace(power_trace pt, short dop, double cycle_number, float frequency){
	// if nearest cycle is equal to the ceiling
	// Is the cycle ever going to be an integer of 100000???
	cout<<"getting power trace inputs: "<<cycle_number<<" "<< dop<<" "<<frequency<<endl;
	vector <float> trace = {};
	if(cycle_number == 0 && pt.empty()){
		cout<<"Trace not found for this DoP and Vdd, try another Vdd, DoP"<<endl;
		return trace;
	}
	//Get the cycle value with trace below the current cycle.
	double temp = floor(cycle_number/(SNIPER_TRACE_INTERVAL))*(SNIPER_TRACE_INTERVAL);
	cout<<"returning power_trace at cycle_number: "<<temp<<endl;
	if(temp < pt.begin()->first){
		cout<<"The app execution is still below 200,000 ms"<<endl;
		for (size_t i =0; i < pt.begin()->second.size() && i < dop; i++){
			trace.push_back(pt.begin()->second[i]*frequency*pow(10.0,6)/(ARM_CORE_POW_REDUCTION * SNIPER_FREQ));
		}
		cout<<"get_power_trace: trace size "<<trace.size()<<endl;
		for (auto i = trace.begin(); i != trace.end(); ++i)
		    std::cout << *i << ' ';
		cout<<endl;
		return trace;
	}
	power_trace::iterator it = pt.find(temp);
	if (it == pt.end()){
		// return an empty trace.
		power_trace::reverse_iterator rit = pt.rbegin();
		if(temp > rit->first){
			cout<<"get_power_trace: reached the end of the trace, END THE APP if there is no task comms"<<endl;
			return trace;
		}
		cout<<"get_power_trace: can't find the trace at: "<< temp <<endl;
		exit(1);
	}
	else{
		// return the trace of the cycle that matched the trace.
		// Change the power values obtained based on the frequency of the application
		for(size_t i=0; i < it->second.size() && i < dop; i++){
			trace.push_back(it->second[i]*frequency*pow(10.0,6)/(ARM_CORE_POW_REDUCTION * SNIPER_FREQ));
		}
		cout<<"get_power_trace: trace size "<<trace.size()<<endl;
		for (auto i = trace.begin(); i != trace.end(); ++i)
		    std::cout << *i << ' ';
		cout<<endl;
		return trace;
	}
}

// Updates the power traces of all the apps in that time instance.
// returns false when the power values did not change, else true
bool update_power(grid_node*** m, vector <app_DoP_pair> &app_queue, double op_time, int routing_sel){
	bool ret_flag = false;
	bool x_even = false;
	bool y_even = false;

	// compute x_odd, y_odd flags and call mart_router function
	router_select ++;
	if((router_select & 3) == 0 ){
		mark_router(m, x_even, y_even);
	}
	else if ((router_select & 3) == 1 ){
		y_even = true;
		mark_router(m, x_even, y_even);
	}
	else if ((router_select & 3) == 2 ){
		x_even = true;
		mark_router(m, x_even, y_even);
	}
	else if ((router_select & 3) == 3 ){
		x_even = true; y_even = true;
		mark_router(m, x_even, y_even);
	}

	// Compute new routes to each mapped application
	for (size_t app=0; app < app_queue.size(); app++){
		app_DoP_pair app_i = app_queue[app];
		routing_new(app_i, m, app_i.tg_nodes,  DIM_X, DIM_Y, DIM_Z, routing_sel);
	}

	// update the bandwidths allocated to each task edge after the new routing
	for (size_t app=0; app < app_queue.size(); app++){
		app_DoP_pair app_i = app_queue[app];
		update_task_bw(app_i, m, app_i.tg_nodes,  DIM_X, DIM_Y, DIM_Z);
	}

	// update the volumes completed at this step
	for (size_t app=0; app < app_queue.size(); app++){
		app_DoP_pair app_i = app_queue[app];
		update_completed_vols(app_i, m, app_i.tg_nodes, DIM_X, DIM_Y, DIM_Z);
	}

	// compute Grid router powers based on their activity
	for (int i=0; i < DIM_X; i++){
		for (int j=0; j< DIM_Y; j++){
			for (int k=0; k < DIM_Z; k++){
				double tot_act = 0;

				if(m[i][j][k].E_out_comms > 0) tot_act += 1;
				if(m[i][j][k].W_out_comms > 0) tot_act += 1;
				if(m[i][j][k].N_out_comms > 0) tot_act += 1;
				if(m[i][j][k].S_out_comms > 0) tot_act += 1;
				if(m[i][j][k].U_out_comms > 0) tot_act += 1;
				if(m[i][j][k].D_out_comms > 0) tot_act += 1;

				m[i][j][k].r_power = (tot_act * ROUTER_DYN_POW_MUL) + ROUTER_LK_POW;
				//cout<<"tot_act: "<<i<<" "<<j<<" "<<tot_act<<" "<<m[i][j][k].r_power<<endl;
			}
		}
	}

	for (size_t app=0; app < app_queue.size(); app++){
		app_DoP_pair app_i = app_queue[app];

		cout<<"Updating the power of the app: "<< app_array[app_i.app_ID]<<endl;
		app_pow_traces::iterator it_d = app_i.app_pt.find(app_i.DoP);
		if (it_d == app_i.app_pt.end()){
			cout<<" The Dop doesnt exist "<<app_i.DoP<<endl;
			exit(1);
		}
		else{
			pow_trace_vdd &apv = it_d->second;
			pow_trace_vdd::iterator it_v = apv.find(app_i.map_time_vdd);
			if (it_v == apv.end()){
				cout<<" The Vdd doesnt exist "<<app_i.map_time_vdd;
				exit(1);
			}
			else{
				power_trace &apt = it_v->second;
				double cycle = (app_i.lastRecCycle + TIME_STEP* app_i.frequency* pow(10.0, 6));
				app_queue[app].lastRecCycle = cycle;
				vector<float> app_power_values = get_power_trace(apt, app_i.DoP, cycle, app_i.frequency);
				if (app_power_values.size() == 0){
					cout<<"get_power_trace Returned an empty power trace"<<endl;
					if (cycle > 0){
						// check if the app has completed its communication
						if (update_completed_vols(app_i, m, app_i.tg_nodes, DIM_X, DIM_Y, DIM_Z)){
							app_queue[app].completed = true;
						}
						// The communication vol is still remaining, set the ret_flag to true to enable PSN_Eval
						else{
							ret_flag = ret_flag || true;
						}
					continue;
					}
				}
				// check if the power values obtained are the same ones as the old ones.
				// or if the comm. is still going on.
				else{
					if (app_i.map_region.tile_P.size() != app_power_values.size()){
						cout<<"update_power: new trace size did not match the existing trace"<< endl;
						exit(1);
					}
					for (size_t pti = 0; pti< app_i.map_region.tile_P.size(); pti++){
						if (app_power_values[pti] != app_i.map_region.tile_P[pti]){
							app_queue[app].map_region.tile_P[pti] = app_power_values[pti];
							//change the grid node power as well
							m[app_queue[app].map_region.coords[pti].x][app_queue[app].map_region.coords[pti].y][app_queue[app].map_region.coords[pti].z].power = app_queue[app].map_region.tile_P[pti];
							ret_flag = ret_flag || true;
						}
						else{
							continue;
						}
					}
				}
		    }
	    }
    }



	return ret_flag;
}
