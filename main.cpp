// This is main function for the implementation of PSN aware mapping for manycore systems.
//--- apps arriving with random app-sequences.. WE use SPLASH2 and PARSEC for starters
// DoP is variable per app -- 4, 8, 16, or 32 -- with 14 apps say
// we consider a 60-core 1-tier 2D-chip..
// -- we assume the same compute-times for all apps -- with compute-intensive apps running at higher frequencies -- 
// -- FOR NOW, assume linear reduction in compute-times with increasing DoP levels --
// also, assume same dyn/leakage power equations for all cores with the same switching capacitance (irrespective of DoP or the app)  -- at least for now until inputs from SNIPER tool --
// -- also, lets have a standard compute-time for say 8-cores (as a const variable) and scale it according to the DoP level --
// mapping process is assumed to be instantaneous , at least for now ---
// -- lets assume Vdds of 0.5,..1.0V.. and freqs between 1300 MHZ to 2GHZ in increments of 100 MHZ --
// 8-core compute time to be 4 secs -- 1-core = 4x8 = 32 secs -- 16-core = 4/2 = 2 secs
// -- Presently we only consider Vt-variations.. later if needed to enhance the magnitude of variations, can introduce Leff-variations as well, so that Vt and Leff are correlated -- 
// Lets assume Vt sensors on-chip -- one per core and one per each router -- look at aging sensors papers later --
// -- app_IDs 1 to 7 are SPLASH2 and app_IDs 8 to 14 are PARSEC -- IDs increasing with rated frequencies..

#include "Constants.h"

// ******* functions declared *******************
bool map_app (app_DoP_pair &app_element, const int mesh_dim_x,
		const int mesh_dim_y, const int num_tiers, grid_node*** m, double &power_slack, int &avail_tiles, double op_time);// -- Vdd should tried to be lowered when an app leaves, while retaining the SEC and freq. constraint satisfaction --- Power_slack needs to be updated if Vdd reduced ---
bool app_exit (vector <app_DoP_pair> &app_vector, const int mesh_dim_x,
		const int mesh_dim_y, const int num_tiers, grid_node*** m, double &power_slack, int &avail_tiles, double op_time);
int PSN_eval(grid_node*** m, vector<app_DoP_pair> &app_queue, double &peak_psn);
app_pow_traces get_app_data(int app_id);
double get_max_run_time(app_pow_traces apt);
bool update_power(grid_node*** m, vector <app_DoP_pair> &app_queue, double op_time, int routing_sel);
bool map_app_compare_1(app_DoP_pair &app_element, const int mesh_dim_x,
		const int mesh_dim_y, const int num_tiers, grid_node*** m, double &power_slack, int &avail_tiles, double op_time);
bool map_app_psn_aware(app_DoP_pair &app_element, const int mesh_dim_x,
		const int mesh_dim_y, const int num_tiers, grid_node*** m, double &power_slack, int &avail_tiles, double op_time);
bool map_route(app_DoP_pair &app_element, const int mesh_dim_x, const int mesh_dim_y, const int num_tiers, grid_node*** m1);

bool myfunction (double i,double j) { return (i>j); }
// **********************************************
//================Sim constants=======================//
char const* app_array[] = {"", "dedup", "canneal", "ocean.cont", "raytrace", "water.sp", "radix","streamcluster", "bodytrack", "fluidanimate",
"radiosity", "blackscholes", "swaptions"};
vector<string> apps(app_array, app_array + arraysize(app_array));
const short DoP_array[] = {32, 16, 8, 4};
vector<short> DoPs(DoP_array, DoP_array+ arraysize(DoP_array));
float Vdd_array[] = {0.75, 0.8, 0.9, 1.0, 1.1}; // 75 = 0.75, 8 = 0.8, 9 = 0.9, 10 = 1, 11 = 1.1 Volts
vector <float> Vdds_trace(Vdd_array, Vdd_array + arraysize(Vdd_array));
float Vdd_arm_array[] = {0.8, 0.7, 0.6, 0.5, 0.4};
vector <float> Vdds(Vdd_arm_array, Vdd_arm_array + arraysize(Vdd_arm_array));
//================Sim constants=======================//
int main(int argc, char* argv[])
{
	short fw_type = 0;
	short routing_sel = 0;

	if (argc != 3)
	{
		cout<<"Error: Give the mapping framework # as input"<<endl;
		cout<<"Usage: ./psn_mapping <fw_type> <routing_type>"<<endl;
		cout<<"fw_type = 0, default, contiguous mapping"<<endl;
		cout<<"fw_type = 1, comparison mapping 1"<<endl;
		cout<<"fw_type = 2, proposed mapping scheme"<<endl;
		cout<<"routing_type = 0, default XY routing"<<endl;
		cout<<"routing_type = 1, Iconoclast routing"<<endl;
		cout<<"routing_type = 2, PSN_aware routing"<<endl;
		exit(1);
	}
	else{
		fw_type = atoi(argv[1]);
		routing_sel = atoi(argv[2]);
	}
// Read the freq. from the core-graph itself, and save in corresponding node_pointers --
// When the application finishes, flush out the corresponding grid-nodes
// if the Vdd is lowered at the time of the application finishing or Vdd hiked after mapping (lets call them events), need to re-calculate power/energy dissipation from that point on.
// -------------------------------------------
// Note that for block_placement, node_pointers may not need to be initialized as communication not considered, only the 1st line of core-graphs to read --
// -------------------------------------------
	const int mesh_dim_x = DIM_X;
	const int mesh_dim_y = DIM_Y;
	const int num_tiers = DIM_Z;

	ofstream outstream;
	vector <app_DoP_pair> app_arr_vec;
	app_DoP_pair arr_element;
	
	bool PSN_EVAL = false;
	int vec_ptr = 0;
	bool mark = 0; // end of simulation marker --

	//=========The variable that controls this simulation=========/
	double op_time = 0;
	//============================================================/

	double power_slack = POWER_BUDGET; // 100W
	int avail_tiles = DIM_X*DIM_Y*DIM_Z;
	double tot_pow = 0;
	int emergencies = 0;
	double peak_psn = 0;
	vector <double> avg_peak_psn;

	GridPtr **m = new GridPtr*[mesh_dim_x];	// array of pointers
	for(size_t i=0; i<mesh_dim_x; i++) {
		m[i] = new GridPtr[mesh_dim_y];

		for (size_t j=0; j<mesh_dim_y; j++)
			m[i][j] = new grid_node [num_tiers];   //Needs change to 2D array -- yash
	}
	app_arr_vec.clear();
	

// ---------------------------------------------------------------------------------------------------
//  at the occurance of an event, map one by one until some app is stalled due to dark-silicon budget or performance dissatisfation --
//  lets define an event when any app finishes 
	vector <double> comm_percentages;
	vector <double> dyn_coeffs;
	vector <double> stat_coeffs;
	vector <double> compute_times;
	vector <double> deadlines;
	vector <int> inter_arrival_times; // Used for making the events work

	comm_percentages.push_back(-1); // dummy
	comm_percentages.push_back(50.0); comm_percentages.push_back(41.0);
	comm_percentages.push_back(12.0); comm_percentages.push_back(10.0); comm_percentages.push_back(9.0);
	comm_percentages.push_back(8.0); comm_percentages.push_back(6.0); comm_percentages.push_back(4.0);
	comm_percentages.push_back(3.0); comm_percentages.push_back(2.5);
	comm_percentages.push_back(2.0); comm_percentages.push_back(0.1); comm_percentages.push_back(0.1);

	//Ex_times have to be modified according to the DoP and freq chosen --yash
	/*double compute_time;
	double ex_times[13] = {382.0, 887.0, 47.0, 365.0, 146.0, 99.0, 14.0, 120.0, 300.0, 280.0, 60.0, 70.0, 40.0};
	compute_times.push_back(-1); // dummy
	for (i=1; i<=13; i++) {
		compute_time = ex_times[i-1]*(1.0 - (comm_percentages[i]/100.0));
		compute_times.push_back(compute_time);   
	} // end for i
	cout << " compute_times " << endl;
	for (i=0; i<compute_times.size(); i++) {
		cout << compute_times[i] << " " ;
	} cout << endl;*/
	// ------------------------------------------------------------------------------------------------------------------------------------

	int seq_num = 2;
	ifstream instream1, instream2;
	// first, lets read the app-arrivals in a vector structure --
	if (seq_num == 1) {
		instream1.open("app_seq_comm.txt");  // -- comm-centric BMs --
	} else if (seq_num == 2) {
		instream1.open("app_seq_compute.txt");  // -- compute-intensive BMs --
	} else if (seq_num == 3) {
		instream1.open("app_seq_all.txt");  // -- mixed or all BMs ---
	} else { cout << " ERROR " << endl; exit(1); }
	// Now read the inter-arrival times from the file
	instream2.open("inter_arrival_times-1.txt");
	int interval = 0; double arrival_time = 0;

	// Save all the apps in the queue with their arrival time, constraints, and traces
	for (size_t i=0; i < TOTAL_INCOMING_APPS; i++) {
		arr_element.app_seq_num = i+1;
		instream1 >> arr_element.app_ID;
		arr_element.DoP = 0;
		arr_element.frequency = 0;
		arr_element.app_pt = get_app_data(arr_element.app_ID);
		arr_element.run_time_constraint = get_max_run_time(arr_element.app_pt);
		cout<<"App run time constraint: "<<arr_element.run_time_constraint<<endl;
		arr_element.compute_time = 0;
		arr_element.comm_perc = comm_percentages[arr_element.app_ID];

		arrival_time += (interval*ARRIVAL_RATE_MUL); // the interval times file is in seconds, convert it to ms.
		inter_arrival_times.push_back(arrival_time);
		arr_element.arrival_time = arrival_time;
		// Set a deadline for this app
	    std::random_device rd;
	    std::mt19937 e2(rd());
	    std::uniform_real_distribution<> dist(0, 1);
	    // sub a random time from max_run_times
		double deadline = arr_element.run_time_constraint - (dist(e2))*arr_element.run_time_constraint/10;
		arr_element.deadline = arrival_time + deadline;
		app_arr_vec.push_back(arr_element);
		cout<<"main.cpp: app_deadline: "<<arr_element.deadline<<endl;
		instream2 >> interval;
	}// end for i
	cout<<""<<endl;
	instream1.close();
	instream2.close();
	if(app_arr_vec.size() != inter_arrival_times.size())
	{
		cout<<"ERROR: main.cpp: size of interarrival rates and app vector not same" << endl;
		exit(1);
	}
	// print-out the arrival vector
	cout << " app_seq_num  app_ID  arr_time_stamp " << endl;
	for (size_t i=0; i<app_arr_vec.size(); i++) {
		cout << app_arr_vec[i].app_seq_num << " " << app_array[app_arr_vec[i].app_ID] << " " << app_arr_vec[i].arrival_time << endl;
	}
	//exit(1);


	int apps_completed = 0;
	vector <app_DoP_pair> mapped_apps;
	// Iterate through the app_list for 10 times and stop
	int count = 0;
	while(count < 1){
		cout <<"=========Starting Simulation=========="<<endl;
		mapped_apps.clear();
		vec_ptr = 0;
		op_time = 0;
		mark = false;
		while (!mark) {
			cout<<"MainL: vec_ptr: "<<vec_ptr<<" app_arr_vector size: "<<app_arr_vec.size()<<endl;
			//check the front of the queue, if the app-arrival-time < current time, map it, set PSN_EVAL if mapping successful
			if (app_arr_vec[vec_ptr].arrival_time <= op_time && vec_ptr < app_arr_vec.size()){
				// If the deadline has already past? drop the application
				if (app_arr_vec[vec_ptr].deadline < op_time ){
					cout<<"Main: Deadline overdue, moving on "<<app_array[app_arr_vec[vec_ptr].app_ID]<<endl;
					vec_ptr++;
				}
				else if (fw_type == 0){
					if (map_app(app_arr_vec[vec_ptr],mesh_dim_x, mesh_dim_y, num_tiers, m, power_slack, avail_tiles, op_time)){
						// Establish the task graphs for the app mapped.
						if(map_route(app_arr_vec[vec_ptr], mesh_dim_x, mesh_dim_y,num_tiers, m)){
							cout<<"Main: Task nodes creation - success"<<endl;
						}
						else{
							cout<<"Main: Task nodes not created"<<endl;
							exit(1);
						}
						mapped_apps.push_back(app_arr_vec[vec_ptr]);
						PSN_EVAL = PSN_EVAL || true;
						vec_ptr ++;
					}
					else{
						cout<<"Main: map_app returned false"<<endl;
						if (mapped_apps.size() == 0){
							cout<<"ALERT: The app is not mapped on an empty chip. moving on"<<endl;
							vec_ptr++;
						}
					}
				}
				else if(fw_type == 1){
					if (map_app_compare_1(app_arr_vec[vec_ptr],mesh_dim_x, mesh_dim_y, num_tiers, m, power_slack, avail_tiles, op_time)){
						// Establish the task graphs for the app mapped.
						if(map_route(app_arr_vec[vec_ptr], mesh_dim_x, mesh_dim_y,num_tiers, m)){
							cout<<"Main: Task nodes creation - success"<<endl;
						}
						else{
							cout<<"Main: Task nodes not created"<<endl;
							exit(1);
						}
						mapped_apps.push_back(app_arr_vec[vec_ptr]);
						PSN_EVAL = PSN_EVAL || true;
						vec_ptr ++;
					}
					else{
						cout<<"Main: map_app_compare returned false"<<endl;
						if (mapped_apps.size() == 0){
							cout<<"ALERT: The app is not mapped on an empty chip. moving on"<<endl;
							vec_ptr++;
						}
					}
				}
				else if (fw_type == 2){
					if (map_app_psn_aware(app_arr_vec[vec_ptr],mesh_dim_x, mesh_dim_y, num_tiers, m, power_slack, avail_tiles, op_time)){
						// Establish the task graphs for the app mapped.
						if(map_route(app_arr_vec[vec_ptr], mesh_dim_x, mesh_dim_y,num_tiers, m)){
							cout<<"Main: Task nodes creation - success"<<endl;
						}
						else{
							cout<<"Main: Task nodes not created"<<endl;
							exit(1);
						}
						mapped_apps.push_back(app_arr_vec[vec_ptr]);
						PSN_EVAL = PSN_EVAL || true;
						vec_ptr ++;
					}
					else{
						cout<<"Main: map_app_psn returned false"<<endl;
						if (mapped_apps.size() == 0){
							cout<<"ALERT: The app is not mapped on an empty chip. moving on"<<endl;
							vec_ptr++;
						}
					}
				}
			}


			// get new power values for the apps running on the chip, if successful, set PSN_EVAL
			// new power values are updated for the apps running in the queues.
			if(update_power(m, mapped_apps, op_time, routing_sel))
			{
				cout<<"Main: power values changed at "<<op_time<<endl;
				PSN_EVAL = PSN_EVAL || true;
			}

			// check for any app exit at the new time step, if app exits, set PSN_EVAL
			if (app_exit(mapped_apps, mesh_dim_x, mesh_dim_y, num_tiers, m, power_slack, avail_tiles, op_time)){
				cout<<"Main: App exit at "<<op_time<<endl;
				apps_completed++;
				PSN_EVAL = PSN_EVAL || true;
			}
			// if PSN_EVAL is set for this time step, calculate the new Vdd values
			if (PSN_EVAL && op_time > 0){
				cout<<"Main: PSN_EVAL is set at "<<op_time<<endl;
				emergencies += PSN_eval(m, mapped_apps, peak_psn);
				avg_peak_psn.push_back(peak_psn);
			}
			// increment time to next time step
			op_time += TIME_STEP;
			cout << "Simulation time: " << op_time << endl;

			// When no apps to map and the current apps have completed executing, end the sim
			if(vec_ptr == app_arr_vec.size() && mapped_apps.empty())
				mark = true;

		}
		count++;
	}

// ----------------------- tot_num_apps run over lifetime ----------------
	cout<<"<===================Simulation ended=================>"<<endl;
	cout<<"Total exeuction time is: "<< op_time<<endl;
	cout << " vec_ptr is: " << vec_ptr << endl;
	cout<<" num of apps completed: "<< apps_completed<<endl;
	cout << " num of VEs observed: " << emergencies << endl;

	std::sort(avg_peak_psn.begin(), avg_peak_psn.end(), myfunction);
	cout<<" peak PSN observed "<< *avg_peak_psn.begin();
	double avg_psn = 1.0 * std::accumulate(avg_peak_psn.begin(), avg_peak_psn.end(), 0LL) / avg_peak_psn.size();
	cout <<" avg_psn observed "<< avg_psn<<endl;
// ------------------------------------------------------------------------

	
  return 0;
} // end main
