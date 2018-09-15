#include "Constants.h"

extern vector<string> apps;
extern vector<short> DoPs;
extern vector<float> Vdds;

double get_fmax(float vdd);

// Evaluates the New volatages after mapping, un-mapping events or if any app has a modified set of power traces
// inputs: grid nodes, apps running on the chip, (power values are in the app data structure queue)
// Output: Updates the power values of the grid nodes.
int PSN_eval(grid_node*** m, vector<app_DoP_pair> &app_queue, double &peak_psn){
	// Initialize an array of power values, default 0
	double currents[DIM_X*DIM_Y*DIM_Z] = {0};
	float voltages[DIM_X*DIM_Y*DIM_Z] = {0};
	int emergencies = 0;

	//idiomatic way of filling up arrays in C++.
	fill_n(currents,DIM_X*DIM_Y*DIM_Z, 0.002);
	fill_n(voltages,DIM_X*DIM_Y*DIM_Z, 0.6);
	char* PDN = "sudo python PDN.py";
	char current [16] = {0};
	char voltage [8] = {0};

    // prepare a sequence of core current consumptions
    for (size_t i =0; i < app_queue.size(); i++){
    	app_DoP_pair app = app_queue[i];
    	if(app.map_region.coords.size() != app.map_region.tile_P.size()){
    		cout<<"Error: Tile coordinate vector and power vector are not of same size:";
    		cout<<app.map_region.coords.size()<<" "<< app.map_region.tile_P.size()<<endl;
    		exit (1);
    	}
    	cout<<"printing app power"<<endl;
    	for (int i = 0; i < app.map_region.coords.size(); i++ ){
    		cout<<app.map_region.coords[i].x<<" "<<app.map_region.coords[i].y<<" ";
    		cout<<app.map_region.tile_P[i]<<endl;
    	}
    	cout<<endl;
    	for (size_t tile = 0; tile < app.map_region.coords.size(); tile++){
    		int tile_id = DIM_X*(app.map_region.coords[tile].y) + app.map_region.coords[tile].x;
    		currents[tile_id] = (double)(app.map_region.tile_P[tile]/app.map_region.Vdd); // 7nm ARM core power is 8 times less than an intel xeon core
    		voltages[tile_id] = app.map_time_vdd;
    	}
    }

    // Fill up the remaining grid currents based on the router powers
    // Also fill up the actual vt_cores of the grid nodes
    for(int i=0; i < DIM_X; i++){
    	for(int j=0; j < DIM_Y; j++){
    		for(int k=0; k< DIM_Z; k++){
    			int tile_id = DIM_X*j + i;
    			currents[tile_id] += (double)(m[i][j][k].r_power/voltages[tile_id]);
    			m[i][j][k].Vt_core = voltages[tile_id];
    		}
    	}
    }
    // create a python command from the array
    // sudo python PDN.py '<current array>' '<voltage array>'
    string command = string(PDN);
    for(int i = 0; i< DIM_X*DIM_Y*DIM_Z; i++){
    	int ret = snprintf(current, sizeof current, "%f", currents[i]);
        if (ret < 0) {
        	cout<<"Error: Converting current values from float to char*"<<endl;
            exit(1);
        }
        else{
        	command = command + " " + string(current);
        }
    }
    for(int i = 0; i< DIM_X*DIM_Y*DIM_Z; i++){
     	int ret = snprintf(voltage, sizeof voltage, "%f", voltages[i]);
         if (ret < 0) {
         	cout<<"Error: Converting Voltage values from float to char*"<<endl;
             exit(1);
         }
         else{
         	command = command + " " + string(voltage);
         }
     }
    cout<<"Python PDN Command is : "<< command<<endl;

    // Call the Python script to run the PSN eval
    system(command.c_str());

    // Read the new Vdd values from the file and update the Cores
    ifstream instream_vdd;
    vector<float> vdd_values;
    instream_vdd.open("vdd_out.log");
    if (instream_vdd.is_open()){

    	istream_iterator <float> start(instream_vdd), end;
    	vdd_values = vector<float>(start, end);
    }

    if (vdd_values.size() < DIM_X*DIM_Y*DIM_Z){
    	cout<<"Error: vdd_out.log did not read the vdd values of all the cores"<<endl;
    	exit(1);
    }
    else{
    	float min_volt = 99999.0;
    	peak_psn = 0;
    	//fill the new _vdd values of each mapped application
    	for (size_t i =0; i < app_queue.size(); i++){
    		for (size_t tile = 0; tile < app_queue[i].map_region.coords.size(); tile++){
    			int index = DIM_X* app_queue[i].map_region.coords[tile].y + app_queue[i].map_region.coords[tile].x;
    			if (vdd_values[index] < min_volt){
    				min_volt = vdd_values[index];
    			}
    		}
    		app_queue[i].map_region.Vdd = min_volt;       // 7nm change
    		app_queue[i].frequency = get_fmax(min_volt);  // 7nm change


    		float diff = app_queue[i].map_time_vdd - min_volt;   // 7nm change
    		diff = diff*100/app_queue[i].map_time_vdd;

    		if (diff > peak_psn) peak_psn = diff;

    		if (diff > VE_THRESH){
    			cout<<"ALERT: Voltage Emergency observed"<<endl;
    			emergencies++;

    			// Take the app 10,000 cycles ahead.
    			if (app_queue[i].lastRecCycle > 100000)
    				app_queue[i].lastRecCycle -= 100000;
    		}
    	}
   		// update the frequency and psn_induced Vdds of the grid nodes after psn_simulation
	  for(int i=0; i < DIM_X; i++){
			for(int j=0; j < DIM_Y; j++){
				for(int k=0; k< DIM_Z; k++){
					int tile_id = DIM_X*j + i;
					m[i][j][k].current_Vdd = vdd_values[DIM_X*j + i];
					m[i][j][k].frequency = get_fmax(m[i][j][k].Vt_core);
				}
			}
		}
    }
    return emergencies;
}
