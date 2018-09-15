// this file has all the functions that will be used by the mapping hueristic while computing the execution times and etc

#include "Constants.h"


// Approximate ideal Execution times for variable DoP
double get_exec_time(int app_DoP, double exec_time, int new_DoP, int app_ID){
	double new_exec_time =exec_time;
	switch(app_ID){
	case 1: //Dedup-1,
		if(new_DoP > app_DoP){ int mul = new_DoP/app_DoP; new_exec_time = exec_time - (log2(mul)*100);}
		else if(new_DoP < app_DoP) { int div = app_DoP/new_DoP; new_exec_time = exec_time + (log2(div)*100);}
		return (new_exec_time);
		break;
	case 2: //Canneal-2,
		if(new_DoP > app_DoP){ int mul = new_DoP/app_DoP; new_exec_time = exec_time - (log2(mul)*10);}
		else if(new_DoP < app_DoP) { int div = app_DoP/new_DoP; new_exec_time = exec_time + (log2(div)*10);}
		return (new_exec_time);
		break;
	case 3: //fft-3,  used a stupid logic for this one
		if(new_DoP > app_DoP){
			if(new_DoP/app_DoP == 2) new_exec_time = exec_time - (8);
			else if(new_DoP/app_DoP == 4) new_exec_time = exec_time - (12);
			else if (new_DoP/app_DoP == 8) new_exec_time = exec_time - (14);
		}
		return (new_exec_time);
		break;
	case 4: // raytrace-4,
		if(new_DoP > app_DoP){
			if(new_DoP/app_DoP == 2) new_exec_time = exec_time - (4);
			else if(new_DoP/app_DoP == 4) new_exec_time = exec_time - (6);
			else if (new_DoP/app_DoP == 8) new_exec_time = exec_time - (7);
		}
		return (new_exec_time);
		break;
	case 5: // Vips-5,
		if(new_DoP > app_DoP){ int mul = new_DoP/app_DoP; new_exec_time = exec_time/(mul);}
		else if(new_DoP < app_DoP) { int div = app_DoP/new_DoP; new_exec_time = exec_time *(div);}
		return (new_exec_time);
		break;
	case 6: //  Cholesky-6,
		if(new_DoP == app_DoP/2) new_exec_time = exec_time + 18;
		else if (new_DoP == app_DoP*2) new_exec_time = exec_time - 12;
		else if (new_DoP == app_DoP*4) new_exec_time = exec_time - 18;
		return (new_exec_time);
		break;
	case 7: //  Radix-7,
		if(new_DoP > app_DoP){ int mul = new_DoP/app_DoP; new_exec_time = exec_time/(mul);}
		else if(new_DoP < app_DoP) { int div = app_DoP/new_DoP; new_exec_time = exec_time *(div);}
		return (new_exec_time);
		break;
	case 8: //Streamcluster-8,
		if(new_DoP == app_DoP/2) new_exec_time = exec_time + 20;
		else if (new_DoP == app_DoP*2) new_exec_time = exec_time;
		else if (new_DoP == app_DoP/4) new_exec_time = exec_time + 40;
		return (new_exec_time);
		break;
	case 9: //Bodytrack-9,
		if(new_DoP == app_DoP*2) new_exec_time = exec_time - 150;
		else if (new_DoP == app_DoP*4) new_exec_time = exec_time - 180;
		else if (new_DoP == app_DoP*8) new_exec_time = exec_time - 200;
		return new_exec_time;
		break;
	case 10: //Fluidanimate-10,
		if(new_DoP == app_DoP/2) new_exec_time = exec_time + 100;
		else if (new_DoP == app_DoP*2) new_exec_time = exec_time - 40;
		else if (new_DoP == app_DoP*4) new_exec_time = exec_time - 80;
		return (new_exec_time);
		break;
	case 11: // Radiosity-11,
		if(new_DoP == app_DoP/2) new_exec_time = exec_time + 20;
		else if (new_DoP == app_DoP/4) new_exec_time = exec_time + 60;
		else if (new_DoP == app_DoP/8) new_exec_time = exec_time + 140;
		return (new_exec_time);
		break;
	case 12: // Blackscholes-12,
		if(new_DoP == app_DoP*2) new_exec_time = exec_time - 10;
		else if (new_DoP == app_DoP*4) new_exec_time = exec_time - 20;
		else if (new_DoP == app_DoP*8) new_exec_time = exec_time - 25;
		return (new_exec_time);
		break;
	case 13: // Swaptions-13
		if(new_DoP == app_DoP/2) new_exec_time = exec_time + 40;
		else if (new_DoP == app_DoP/4) new_exec_time = exec_time + 40;
		else if (new_DoP == app_DoP/8) new_exec_time = exec_time + 80;
		return (new_exec_time);
		break;
	default:
		return (new_exec_time);
		break;
	}
	return new_exec_time;
}

// approximate the execution times for variable freq
double get_exec_time_freq(int freq, double exec_time, int new_freq){
	//cout<<"Var_DoP_VF_utils.cpp: get_exec_time_freq: old_freq: "<<freq<<" old compt_time "<< exec_time <<" new_freq: "<<new_freq;
	double new_exec_time = exec_time;
	double slope = 0.05 ; // 5 ms is the step increase in freq;
	int delta_freq = freq-new_freq ;
	double delta_times = slope * delta_freq;
	if((exec_time+delta_times)<=0 ){ new_exec_time = exec_time/2; return new_exec_time;}
	new_exec_time = exec_time + (delta_times); // change in the exec_times
	cout<<" new_exec_time: "<<new_exec_time<<endl;
	return new_exec_time;
}
// Added to change the max_execution_time constraint for routers when the execution time changes
double max_times_DoP(double max_time, double exec_time, double new_exec_time){
	double slope =1;
	cout<<"new_exec_time "<<new_exec_time<<" old_exec_time "<<exec_time<<" old max_time "<<max_time<<endl;
	if(exec_time > 0) slope = (new_exec_time/exec_time);
	else {cout<<"Var_DoP_VF_utils.cpp: exec_time is 0"<<endl; exit(1);}
	double del_max_time = slope * max_time;
	//cout<< " new_app_run_time_constraint "<<del_max_time<<endl;
	return (del_max_time);
}

// fills total_runtime and ret_val '0' when the overheads can be computed, else ret_val = 1
bool get_computation_overheads(int DoP, double compute_time, double deadline, int num_errors, double & total_runtime){
	// Decide the no. of checkpoints
	// deadline here is not the absolute time.. its a time diff between arrival and deadline time
	double q_i = 1; // millisecs per check point
	double o_i = 7; // millisecs per check point
	double r_i = 2.5; // millisecs per check point
	double over_r = 5.5; // ms per recovery
	//double extra = deadline - compute_time;
	int chk_num = 5;
	// compute the overhead (check pointing overhead, rollback overheads)
	// Make sure that there is time to accommodate x errors
	//cout<<"Var_DoP_Vf_utils.cpp: # of chk_points "<<chk_num<<endl;
	//if(chk_num < 0) {cout << "Var_DoP_Vf_utils.cpp: ERROR - we have to drop this app, cannot handle at least two errors " << endl; exit(1);}
	double time_overhead = (compute_time + chk_num*o_i + (chk_num+1)*q_i);
	double error_overhead = time_overhead+(r_i*num_errors + (time_overhead/chk_num)+q_i);
	cout<<"Var_DoP_VF_utils : comp_time: "<<compute_time<<" deadline: "<<deadline;
	cout<<" overall execution time is: "<<error_overhead<<endl;
	if (deadline < 0){
		cout<<"ALERT: get_compute_overheads: deadline is negative"<<endl;
	}
	else if(error_overhead > deadline){
		cout<<"BUZZ: get_compute_overheads: too many errors, skipping this DoP-Vdd pair!"<<endl;
		return true;
	}
	else {
		/*cout<<"get_compute_overheads: overhead is within limit"<<endl;*/
		total_runtime = error_overhead; return false;
	}
}

