// ---------------------------------------------------------------------------------------
// For starters, lets just choose the block with least leakage power (with slowest avg_Vt that satisfies freq. constraints) -----
// Later augment the cost func. with other things like PDN and NBTI-awareness -----
//*****************************************************************************************************
//*********************************************  NEW  ********************************************************
// -- Lets just do the power_slack re-eval and PC-constraint check within this func. (and finalize mapping) because NoC power is not considered any more ---
// -- WE now try out increasing Vdds to meet freq. constraints within this block_placement func. -- 
//     -- if PC constraint cannot be met for the current Vdd, we return 0, i.e., no need to hike Vdd 
//     -- if freq. constraints not met, we check for higher Vdds..
// -- We invoke mapping and routing only if PC constraint satisfied here -- 
//*****************************************************************************************************
// Possible other objectives to try-out --- minimization ----****** NEW *******
// 1) Based on Vt-profile
// 2) Based on IR-drop profile
// 3) Recent core-degradation -- NO -- for our model, this may not be applicable ---
// 4) minimum Vdd needed ?? -- always use the best (based on cost-func) block that is available at the current (lowest) Vdd, hike Vdd only if u have to, thus, Vdd would not figure in the cost-func.
	// ------- For now, lets not consider thermal effects on aging in our cost-function, as it may turn out to be too complicated -------------
// *********************************************** NEW *****************************************************
// -- Question: how do u account for performance loss due to IR-drops ? -- we keep track of the perc_max-IR-drop (seen by the grid-pt until now) in g1
//    -- so, calculate the WC (lowest) Vdd everytime we map any app and calculate the corresponding max_freq to check for min. freq. constraint 
//    -- may make sense to add a field called max_IR_drop in m so that it can be passed into the block_pacement func. --
// -- Even though we do not consider IR-drops and thermal-profiles for power calculations of cores, they are taken into account while calculating effects of core-aging
// *****************************************************************************************************
// -- Our Panacea framework with PDN-awareness, aging-awareness, and/or leakage-power awareness  -- with aging-aware routing (will also compare with Panacea:XYZ-routing) --
// -------------------------------------------------------------------------------------------------------
// ----- here, we are trying to reverse the Vt term in the cost func. to prioritize leakage reduction (at the same Vdd) instead of youth ------------

#include "My_Class.h"

double max_times_DoP(double max_time, double exec_time, double new_exec_time);
// -- Change the max_time constraints for the routing algorithm
double power_eval(grid_node*** m1, int indx_x, int indx_y, int indx_z, int dim_1, int dim_2, int dim_3, double Vdd_glob, int frequency, double stat_coeff, double dyn_coeff);
// -- find the tot_pow of the block under consideration --
// -- to get the approximate ideal execution time when the app_DoP is changed
double get_exec_time(int app_DoP, double exec_time, int new_DoP, int app_ID);
// -- to get the approximate ideal execution time when the freq is changed
double get_exec_time_freq(int freq, double exec_time, int new_freq);
// -- to get the overall execution times of the application with the overheads included
bool get_computation_overheads(int DoP, double compute_time, double deadline, int num_errors, double & total_runtime);
// Returns the num of estimated errors in a compute period
int get_soft_error_num(double Vdd_glob, double frequency, int DoP, double compute_time);
// return 1 if block found, else return 0
bool block_placement (const int mesh_dim_x, const int mesh_dim_y, const int num_tiers, grid_node*** m1, app_DoP_pair &app_element, double &power_slack, double &Vdd_glob) {
// find the best placement for the block for the given DoP -- with the given freq. constraint (in terms of Vt_map) for the app --
	// Add, the DoP and Vdd values allowed for the search

	vector<int>app_DoPs; vector<double>app_Vdds;
	app_DoPs.push_back(4);app_DoPs.push_back(8);app_DoPs.push_back(16);app_DoPs.push_back(32);
	app_Vdds.push_back(0.75); app_Vdds.push_back(0.8);app_Vdds.push_back(0.85);app_Vdds.push_back(0.9);app_Vdds.push_back(0.95);app_Vdds.push_back(1.0);
	// deduce the frequency from the mathematical model

	int frequency;  frequency = app_element.frequency;
	int i, j, k, c, c1; int c2=0;  int dim_1, dim_2, dim_3;	 int dim_x1, dim_y1, dim_z1;
	double Vdd_init;  Vdd_init = Vdd_glob;
	// -- tiles are chosen based on Vt values, and all core-power calculations are within this func...
	const double freq_pow = 1.2;		// ------- these coeffs in Vdd_reduce as well ----------
	//const double const_K = 0.0006; 
	double stat_coeff; stat_coeff = app_element.stat_coeff; //2000; (for 100) // 2600 (for 64 cores)
	double dyn_coeff; dyn_coeff = app_element.dyn_coeff; // 0.39 (for 100) // 0.33 (for 64 cores)
	cout << " app ID, stat and dyn coeff : " << app_element.app_ID << " " << app_element.stat_coeff << " " << app_element.dyn_coeff << endl;
	double Vt;
	
	double WC_Vt;	// -- highest tolerable IR_drop for nominal Vt at highest external Vdd
	double WC_IR_drop; // highest tolerable Vt for zero IR-drop at highest external Vdd
	// these need to be calculated offline for specific applications -- it is hard to isolate Vdd and Vt from the max_freq equation --
	WC_Vt = 0.5;  WC_IR_drop = 0.3; // absolute value now --- /*30.0;*/ // IR_drops are used as perc. values -- 
	// we assume some arbitrary values for now, change later ------
	double alpha = 0.3;  double beta = 0.7; // coeffs for cost_func.
	const double nom_Vt = 0.3;
	const double leak_pow_max = stat_coeff*exp(-3*nom_Vt);
	const double leak_pow_min = stat_coeff*exp(-3*WC_Vt)*0.75;


	// for each valid block found, sum up the Vt_avg -- and keep track of the block with the greatest Vt_avg-sum..
	double sum=0; double sum_vt = 0; double sum_times=0; double min_cost = 100000.0;  // Vt sum for the block
	double sum_pow;  bool Vdd_hike_flag = 1; // 1 by default, reset when Vdd hike is no longer allowed
	int indx_x = -1; int indx_y = -1;  int indx_z = -1;  // save the coords for the greatest sum --
	bool break_flag = 0;   int best_x = -1;   int best_y = -1;   int best_z = -1;   double max_freq, Vt_max, Vt_max_block;  double best_tot_pow = -1;
	double WC_Vdd; // or the estimated Vdd based on the WC-IR-drop seen by this core until now -- use this to calculate the max_freq of the core --


	// yash changes for CHARM framework -- heuristic.
	map<double, region> map_list_time;
	map<double, region> map_list_age;
	double ideal_comp_time = 0; double comp_time_oh = 0; double comp_time_f = 0;
	// Available DoPs for an app are based on the queue-pressure, so take that into consideration (How ?)
	// First sort the blocked regions into two lists. list_time and list_age
	// check DS_PB for every contender

	for (int d=0; d< app_DoPs.size(); d++){
		//app_element.DoP = app_DoPs[d];
		if (app_DoPs[d] == 4) { dim_1 = 1;  dim_2 = 1;  dim_3 = 0;  dim_x1=-1;  dim_y1=-1;  dim_z1=-1;  } // 2x2x1 block
		else if (app_DoPs[d] == 8) { dim_1 = 1; dim_2 = 3; dim_3 = 0; /* {4x2x1} OR {2x4x1} */ dim_x1=-1;  dim_y1=-1;  dim_z1=-1; } // 2x2x2 block
		else if (app_DoPs[d] == 16) { dim_1 = 7;  dim_2 = 1;  dim_3 = 0; /* {8x2x1} OR {2x8x1} */ dim_x1=3;  dim_y1=3;  dim_z1=0; } // 4x4x1 block
		else if (app_DoPs[d] == 32) { dim_1 = 3; dim_2 = 7; dim_3 = 0; /* {4x8x1} OR {8x4x1} */ dim_x1=-1;  dim_y1=-1;  dim_z1=-1; } // 4x4x2 block
		else { cout << " ERROR: block placement -wrong DoP " << app_element.DoP << endl; exit(1); }

		for (int v=0; v< app_Vdds.size(); v++){
			WC_Vdd = app_Vdds[v]; //(m1[i+k][j+c][c1+c2].max_IR_drop/100.0)*Vdd_glob;
			Vdd_glob = app_Vdds[v];
			for (i=0; i<mesh_dim_x; i++){
				for (j=0; j<mesh_dim_y; j++) {
					for (c1=0; c1<num_tiers; c1++){
						if (m1[i][j][c1].empty_flag == 1) {
							// 1st possible block
							// calculate the total of the cost function allowed blocks, and insert in the list
							//cout<<"blockplacement.cpp: checking region i, j, c1, dim_1, dim_2, dim_3"<<i<<" "<<j<<" "<<k<<" "<<endl;
							if (((i+dim_1) < mesh_dim_x) && ((j+dim_2) < mesh_dim_y) && ((c1+dim_3) < num_tiers)) {
								Vt_max_block =0;
								sum_vt = 0;  break_flag = 0;  sum_pow = 0;
								for (k=0; k<=dim_1; k++) {
									for (c=0; c<=dim_2; c++) {
										//cout<<dim_1<<" "<<dim_2<<" "<<dim_3<<" "<<endl;
										Vt_max =  max(m1[i+k][j+c][c1+c2].Vt_core, m1[i+k][j+c][c1+c2].Vt_router);
										Vt_max_block = max(Vt_max_block, Vt_max);
											if (m1[i+k][j+c][c1+c2].empty_flag == 1){
												double leak_pow = stat_coeff*exp(-3*m1[i+k][j+c][c1+c2].Vt_core)*WC_Vdd;
									//		    cout<<"block_placement_111: core_vt = "<<m1[i+k][j+c][c1+c2].Vt_core;
									//			cout<<" norm_vt = "<<((WC_Vt-m1[i+k][j+c][c1+c2].Vt_core)/(WC_Vt-nom_Vt));
									//			cout<<"leakage power of the core "<<leak_pow;
												//if(m1[i+k][j+c][c1+c2].Vt_core > 0.5) {cout<<"Found a failed core in this region, skipping it"<<endl; break_flag =1;}
												double norm_vt_degradation = (WC_Vt-m1[i+k][j+c][c1+c2].Vt_core)/(WC_Vt-nom_Vt);
												//cout<<"norm_vt_degradation = "<<norm_vt_degradation ;
												double norm_leak_power = ((leak_pow-leak_pow_min)/(leak_pow_max-leak_pow_min));
												//cout<<" norm_leak_power = "<<norm_leak_power<<endl;
												sum_vt = sum_vt + (alpha*norm_vt_degradation)+(beta*norm_leak_power);}
											else { break_flag = 1; cout<<"block_placement.cpp: skipping this mapping region "<<endl;  break; }
									} if (break_flag == 1) break;
								} // end for k
								//cout << " sum is " << sum << endl;
								// Calculate the approximate execution times and insert in the blocks.
								if(break_flag == 0){
								max_freq = (1000.0/((WC_Vdd/4.5)/(pow (WC_Vdd-Vt_max_block, freq_pow))))*0.7;
								if(max_freq < 5){cout<<"Low Freq error: Vdd, Vt degradation of this core "<<WC_Vdd<<" "<<Vt_max_block<<endl; exit(1);}
								ideal_comp_time = get_exec_time(app_element.DoP,app_element.compute_time, app_DoPs[d], app_element.app_ID);
								comp_time_f = get_exec_time_freq(app_element.frequency, ideal_comp_time, max_freq);
								//get the modified deadline for this new app_DoP and freq;
								//cout<<"block placement.cpp: chk deadline, start time: "<<app_element.deadline<<" "<<app_element.start_time<<endl;
								double app_deadline_constr = app_element.deadline - app_element.start_time;
								if(app_deadline_constr <= 0){cout<<"App "<<app_element.app_ID<<" missed its deadline. Going to next app"<<endl; break_flag =1;}
								// get number of errors
								int num_errors = get_soft_error_num(app_Vdds[v], max_freq, app_DoPs[d], comp_time_f);
								// We should also take into account the estimated errors in this compute_time
								break_flag = get_computation_overheads(app_DoPs[d], comp_time_f, app_deadline_constr, num_errors, comp_time_oh);
								}// if this compute_time_oh is missing the deadline, we should signal the break_flag
								if (break_flag == 0)
									{ sum_pow = power_eval(m1, i, j, c1, dim_1, dim_2, dim_3, app_Vdds[v], max_freq, stat_coeff, dyn_coeff);
										if (sum_pow > power_slack) {  break_flag = 1;
										cout<<"block placement.cpp: power_slack violated Vdd, Freq"<<app_Vdds[v]<<"/"<<max_freq<<" sum_pow "<<sum_pow<<" pow_slack "<<power_slack<<endl;}  // freq. met but PC not met, we will not hike Vdd
								}
								if ((break_flag == 0) && (!(sum_pow > power_slack))) {

									region new_region;
									cout<<"testing the regions selected "<<i<<" "<<j<<endl;
									new_region.ind_x = i; new_region.ind_y = j;  new_region.ind_z = c1;
									new_region.best_x = dim_1; new_region.best_y = dim_2; new_region.best_z = dim_3;
									new_region.compute_time = comp_time_oh;
									new_region.Vdd = WC_Vdd;Vdd_glob = WC_Vdd;
									new_region.frequency = max_freq;
									new_region.DoP = app_DoPs[d];
									new_region.best_tot_pow = sum_pow;
									// insert the region into both heaps
									 std::pair<std::map<double, region>::iterator,bool> ret;
									  ret = map_list_time.insert ( std::pair<double,region>(comp_time_oh,new_region) );
									  while (ret.second==false) {
									    //std::cout << "comp_time_oh already exists"<<endl;
									    comp_time_oh = comp_time_oh - 0.001;
									    //cout<<" comp_time of this region "<<comp_time_oh<<endl;
									    ret= map_list_time.insert ( std::pair<double,region>(comp_time_oh,new_region) );
									  }
									  ret = map_list_age.insert ( std::pair<double,region>(sum_vt,new_region) );
									  while (ret.second==false) {
									    //std::cout << "sum_vt already exists"<<endl;;
									    sum_vt= sum_vt- 0.00001;
									    //cout<<" Vt degradation in this region "<<sum_vt<<endl;
									    ret= map_list_age.insert ( std::pair<double,region>(sum_vt,new_region) );
									  }
									/*best_tot_pow = sum_pow;*/ }
								 cout<<"==================================================================="<<endl;
							} // end if
							// 2nd possible block
							//cout<<"blockplacement.cpp: checking region i, j, c1, dim_1, dim_2, dim_3"<<i<<" "<<j<<" "<<k<<" "<<endl;
							if ((dim_1 != dim_2) && ((i+dim_2) < mesh_dim_x) && ((j+dim_1) < mesh_dim_y) && ((c1+dim_3) < num_tiers)) {
								Vt_max_block =0;
								sum_vt = 0;  break_flag = 0;  sum_pow = 0;
								for (k=0; k<=dim_2; k++) {
									for (c=0; c<=dim_1; c++) {
										//cout<<"blockplacement.cpp: i, j, c1, dim_1, dim_2, dim_3"<<i<<" "<<j<<" "<<k<<" ";
										//cout<<dim_2<<" "<<dim_1<<" "<<dim_3<<" "<<endl;
										Vt_max =  max(m1[i+k][j+c][c1+c2].Vt_core, m1[i+k][j+c][c1+c2].Vt_router);
										Vt_max_block = max(Vt_max_block, Vt_max);
											if (m1[i+k][j+c][c1+c2].empty_flag == 1){
												double leak_pow = stat_coeff*exp(-3*m1[i+k][j+c][c1+c2].Vt_core)*WC_Vdd;
										//		cout<<"block_placement_111: norm_vt = "<<((WC_Vt-m1[i+k][j+c][c1+c2].Vt_core)/(WC_Vt-nom_Vt));
										//		cout<<"leakage power of the core "<<leak_pow;
										//		cout<<" norm_leak_power = "<<((leak_pow-leak_pow_min)/(leak_pow_max-leak_pow_min))<<endl;
												//if(m1[i+k][j+c][c1+c2].Vt_core > 0.5) {cout<<"Found a failed core in this region, skipping it"<<endl; break_flag =1;}
												double norm_vt_degradation = (WC_Vt-m1[i+k][j+c][c1+c2].Vt_core)/(WC_Vt-nom_Vt);
												//cout<<"norm_vt_degradation = "<<norm_vt_degradation ;
												double norm_leak_power = ((leak_pow-leak_pow_min)/(leak_pow_max-leak_pow_min));
												//cout<<" norm_leak_power = "<<norm_leak_power<<endl;
												sum_vt = sum_vt + (alpha*norm_vt_degradation)+(beta*norm_leak_power);}
											else { break_flag = 1; cout<<"block_placement.cpp: skipping this mapping region "<<endl;  break; }
									} if (break_flag == 1) break;
								} // end for k
								//cout << " sum is " << sum << endl;
								// Calculate the approximate execution times and insert in the blocks.
								if(break_flag == 0){
								WC_Vdd = app_Vdds[v]; //(m1[i+k][j+c][c1+c2].max_IR_drop/100.0)*Vdd_glob;
								max_freq = (1000/((WC_Vdd/4.5)/(pow (WC_Vdd-Vt_max_block, freq_pow))))*0.7;
								if(max_freq < 5){cout<<"Low Freq error: Vdd, Vt degradation of this core "<<WC_Vdd<<" "<<Vt_max_block<<endl; exit(1);}
								ideal_comp_time = get_exec_time(app_element.DoP,app_element.compute_time, app_DoPs[d], app_element.app_ID);
								comp_time_f = get_exec_time_freq(app_element.frequency, ideal_comp_time, max_freq);
								//get the modified deadline for this new app_DoP and freq;
								//cout<<"block placement.cpp: chk deadline, start time: "<<app_element.deadline<<" "<<app_element.start_time<<endl;
								double app_deadline_constr = app_element.deadline - app_element.start_time;
								if(app_deadline_constr <= 0){cout<<"App "<<app_element.app_ID<<" missed its deadline. Going to next app"<<endl; break_flag =1;}
								// get number of errors
								int num_errors = get_soft_error_num(app_Vdds[v], max_freq, app_DoPs[d], comp_time_f);
								// We should also take into account the estimated errors in this compute_time
								break_flag = get_computation_overheads(app_DoPs[d], comp_time_f, app_deadline_constr, num_errors, comp_time_oh);
								}// if this compute_time_oh is missing the deadline, we should signal the break_flag
								if (break_flag == 0)
									{ sum_pow = power_eval(m1, i, j, c1, dim_2, dim_1, dim_3, app_Vdds[v], max_freq, stat_coeff, dyn_coeff);
										if (sum_pow > power_slack) {  break_flag = 1;
										cout<<"block placement.cpp: power_slack violated Vdd, Freq"<<app_Vdds[v]<<"/"<<max_freq<<" sum_pow "<<sum_pow<<" pow_slack "<<power_slack<<endl;}  // freq. met but PC not met, we will not hike Vdd
								}
								if ((break_flag == 0) && (!(sum_pow > power_slack))) {

									region new_region;
									cout<<"testing the regions selected "<<i<<" "<<j<<endl;
									new_region.ind_x = i; new_region.ind_y = j;  new_region.ind_z = c1;
									new_region.best_x = dim_2; new_region.best_y = dim_1; new_region.best_z = dim_3;
									new_region.compute_time = comp_time_oh;
									new_region.Vdd = WC_Vdd; Vdd_glob = WC_Vdd;
									new_region.frequency = max_freq;
									new_region.DoP = app_DoPs[d];
									new_region.best_tot_pow = sum_pow;

									// insert the region into both heaps
									 std::pair<std::map<double, region>::iterator,bool> ret;
									  ret = map_list_time.insert ( std::pair<double,region>(comp_time_oh,new_region) );
									  while (ret.second==false) {
									   // std::cout << "comp_time_oh already exist"<<endl;
									    comp_time_oh= comp_time_oh- 0.001;
									    //cout<<" comp_time of this region "<<comp_time_oh<<endl;
									    ret= map_list_time.insert ( std::pair<double,region>(comp_time_oh,new_region) );
									  }
									  ret = map_list_age.insert ( std::pair<double,region>(sum_vt,new_region) );
									  while (ret.second==false) {
									  //  std::cout << "sum_vt already exist"<<endl;
									    sum_vt= sum_vt- 0.00001;
									    //cout<<" Vt degradation in this region "<<sum_vt<<endl;
									    ret= map_list_age.insert ( std::pair<double,region>(sum_vt,new_region) );
									  }
									/*best_tot_pow = sum_pow;*/ }
								 cout<<"==================================================================="<<endl;
							}
							// 3rd possible block
							//cout<<"blockplacement.cpp: checking region i, j, c1, dim_x1, dim_y1, dim_z1"<<i<<" "<<j<<" "<<k<<" "<<endl;
							if ((dim_x1 > -1) && ((i+dim_x1) < mesh_dim_x) && ((j+dim_y1) < mesh_dim_y) && ((c1+dim_z1) < num_tiers)) {
								Vt_max_block =0;
								sum_vt = 0;  break_flag = 0;  sum_pow = 0;
								for (k=0; k<=dim_x1; k++) {
									for (c=0; c<=dim_y1; c++) {
										Vt_max =  max(m1[i+k][j+c][c1+c2].Vt_core, m1[i+k][j+c][c1+c2].Vt_router);
										Vt_max_block = max(Vt_max_block, Vt_max);
											if (m1[i+k][j+c][c1+c2].empty_flag == 1){
												double leak_pow = stat_coeff*exp(-3*m1[i+k][j+c][c1+c2].Vt_core)*WC_Vdd;
											//	cout<<"block_placement_111: norm_vt = "<<((WC_Vt-m1[i+k][j+c][c1+c2].Vt_core)/(WC_Vt-nom_Vt));
											//	cout<<" norm_leak_power = "<<((leak_pow-leak_pow_min)/(leak_pow_max-leak_pow_min))<<endl;
												double norm_vt_degradation = (WC_Vt-m1[i+k][j+c][c1+c2].Vt_core)/(WC_Vt-nom_Vt);
												//cout<<"norm_vt_degradation = "<<norm_vt_degradation ;
												//if(m1[i+k][j+c][c1+c2].Vt_core > 0.5) {cout<<"Found a failed core in this region, skipping it"<<endl; break_flag =1;}
												double norm_leak_power = ((leak_pow-leak_pow_min)/(leak_pow_max-leak_pow_min));
												//cout<<" norm_leak_power = "<<norm_leak_power<<endl;
												sum_vt = sum_vt + (alpha*norm_vt_degradation)+(beta*norm_leak_power);}
											else { break_flag = 1; cout<<"block_placement.cpp: skipping this mapping region "<<endl;  break; }
									} if (break_flag == 1) break;
								} // end for k
								//cout << " sum is " << sum << endl;
								// Calculate the approximate execution times and insert in the blocks.
								if(break_flag == 0){
								WC_Vdd = app_Vdds[v]; //(m1[i+k][j+c][c1+c2].max_IR_drop/100.0)*Vdd_glob;
								max_freq = (1000/((WC_Vdd/4.5)/(pow (WC_Vdd-Vt_max_block, freq_pow))))*0.7;
								if(max_freq < 5){cout<<"Low Freq error: Vdd, Vt degradation of this core "<<WC_Vdd<<" "<<Vt_max_block<<endl; exit(1);}
								ideal_comp_time = get_exec_time(app_element.DoP,app_element.compute_time, app_DoPs[d], app_element.app_ID);
								comp_time_f = get_exec_time_freq(app_element.frequency, ideal_comp_time, max_freq);
								//get the modified deadline for this new app_DoP and freq;
								//cout<<"block placement.cpp: chk deadline, start time: "<<app_element.deadline<<" "<<app_element.start_time<<endl;
								double app_deadline_constr = app_element.deadline - app_element.start_time;
								if(app_deadline_constr <= 0){cout<<"App "<<app_element.app_ID<<" missed its deadline. Going to next app"<<endl; break_flag =1;}
								// get number of errors
								int num_errors = get_soft_error_num(app_Vdds[v], max_freq, app_DoPs[d], comp_time_f);
								// We should also take into account the estimated errors in this compute_time
								break_flag = get_computation_overheads(app_DoPs[d], comp_time_f, app_deadline_constr, num_errors, comp_time_oh);
								}// if this compute_time_oh is missing the deadline, we should signal the break_flag
								if (break_flag == 0)
									{ sum_pow = power_eval(m1, i, j, c1, dim_x1, dim_y1, dim_z1, app_Vdds[v], max_freq, stat_coeff, dyn_coeff);
										if (sum_pow > power_slack) {  break_flag = 1;
										cout<<"block placement.cpp: power_slack violated Vdd, Freq"<<app_Vdds[v]<<"/"<<max_freq<<" sum_pow "<<sum_pow<<" pow_slack "<<power_slack<<endl;}  // freq. met but PC not met, we will not hike Vdd
								}
								if ((break_flag == 0) && (!(sum_pow > power_slack))) {

									region new_region;
									cout<<"testing the regions selected "<<i<<" "<<j<<endl;
									new_region.ind_x = i; new_region.ind_y = j;  new_region.ind_z = c1;
									new_region.best_x = dim_x1; new_region.best_y = dim_y1; new_region.best_z = dim_z1;
									new_region.compute_time = comp_time_oh;
									new_region.Vdd = WC_Vdd;Vdd_glob = WC_Vdd;
									new_region.frequency = max_freq;
									new_region.DoP = app_DoPs[d];
									new_region.best_tot_pow = sum_pow;
									// insert the region into both heaps
									 std::pair<std::map<double, region>::iterator,bool> ret;
									  ret = map_list_time.insert ( std::pair<double,region>(comp_time_oh,new_region) );
									  while (ret.second==false) {
									   // std::cout << "comp_time_oh already exist"<<endl;
									    comp_time_oh= comp_time_oh- 0.001;
									    //cout<<" comp_time of this region "<<comp_time_oh<<endl;
									    ret=map_list_time.insert ( std::pair<double,region>(comp_time_oh,new_region) );
									  }
									  ret = map_list_age.insert ( std::pair<double,region>(sum_vt,new_region) );
									  while (ret.second==false) {
									   // std::cout << "sum_vt already exist"<<endl;
									    sum_vt= sum_vt- 0.00001;
									    //cout<<" Vt degradation in this region "<<sum_vt<<endl;
									    ret= map_list_age.insert ( std::pair<double,region>(sum_vt,new_region) );
									  }
									/*best_tot_pow = sum_pow;*/ }
								cout<<"==================================================================="<<endl;
							} // end if

						}
					}
				}
			}
		}
	}


	// if the CMP Vt profile is below the threshold, we select from the list_age
	// nom_vt =0.3, there are 60 cores. max_vt = 0.5 based on the lifetime, we should change this threshold
	// say when vt profile is 0.4*60 we can switch from list_time to list_age
	// Based on the app-slack time, we either select the top from list_time or list_age
	// if the slack_time is 0.75 times the compute_time, we choose from list_age, else we choose from list_time
	double chip_vt_profile =0;
	for (i=0; i<mesh_dim_x; i++) {
		for (j=0; j<mesh_dim_y; j++) {
			for (c1=0; c1<num_tiers; c1++) {
				chip_vt_profile += max(m1[i][j][c1].Vt_core,m1[i][j][c1].Vt_router);
			}
		}
	} // end for i
	std::map<double, region>::iterator it_a=map_list_age.begin();
	std::map<double, region>::iterator it_t=map_list_time.begin();
	cout<<"block placement: chip_vt_profile: "<< chip_vt_profile;
	if (chip_vt_profile > 22){
		// make sure that the list is not empty
/*		cout<<"Printing all the Vt profiles we analzed "<<endl;
		cout<<" Region 		Dimension 		Cost_function "<<endl;
		for (it_a=map_list_age.begin(); it_a != map_list_age.end(); it_a++ ){
			cout<< it_a->second.ind_x<<" "<<it_a->second.ind_y<<"  "<<it_a->second.DoP<<"  "<<it_a->second.Vdd<<" "<< it_a->first<<endl;
		}
		it_a=map_list_age.begin();*/
		cout<<" choosing from the list_age"<<endl;
		indx_x = it_a->second.ind_x; best_x = it_a->second.best_x;
		indx_y = it_a->second.ind_y; best_y = it_a->second.best_y;
		indx_z = it_a->second.ind_z; best_z = it_a->second.best_z;
		double max_time = app_element.run_time_constraint ;
		//Change the run time constraint of the app for routing purposes
		app_element.run_time_constraint = max_times_DoP(max_time, app_element.compute_time, it_a->second.compute_time);
		if(app_element.run_time_constraint > (app_element.deadline - app_element.start_time))app_element.run_time_constraint = (app_element.deadline - app_element.start_time);
		app_element.compute_time = it_a->second.compute_time; // Add the modified Compute time
		app_element.Vdd = it_a->second.Vdd;
		app_element.DoP = it_a->second.DoP;
		app_element.frequency = it_a->second.frequency;
		best_tot_pow = it_a->second.best_tot_pow;
	}
	else if((app_element.deadline - app_element.start_time) > app_element.compute_time*1.5)
	{
//		cout<<"Printing all the Vt profiles we analzed "<<endl;
//		cout<<" Region 	Dimension  Vdd Cost_function "<<endl;
//		for (it_a=map_list_age.begin(); it_a != map_list_age.end(); it_a++ ){
//			cout<< it_a->second.ind_x<<" "<<it_a->second.ind_y<<"  "<<it_a->second.DoP<<"  "<<it_a->second.Vdd<<" "<< it_a->first<<endl;
//		}
//		it_a=map_list_age.begin();
		cout<<" choosing from the list_age as there is enuf time"<<endl;
		indx_x = it_a->second.ind_x; best_x = it_a->second.best_x;
		indx_y = it_a->second.ind_y; best_y = it_a->second.best_y;
		indx_z = it_a->second.ind_z; best_z = it_a->second.best_z;
		double max_time = app_element.run_time_constraint ;
		//Change the run time constraint of the app for routing purposes
		app_element.run_time_constraint = max_times_DoP(max_time, app_element.compute_time, it_a->second.compute_time);
		if(app_element.run_time_constraint > (app_element.deadline - app_element.start_time))
			app_element.run_time_constraint = (app_element.deadline - app_element.start_time);

		app_element.compute_time = it_a->second.compute_time; // Add the modified Compute time
		app_element.Vdd = it_a->second.Vdd;
		app_element.DoP = it_a->second.DoP;
		app_element.frequency = it_a->second.frequency;
		cout<<" Chosen app frequency is "<<it_a->second.frequency<<endl;
		if(app_element.frequency < 0)  {cout<<"error with frequency selection "<< app_element.frequency<<endl; exit(1);}
		best_tot_pow = it_a->second.best_tot_pow;
	}
	else {
		cout<<" choosing from the list_time"<<endl;
		indx_x = it_t->second.ind_x; best_x = it_t->second.best_x;
		indx_y = it_t->second.ind_y; best_y = it_t->second.best_y;
		indx_z = it_t->second.ind_z; best_z = it_t->second.best_z;
		double max_time = app_element.run_time_constraint ;
		//Change the run time constraint of the app for routing purposes
		app_element.run_time_constraint = max_times_DoP(max_time, app_element.compute_time, it_t->second.compute_time);
		if(app_element.run_time_constraint > (app_element.deadline - app_element.start_time))
			app_element.run_time_constraint = (app_element.deadline - app_element.start_time);

		app_element.compute_time = it_t->second.compute_time; // Add the modified Compute time
		app_element.Vdd = it_t->second.Vdd;
		app_element.DoP = it_t->second.DoP;
		app_element.frequency = it_t->second.frequency;
		best_tot_pow = it_t->second.best_tot_pow;
	}
 // cout << " Vdd is : " << Vdd_glob << endl;
//cout << " min_cost is : " << min_cost << endl;
	if ((indx_x >= 0) && (indx_y >= 0) && (indx_z >= 0) && /*if both the lists are empty*/(map_list_age.size() >0 || map_list_time.size() >0))
	{
		for (k=0; k<=best_x; k++) { // map on m1 -- finalize -- 
			for (c=0; c<=best_y; c++) {
				for (c2=0; c2<=best_z; c2++) {
					if (m1[indx_x+k][indx_y+c][indx_z+c2].empty_flag == 0) {
						cout << " ERROR: block-placement " << endl; exit(1); }
						m1[indx_x+k][indx_y+c][indx_z+c2].app_ID = app_element.app_ID; // * this info need to be passed into this func. *****
						m1[indx_x+k][indx_y+c][indx_z+c2].app_seq_num = app_element.app_seq_num; 
						m1[indx_x+k][indx_y+c][indx_z+c2].empty_flag = 0;
						m1[indx_x+k][indx_y+c][indx_z+c2].frequency = app_element.frequency;
						m1[indx_x+k][indx_y+c][indx_z+c2].compute_time = app_element.compute_time; // This is the new compute time
						Vt = m1[indx_x+k][indx_y+c][indx_z+c2].Vt_core; 
						m1[indx_x+k][indx_y+c][indx_z+c2].leakage_pow = (stat_coeff*exp(-3*Vt)*app_element.Vdd); // core-powers to be used by the thermal tool --
						m1[indx_x+k][indx_y+c][indx_z+c2].dyn_pow = (((double) app_element.frequency)*(0.001*dyn_coeff*app_element.Vdd*app_element.Vdd));
						m1[indx_x+k][indx_y+c][indx_z+c2].current_Vdd = app_element.Vdd;
				}
			} 
		} // end for k
		cout<<"Block placement-111.cpp: selected DoP: "<<app_element.DoP<<" selected Vdd: "<<app_element.Vdd<<endl;
		cout<<"selected coordinates "<<indx_x<<" "<<indx_y<<" "<<indx_z<<endl;
		cout <<"selected dimensions "<<best_x+1<<" "<<best_y+1<<" "<<best_z+1<<endl;
		app_element.x_cord = indx_x;	app_element.y_cord = indx_y;	app_element.z_cord = indx_z;
		app_element.dim_x = best_x+1;	app_element.dim_y = best_y+1;	app_element.dim_z = best_z+1;
		// Add the Vdd of the cores
		// --------------------------------------------------------------------
		if ((best_tot_pow > power_slack) || (best_tot_pow < 0)) { cout << " ERROR: u got to be kidding me " << endl; exit(1); } 
		power_slack = power_slack - best_tot_pow;  cout << " best_tot_pow " << best_tot_pow << "  "; 
		cout << endl << " // pass back the updated power_slack --- block_plac. " << power_slack << endl;
		// ----------------------------------------------------------------------
		return 1; 
	}
	else { cout<<"Block placement: /* no valid mapping found !! -- try the next higher Vdd if it is allowed --*/"<<endl;return 0; }

} // end func -block_placement

double power_eval(grid_node*** m1, int indx_x, int indx_y,  int indx_z, int dim_1, int dim_2, int dim_3, double Vdd_glob, int frequency, double stat_coeff, double dyn_coeff) {
	double dyn_pow; 	
	double stat_pow;
	double tot_pow = 0; 
	double Vt;
	int i, j, k;

	for (i=indx_x; i<=indx_x+dim_1; i++) {
		for (j=indx_y; j<=indx_y+dim_2; j++) {
			for (k=indx_z; k<=indx_z+dim_3; k++) {
				Vt = m1[i][j][k].Vt_core; 
				stat_pow = stat_coeff*exp(-3*Vt)*Vdd_glob;
				dyn_pow = ((double) frequency)*(0.001*dyn_coeff*Vdd_glob*Vdd_glob);
				tot_pow = tot_pow + stat_pow + dyn_pow;
			}
		}
	}
	//tot_pow = tot_pow/1000.0;
	return tot_pow;

// Add the overhead calculation functions

} // end pow_eval
