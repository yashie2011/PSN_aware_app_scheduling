#include <iostream>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <string>
#include <sstream>
#include <string.h>
#include <vector>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <math.h>
#include <queue>
#include <map>
#include <random>
#include <algorithm>
using namespace std;

// Define some constants
#ifndef CONSTANTS

#define DIM_X 10
#define DIM_Y 6
#define DIM_Z 1
#define VDD_MIN 0.8
#define TIME_STEP 5E-3  // 1ms
#define RANDOM_TIME_MUL 200
#define ARRIVAL_RATE_MUL 0.1 // second time scale
#define TOTAL_INCOMING_APPS 10
#define SNIPER_FREQ 2660E6 //Mhz
//TODO Change the SIM_MIN_FREQ value to the least it is going to be
#define SIM_MIN_FREQ 900E6 //MHZ this should be changed later
#define POWER_BUDGET 85.0
#define SNIPER_TRACE_INTERVAL 100E3   //cycles
#define VE_THRESH 6
#define MAX_THREAD_COUNT 32
#define ROUTER_DYN_POW_MUL 5.6E-2
#define ROUTER_LK_POW 1.05E-2
#define ARM_CORE_POW_REDUCTION 9
#endif

#define arraysize(ar) (sizeof(ar) / sizeof(ar[0]))

typedef map <double, vector <float> > power_trace;
// The below map saves <vdd and power trace>
typedef map <float, power_trace> pow_trace_vdd;
typedef map<short, pow_trace_vdd > app_pow_traces;

class tile_coord
{
	public:
		tile_coord (int x_c, int y_c, int z_c, bool mark) : x(x_c), y(y_c), z(z_c), marked(mark) {}
		tile_coord(): x(0), y(0),z(0), marked(0) {}
		int x;
		int y;
		int z;
		bool marked; // unmarked is 0
};


class region{
public:
	region(): coords(), tile_P(), Vdd(0), frequency(0), DoP(0), best_tot_pow(0){};
	vector <tile_coord> coords;
	vector <float> tile_P;
	double Vdd, frequency;
	int DoP;
	double best_tot_pow;
};



class node	// Accommodate per core V/F -- Yash	// node can be either a mapped or an unmapped core, when mapped, the co_ords are assigned
{
  public:
    node (int IDNum) : ID(IDNum),marked(false), in_v_connect(), in_v_volume(), out_v_connect(), out_v_volume(), out_v_fin_vol(), out_v_bw(), paths(),
	total_vol(0), thru(0), in_degree(0), out_degree(0), x_coord(0), y_coord(0), z_coord(0), compute_time(0), start_time(0) {}  //class constructor
    //default constructor
	int ID;
	bool marked;
	vector <int> in_v_connect; // vector of connecting nodes -- fanins
	vector <double> in_v_volume;  // vector of corresponding comm. volumes -- 
	vector <int> out_v_connect; // vector of connecting nodes -- fanouts
	vector <double> out_v_volume;  // vector of corresponding comm. volumes -- 
	vector <double> out_v_fin_vol; // vector of completed volumes of each comm.
	vector <double> out_v_bw;  // vector of corresponding bandwidths assigned to each comm volume
	vector <vector<int>> paths ; // Vector of paths assigned between tasks
	double total_vol;	// total communication volume for this node
	double thru;
	int in_degree;
	int out_degree;
	int x_coord;
	int y_coord;
	int z_coord;
	double compute_time; // in secs
	double start_time;  // in secs
}; // when mapping onto grid_node, map ID, app_ID, frequency, compute_time, and start_time..


class grid_node	// Accommodate per core V/F -- Yash	// -- this class retains the Vt values over the entire lifetime.. all other fields would need to be reset after the 5-day window !! ------
{ // separate Vts for core and the router within this tile ---- just one value for the entire core and just value for the router --- 
	// note, active times of routers will be relatively small, so to induce comparative (w.r.t. cores) aging in routers, we may need more traffic !! ----
	public:
		grid_node (int IDnum) : ID(IDnum) {}  // class constructor
		grid_node(): ID(0), app_ID(0), app_seq_num(0), compute_time(0), run_time(0), finish_time(0), Vt_core(0.3), Vt_router(0.3), frequency(1000), empty_flag(1),
			N_out_link(0), E_out_link(0), S_out_link(0), W_out_link(0), Up_out_link(0), Dn_out_link(0), N_out_BW(0), E_out_BW(0), W_out_BW(0), S_out_BW(0),  Up_out_BW(0), Dn_out_BW(0),
			N_out_comms(0), E_out_comms(0), W_out_comms(0), S_out_comms(0), U_out_comms(0), D_out_comms(0),
			N_out_thru(0), E_out_thru(0), W_out_thru(0), S_out_thru(0), U_out_thru(0), D_out_thru(0), power(0), r_power(0), r_mark(0), AT_max(0),
			N_out_AT(0), E_out_AT(0),  W_out_AT(0), S_out_AT(0), fir(0) {}	// Default Constructor
		int ID;		// same as the ID of the corresponding node
		int app_ID; // -- app_IDs 1 to 7 are SPLASH2 and app_IDs 8 to 14 are PARSEC -- IDs increasing with rated frequencies..
		int app_seq_num; // starting from 1 --
		double compute_time;
		double run_time; // compute_time + sum{flow_times} for the core --
		double finish_time; // -- its a time-point 't' -- not time-duration --
		double Vt_core;
		double Vt_router;
		int frequency;  // ***** assumed that each app. operates at its rated freq.
		bool empty_flag; // shows which tiles are being considered for mapping, not the tiles which are running apps -- finally, taken_flag is set for the mapped tiles --
		double N_out_link, E_out_link, S_out_link, W_out_link, Up_out_link, Dn_out_link; // the four out links for each tile.. these out links also correspond to the in links of their neighbors
		double N_out_BW, E_out_BW,  W_out_BW, S_out_BW, Up_out_BW, Dn_out_BW;	// Max. BW that the link can support (based on the operating freq. of the connecting cores)
		int N_out_comms, E_out_comms, W_out_comms, S_out_comms, U_out_comms, D_out_comms;
		double N_out_thru, E_out_thru, W_out_thru, S_out_thru, U_out_thru, D_out_thru;
		double power;
		double r_power; //router power
		bool r_mark;  // mark the router for throttling
		double AT_max;
		double N_out_AT, E_out_AT,  W_out_AT, S_out_AT, Up_out_AT, Dn_out_AT;
		//double max_IR_drop;  // this field is never reset just updated throughout lifetime --
		double current_Vdd; // I don't know if this can be useful. But, lets have it
		// lets change this to absolute value instead of perc_max_IR_drop, which gives pessimistic results for DVS scenario ...
		// vector <int> temperatures;
		vector <double> t_windows;  // time duration windows -- applies for both temperatures and Vdds vecs --
		vector <double> start_times; // -- applies to the t_windows vec --
		double fir; // flit injection rate
	// ------------------------------------------------------------------------------------------------------------------------
};

typedef grid_node* GridPtr;


//power_trace(): trace() {}
// <timestamp, power_trace_vector>



class app_DoP_pair   // there is no App-DoP pair this time -- Yash
{
	public:
		app_DoP_pair (int app_c, int DoP_c, int seq_num) : app_ID(app_c), DoP(DoP_c), app_seq_num(seq_num) {} 
		app_DoP_pair(): app_ID(0), DoP(0), app_seq_num(0), frequency(0), x_cord(-1), y_cord(-1), z_cord(-1), dim_x(0), dim_y(0), dim_z(0), run_time_constraint(0), start_time(-1), 
		                compute_time(0), comm_perc(0), map_time_vdd(0), app_pt(), map_region(), lastRecCycle(0), completed(false), total_vol(0){}
		int app_ID;
		int DoP;
		int app_seq_num;
		int frequency;
		int x_cord;
		int y_cord;
		int z_cord;
		int dim_x;
		int dim_y;
		int dim_z;
		double run_time_constraint;
		double start_time;
		double compute_time;
		double arrival_time; //Added for CHARM
		double deadline; // Added for CHARM framework
		double comm_perc;
		double map_time_vdd; // Time at which Vdd is mapped used to pull the power trace.
		app_pow_traces app_pt;  // To save the power traces of the application
		region map_region;
		double lastRecCycle;
		bool completed;
		double total_vol;
		node* tg_nodes[MAX_THREAD_COUNT];
};


class sdlv		// source - dest - volume triplets 
{
	public:
	sdlv (int source, int dest) : source_ID(source), dest_ID(dest) {}  // class constructor
	sdlv() : source_ID(0), dest_ID(0), volume(0), length(0), j(0) {} // default constructor
	int source_ID;
	int dest_ID;
	double volume;
	int length;
	int j;	// jth indx of out_connect
};

class path_vec
{
	public:
		vector <int> single_path;
};


