// This code snippet models the soft errors on a chip

#include "Constants.h"

double soft_error_rate(double Vdd_glob, double frequency ){
	double normalized_VF, scale_fac, SER, SER_min;
	SER_min = 0.5; int d= 3;
	normalized_VF = (double) ((((double)frequency/1950.0) + (Vdd_glob))/2.0); //cout << " normalized_VF " << normalized_VF << endl;
	scale_fac = (double) (d*((1.0 - normalized_VF)/(0.33)));  // [1 - {((1300/1900)+(0.8/1.1))/2}] = 0.295 -- change if Vdd or freq. ranges are changed -- **********
	//cout<<"scaling factor = "<<scale_fac<<endl;
	SER = SER_min*(pow(10.0,scale_fac)); // errors/sec/compute-core   -- This can be changed while running the simulations
	//cout << "The SER rate is "<<SER<<" errors/sec/core"<<endl;
	return SER;
}

// When app-mapping, just get the # of errors, estimate the re-execution time if it crosses the deadline, leave it.
int get_soft_error_num(double Vdd_glob, double frequency, int DoP, double compute_time){
	double SER;
	cout<< "Soft Errors- Vdd_glob: "<<Vdd_glob<<" frequency: "<<frequency<<" DoP: "<<DoP;
	SER= soft_error_rate(Vdd_glob, frequency);
	double time2;
	if(compute_time< 1000) time2 = 10*(pow(10.0, -3)*compute_time);
	else {time2 =  10*(pow(10.0, -3)*compute_time);}
	double temp = (double)(-1* SER * time2);
	//cout <<"temp = "<<temp<<endl;
	double exp_func = exp((double)temp);
	//cout <<"err prob1 = "<<(1-exp_func)<<endl;
	double est_err_1 = (double) time2*(1- exp_func)*DoP;
	//cout <<"estimated error for time1 "<< est_err_1;
	cout<<" estimated error for time2 "<< time2<<"==============> "<< est_err_1<<endl;
	// if the estimated errors < 0.09 make them 0, else take the ceil of the estimated errors.
	if(est_err_1 < 0.09) { est_err_1 = 0; return(est_err_1);}
	else {return((double)ceil(est_err_1));}
}

