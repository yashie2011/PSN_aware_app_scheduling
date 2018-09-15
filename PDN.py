#!/usr/bin/python

# This code simulates the PDN on a mutlicore chip
# Inputs: Core current values.
# Outputs: Core node voltages into a file.

# 3rd party libraries
import ahkab
from ahkab import circuit, printing, time_functions
import argparse
import logging 
import matplotlib.pyplot as plt
import numpy as np
#constants
DEF_NODE_COUNT = 60
DEF_ROW_SIZE = 10
# takes Vdds and Iloads as inputs 
def PDN_circuit(args):
    logging.basicConfig(filename='PDN.log',filemode='w', level=logging.INFO)
    #args = parser.parse_args()
    if len(args.currents) == 0:
        raise ValueError("input currents should not be empty")
    
    mycircuit = circuit.Circuit(title="PDN circuit")
    gnd = mycircuit.get_ground_node()
    v_nodes = []
    c_nodes = []
    inter_nodes = []
    _NODE_COUNT = args.cores
    ROW_SIZE = args.rsize
    # declare Vdd nodes
    for i in range(_NODE_COUNT):
        v_nodes.append(mycircuit.create_node('nv'+str(i)))
        c_nodes.append(mycircuit.create_node('nc'+str(i)))
        inter_nodes.append(mycircuit.create_node('ni'+str(i)))
    # subcircuit for Metal layer parasitics (MLP) and cores
    # The values to the cores are obtained as command line inputs. 
    for i in range(_NODE_COUNT):
        mycircuit.add_resistor("Rb"+str(i), n1=v_nodes[i] , n2=inter_nodes[i] , value = 40e-3)
        mycircuit.add_inductor("Lb"+str(i), n1=inter_nodes[i] , n2=c_nodes[i] , value = 0.5e-11)
        mycircuit.add_capacitor("Cb"+str(i), n1=c_nodes[i] , n2=gnd , value = 1.6e-6)
        mycircuit.add_isource("Ib"+str(i), n1=c_nodes[i] , n2=gnd , dc_value = args.currents[i]) # 0.1e-3)

        # connection between cores 
        if (i+1)%ROW_SIZE != 0:
            if i%2 == 0:
                mycircuit.add_resistor("Rcc"+str(i), n1=c_nodes[i] ,n2=c_nodes[i+1] ,value = 50e-3)
        if (i+ROW_SIZE) < _NODE_COUNT:
            if (i/ROW_SIZE)%2 == 0:
                mycircuit.add_resistor("Rcc"+str(i+_NODE_COUNT), n1=c_nodes[i] ,n2=c_nodes[i+ROW_SIZE] ,value = 50e-3)
        
        mycircuit.add_vsource("V"+str(i), n1=v_nodes[i], n2=gnd, dc_value=args.voltages[i])
    
    # OP analysis
    op_analysis = ahkab.new_op()
    r = ahkab.run(mycircuit, an_list=[op_analysis])
    # print r['op'].results
    with open('vdd_out.log', 'w') as v_out:
        for i in range(_NODE_COUNT):
            gvdd = r['op'].results['vnv'+str(i)]
            cvdd = r['op'].results['vnc'+str(i)]
            if gvdd > 0:
                F_normal = float((gvdd-0.30)*(gvdd-0.30)/gvdd)
                F_dip = float((cvdd-0.30)*(cvdd-0.30)/cvdd)
                print ("Node %d : current: %f, Grid Vdd is %f, Core Vdd is %f, variation is %f percent" % (i, args.currents[i], gvdd , cvdd, 100*(gvdd-cvdd)/gvdd))
            v_out.write(str(cvdd)+' ')
    return r
        
# Builds arguments from the commandline 
def build_args():
    parser = argparse.ArgumentParser(description='CMP PDN simulator')
    parser.add_argument('--cores', help='Core count', type=int, default = DEF_NODE_COUNT)
    parser.add_argument('--rsize', help = 'No. of cores in each row', type = int, default = DEF_ROW_SIZE)
    parser.add_argument('currents',help='Currents drawn by each core in the order of their placement', nargs = DEF_NODE_COUNT, type=float)
    parser.add_argument('voltages', help='Node Voltages at each core', nargs = DEF_NODE_COUNT, type = float)
    return parser
    
if __name__ == "__main__":
    parser = build_args()
    args = parser.parse_args()
    PDN_circuit(args)
        

