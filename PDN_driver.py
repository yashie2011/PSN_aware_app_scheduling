import random
import PDN
import subprocess

# What do we want to do?? We want to know the thresholds at which the Vdd values of a particular core in a region  
# get effected by surrounding activity. We measure activity as the current drawn by cores when they are working
# If we can step the power drawn by each core from 1.5W to 7W and see how the variation effects the Vdd drop on a core of study
# PROG_NAME = ['sudo', 'python', 'PDN.py']
ROW_SIZE = PDN.DEF_ROW_SIZE

if __name__ == "__main__":
    # create a list of 60 random current values between X and Y
    currents = []
    voltages = []
    vdd_begin = 0.9
    vdd_end = 1.1
    initial_pow_sel = random.uniform(1.2, 1.6)
    pow_normal = 1.6

    pow_abnormal = 4.6
    for i in range (PDN.DEF_NODE_COUNT):
        vdd = random.uniform(vdd_begin, vdd_end)
        rand_pow = random.uniform(initial_pow_sel, 4.9)
        print rand_pow
        rand_curr = rand_pow/vdd
        #currents.append(str(rand_curr))
        if i == 4 or i == 6:
            currents.append(str(pow_abnormal/vdd_begin))
        else:
            currents.append(str(pow_normal/vdd_begin))
        voltages.append(str(vdd_begin))
    parser = PDN.build_args()
    args = parser.parse_args(currents + voltages) #+ ['--cores', str(PDN.DEF_NODE_COUNT), '--rsize', str(ROW_SIZE)])
    res = PDN.PDN_circuit(args)
    print res['op'].results
    #cmd = PROG_NAME + currents
    #cmd = cmd + ['--cores', str(4), '--rsize', str(2)]
    #print cmd
    #subprocess.call(cmd)
    #call the function along with the current values as inputs to the function     
