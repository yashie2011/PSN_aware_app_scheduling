from subprocess import call
import sys
import os
import linecache

cores = 60
f = open("example2.flp", 'w')
for i in range(0,60):
	f.write("\n")
	temp = "Core"+str(i+1)+" :"+"\n"
	f.write(temp)
	f.write("\n")
	temp = "  position    "+str((i%10)*3000)+",    "+str((i/10)*3000)+" ;"
	f.write(temp)
	f.write("\n")
	f.write("  dimension   3000, 3000 ;")
	f.write("\n")
	f.write("\n")
	f.write("  power values 2;")
	f.write("\n")
f.close()	

