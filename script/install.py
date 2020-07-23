#!/usr/bin/env python

import os


path = os.path.dirname(os.path.abspath(__file__))
print(path)
# a = input("Do you want to install the optimal navigation toolkit with FORCES PRO solver? Press y/n")
# if(a=="y"):
# 	os.environ["FORCES"]= "true"
# a = input("Do you want to install the optimal navigation toolkit with ACADO toolkit solver? Press y/n")
# if(a=="y"):

os.environ["ACADO"] = "true"
os.system('git submodule init')
os.system('git submodule update')
os.chdir(path)
os.chdir("..")
os.chdir("acado")
if(not os.path.isdir('build')):
	os.mkdir("build")
os.chdir('build')
os.system('cmake ..')
os.system('make')

if(os.path.isdir('/usr/local/share/acado')):
	cmd = 'sudo rm -r /usr/local/share/acado'
	

os.system('sudo make install')

cmd = 'catkin build'
os.system(cmd)


