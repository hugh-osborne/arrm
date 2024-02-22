/* File : sim.i */
/* arrm = afferent responding rehab model */
%module arrm 

%{
#include "../arrmlib/Sim.h"
%}

/* Let's just grab the original header file here */
%include "../arrmlib/Sim.h"

/* ATTENTION!!!!
When changes are made to arrmlib/Sim.h, run:
swig -c++ -python sim.i
Then in the generated arrm.py, add the following to the end of the file:

import sys
import os
import shutil
import inspect
from shutil import copyfile, copy2
def loadResources(arrmsim):
    filename = inspect.getframeinfo(inspect.currentframe()).filename
    path = os.path.dirname(os.path.abspath(filename))
	
    try:
        print('Copying simulation resource files to: ', os.getcwd()) 
        file_dir = path + '/Geometry'
        shutil.copytree(file_dir, os.getcwd() + '/Geometry')
        file_dir = os.path.join(path, 'MoBL_ARMS_module2_4_onemuscle_afferent.osim')
        shutil.copy2(file_dir, os.getcwd())
        file_dir = os.path.join(path, 'MoBL_ARMS_module2_4_onemuscle_afferent_no_string.osim')
        shutil.copy2(file_dir, os.getcwd())
        print('Success.')
    except:
        print("Simluation Resources already loaded.")
        
    arrmsim.setVisualiserDirectory(path)

*/