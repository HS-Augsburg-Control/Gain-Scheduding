# Gain-Scheduding
Academic Gain Scheduling Example

Requirements: Matlab 2020b or newer

In order to perform simulations, please run the script Run_AcademicExample.m which will call the simulink models

gainSchedExample_nonlinPlant_linearCntr.slx   - Simulate linear controllers on nonlinear plant
gainSchedExample_nonlinPlant_nonlinCntr.slx   - Gain-Scheduling Controllers with and without MRAW anti-windup

Per default, the standard parameter set is used but you can easily switch to the alternate parameters commenting

load cntrDesign_1.mat

and uncommenting 

load cntrDesign_2.mat

The simulations require that Matlab function "sat" provided in the Tools folder.


Design Script: Design_ControllerObserverAndAW.m 

This script calls LMI design functions contained in the tools subfolder. In order to run the script you'll have to download and install the interface Yalmip and the LMI solvers sdpt3 and SeDumi.

