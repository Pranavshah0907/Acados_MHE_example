# MHE new approach (Taking variance as control)

The folder contains the main MHE called **Main_script**. 

The models are set for both linear and nonlinear cost types.

The models are defined in the folder **05_model**. (OCP_MHE_model, OCP_MHE_S02 and OCP_MHE_S04)

The neural network is in the folder **02_parameter**

And the initial files with some variables required are in the **01Init** folder.

The folder also contains a pdf called **MHE_Problem_Formulation.pdf** which describes how the acados formulation is setup, The matrices for cost, reference etc.

The parameters are stored in the **pararmter_remake.mat**.


The problem is that the solver fails stat(4) after iteration 2.

The model can be directly simulated by pasting the folder alongside the installed acados folder.


--------------------------------------------------------------------------------------------------


# MHE_BlackBox 
example for thermal derating

The folder MHE_BlackBox contains all the files for the MHE being developed using a black box.
The main scripts are:
1. **MAIN_MHE_Linear_cost_working** : Working file for linear cost function, which complies and also gives decent results.
2. **MAIN_MHE_Nonlinear_cost** : Here the code is set for non-linear cost function but can also be changed to linear and the code functions properly.

The models are in the **05_model** folder.

After the models are compiled the main simulation can be run using the **Main_simulation_loop** file.

-----------------------------------------------------------------------------------------------------

# Acados_MHE_example

The folder contains the files for the Pendulum On Cart MHE estimation example using Acados.

The folder contains the following files:
1. **Pend_on_cart_example_ocp** : The main file for the model simulation.
  a. **export_pendulum_ode_model**: The function to generate model ode which is called in 1.
  b. **setup_sim**: The function to set up the ocp for simulations, also called in 1.

2. **Pend_on_cart_with_noisy_param_MHE** :  The main file for MHE estimation.
  a. **export_mhe_ode_model_with_noisy_param**: The function to create the model for MHE which is called in 2.
  b. **setup_estimator**: The function to setup the acados estimator, called in 2.
  
File sets for simulation and estimation (1 and 2) are independent of each other and the results generated from the simulation are stored and given as input for the MHE estimation.
The files can be run normally using acados by extracting them to a folder in the acados MATLAB example folder.  
