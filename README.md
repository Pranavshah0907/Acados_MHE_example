# MHE example for thermal derating

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
