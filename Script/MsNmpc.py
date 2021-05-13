# -*- coding: utf-8 -*-
"""
**********************************************************************
Robust multi stage NMPC with CasADi
Authors: Sankaranarayanan Subramanian, TU Dortmund
Date: JAN 2016

 Main file for the NMPC loop
 The structure of the code is the following:
	1. Initialize problem, model and tunning parameters
	2. Until end of the batch do
		2.1. Solve optimization problem and with x0 and get u
		3.1. Integrate the model using u and get next x0
	3. Plot results
	
**********************************************************************
"""
xpos = []
ypos = []
time1 =[]
start_time= time.time()
takeEstimateRobot1 = 0
takeEstimateRobot2 = 0
Estop = 0
fbtime = 0
fbtimeTotal = 0

# Dictionary of parameters
par_names = ("lin_vel")
par_dict = {"lin_vel": 0}

# Number of control intervals
nk = 20

open_loop = 0
# Numbr of finite elements per control interval
ni = 1
EKF_bypass = 1
# Robust horizon, set to 0 for standard NMPC
n_robust = 0
# Uncertain parameters
uncertain = ["lin_vel"]

temp_PF = 1

# Bang Bang behavour 1 is on 0 is for sin disturbance
Step_at_halfSimTime = 0


### Simulation Time
# End time
tf = nk * t_step
theta_err_real = 0.0
p_real = NP.array([theta_err_real])
# Use bias term (only for standard NMPC, robust horizon set to 0)
# bias=1
bias = 0
# If bias term is used include it as an extra parameter
# Number of realizations for the uncertain parameters
np_discretization = 3

# Choose if you want to plot the predictions at each sampling time and store them as .png in the folder /animations
activate_animation = 0
# Choose if free initial condition should be used
free_initial = 0
# State discretization scheme: 'multiple-shooting' or 'collocation'
state_discretization = 'collocation'
if state_discretization == 'collocation':

    # Degree of interpolating polynomials: 1 to 5
    deg = 1

    # Collocation points: 'legendre' or 'radau'
    coll = 'radau'
elif state_discretization == 'multiple-shooting':

    # Integrator class
    Integ = CVodesIntegrator


obj = 'minimal_time'

# Number uncertain parameters
np_uncertain = len(uncertain)

# -------------------------------------------------------------------------------------
# ---Define the ODE's of the model, states, control inputs, parameters and constants---
# -------------------------------------------------------------------------------------
execfile("model.py")

# -------------------------------------------------------------
# ----Set initial conditions, constraints and cost function----
# -------------------------------------------------------------

# Dimensions
nx_p = x_p.size()
nx = x_c.size()
nu = u.size()
np = p.size()

# Compose initial state
x0 = NP.array([x1_0, y1_0, theta1_0])


# Bounds for the states and initial guess
x1_lb = -1000 / 1000.0
y1_lb = -1000 / 1000.0
theta1_lb = -inf

x1_ub = 7000 / 1000.0
y1_ub = 7000 / 1000.0
theta1_ub = inf

x_lb = NP.array([x1_lb, y1_lb, -1e6])
x_ub = NP.array([x1_ub, y1_ub, 1e6])

x_init = deepcopy(x0)
x_lbf = deepcopy(x_lb)
x_ubf = deepcopy(x_ub)

# Bounds for, initial guess and scaling factors for the control inputs
vel1_init = 0
vel1_lb = -400 / 1000.0
vel1_ub = 400 / 1000.0
omega1_init = 0
omega1_lb = -pi / 3
omega1_ub = pi / 3

# Increase the lower bound for the feed in the beginning of the time horizon to prevent degenerated solution
global u_init0
u_lb = NP.array([vel1_lb, omega1_lb])
u_ub = NP.array([vel1_ub, omega1_ub])
u_init0 = NP.array([vel1_init, omega1_init])

epsilon = SX.sym("epsilon", 1)
epsilon_lbf = NP.array([-10.0])
epsilon_ubf = NP.array([10.0])
epsilon_init = NP.zeros(1)


x_pos1 = SX.sym("x_pos1")
y_pos1 = SX.sym("y_pos1")


#Add new symbols for multiple Robot Constraints

x_pos2 = SX.sym("x_pos2")
y_pos2 = SX.sym("y_pos2")

cons_lb = NP.array([0.15,0.15])#0.35
cons_ub = NP.array([1e6,1e6])

cons = vertcat([((x_pos - (x_pos1)) ** 2 + (y_pos - (y_pos1)) ** 2) + epsilon,
                ((x_pos - (x_pos2)) ** 2 + (y_pos - (y_pos2)) ** 2) + epsilon])
                # ((x_pos1 - (x_pos3)) ** 2 + (y_pos1 - (y_pos3)) ** 2) + epsilon])
# TODO: add another constraint for the 3rd AGV

nc_p = 2
cfcn = SXFunction([x, vertcat((x_pos1, y_pos1)), vertcat((x_pos2,y_pos2)),epsilon], [cons])  # ,x_pos3,y_pos3


noise_y = 0.1


def u_init(t):
    if t <= 9250:
        return u_init0
    else:
        return u_init0


# ODE right hand side function (use the format used by CasADi's integrators)
ffcn = SXFunction(daeIn(x=x, p=vertcat((u, p))), daeOut(ode=xdot))

# ----------------------------------------------------------
# ---------- Cost function for the MHE problem -------------
# ----------------------------------------------------------
# Objective function for the moving horizon estimator problem




# Symbol for changing references +bias 



if obj == 'minimal_time':
    mterm =  (x_pos - goal[0]) ** 2 +  (y_pos - goal[1]) ** 2 + 1e6 * epsilon ** 2
else:
    raise Exception("Unknown objective")

mfcn = SXFunction([x, u, epsilon], [mterm])

# Control movement penalty weighting matrix
u_prev = SX.sym("u_prev", nu)
du = u - u_prev
R = 1.0 * diag(SX([0, 0]))

dfcn = SXFunction([u_prev, u], [mul(du.T, mul(R, du))])

# Initialize functions
ffcn.init()
mfcn.init()
dfcn.init()
cfcn.init()
# Length of a control interval
hk = tf / nk

# Time grid
tgrid = NP.linspace(0, tf, nk + 1)

# -------------------------------------------------------------------------------------
# ----------------Define the integrator and the model for the simulator----------------
# -------------------------------------------------------------------------------------
# Substitute the real value of the uncertain parameters for the simulator

up = vertcat((u, lin_vel, ang_vel))
t0_sim = 0
tf_sim = t0_sim + t_step
# Create an integrator using the simulator model
f_sim = SXFunction(daeIn(x=x_p, p=u), daeOut(ode=xdot_p))
integrator = Integrator("cvodes", f_sim)
integrator.setOption("abstol", 1e-10)  # tolerance
integrator.setOption("reltol", 1e-10)  # tolerance
integrator.setOption("steps_per_checkpoint", 100)
integrator.setOption("t0", t0_sim)
integrator.setOption("tf", tf_sim)
integrator.setOption("fsens_abstol", 1e-8)
integrator.setOption("fsens_reltol", 1e-8)
integrator.setOption("exact_jacobian", True)
integrator.init()

# -------------------------------------------------------------------------------------
# ----------------delay----------------
# -------------------------------------------------------------------------------------
#
up_d = vertcat((u_d, p))
t0_d = 0
tf_d = tSensDelay
# Create an integrator using the simulator model
d_sim = SXFunction(daeIn(x=x_d, p=up_d), daeOut(ode=xdot_d))
d_integrator = Integrator("cvodes", d_sim)
d_integrator.setOption("abstol", 1e-10)  # tolerance
d_integrator.setOption("reltol", 1e-10)  # tolerance
d_integrator.setOption("steps_per_checkpoint", 100)
d_integrator.setOption("t0", t0_d)
d_integrator.setOption("tf", tf_d)
d_integrator.setOption("fsens_abstol", 1e-8)
d_integrator.setOption("fsens_reltol", 1e-8)
d_integrator.setOption("exact_jacobian", True)
d_integrator.init()


nk1 = 5  # Prediction horizon of obstacle
n_robust1 = 2  # robust horizon of obstacle
V_est = -150 / 1000.0
Omega_est = pi / 5
if n_robust1 > 0:
    p_scenario1 = [NP.array([V_est])]
    for p_name in uncertain:
        # Get the index of the uncertain parameter
        p_el = par_dict[p_name]

        # Create a new set of scenarios where the parameter has been set to its lower bound
        p_scenario1_lb = deepcopy(p_scenario1)
        for pp in p_scenario1_lb: pp[p_el] = p_lb[p_el]

        # Create a new set of scenarios where the parameter has been set to its upper bound
        p_scenario1_ub = deepcopy(p_scenario1)
        for pp in p_scenario1_ub: pp[p_el] = p_ub[p_el]

        # Append new scenarios to existing ones
        p_scenario1 += p_scenario1_lb
        p_scenario1 += p_scenario1_ub

n_branches1 = [len(p_scenario1) if k < n_robust1 else 1 for k in range(nk1)]

# Calculate the number of scenarios for x and u
n_scenarios1 = [len(p_scenario1) ** min(k, n_robust1) for k in range(nk1 + 1)]

# Scenaro tree structure
child_scenario1 = NP.resize(NP.array([-1], dtype=int), (nk1, n_scenarios1[-1], n_branches1[0]))
parent_scenario1 = NP.resize(NP.array([-1], dtype=int), (nk1 + 1, n_scenarios1[-1]))
branch_offset1 = NP.resize(NP.array([-1], dtype=int), (nk1, n_scenarios1[-1]))
for k in range(nk1):
    # Scenario counter
    scenario_counter = 0
    # For all scenarios
    for s in range(n_scenarios1[k]):
        # For all uncertainty realizations
        for b in range(n_branches1[k]):
            child_scenario1[k][s][b] = scenario_counter
            parent_scenario1[k + 1][scenario_counter] = s
            scenario_counter += 1

        # Store the range of branches
        if n_robust1 == 0:
            branch_offset1[k][s] = 0
        elif k < n_robust1:
            branch_offset1[k][s] = 0
        else:
            branch_offset1[k][s] = s % n_branches1[0]

z_obst = NP.resize(NP.array([], dtype=MX), (nk1 + 1, n_scenarios1[-1]))
#Add the number of extra obstacles here (more than one)
extra_obst= 1
pk_ext_init0 = 1.0 * NP.zeros(sum(n_scenarios1) * nc_p+extra_obst*2)

for i in range(nr_obst):
    pk_ext_init0[i*2] = xInt[i]
    pk_ext_init0[i*2+1] = yInt[i]

V_est = NP.resize(NP.array([], dtype=MX), (nk1 + 1, n_scenarios1[-1]))
z_obstacle = NP.resize(NP.array([], dtype=MX), (nk1 + 1, n_scenarios1[-1]))
V_estimate = NP.resize(NP.array([], dtype=MX), (nk1 + 1, n_scenarios1[-1]))

# -------------------------------------------------------------
# -------------------Set up the NLP----------------------------
# -------------------------------------------------------------
execfile("setupNLP.py")
# -------------------------------------------------------------
# ----------------------Set up the solver----------------------
# -------------------------------------------------------------
# Allocate an NLP solver
solver = NlpSolver("ipopt", nlp_fcn)

# Set options
if state_discretization == 'collocation':
    solver.setOption("expand", True)
solver.setOption("max_iter", 500)
solver.setOption("tol", 1e-6)
solver.setOption("linear_solver", "ma27")

# initialize the solver
solver.init()

# Initial condition
solver.setInput(vars_init, NLP_SOLVER_X0)

# Bounds on x
solver.setInput(vars_lb, NLP_SOLVER_LBX)
solver.setInput(vars_ub, NLP_SOLVER_UBX)

# Bounds on g
solver.setInput(lbg, NLP_SOLVER_LBG)
solver.setInput(ubg, NLP_SOLVER_UBG)

# NLP parameters
solver.setInput(vertcat((u_init0, pk_ext_init0)), NLP_SOLVER_P)

nx_ = nx_p
nx_x = nx_
# Initialize MPC information structures
mpc_states = NP.resize(NP.array([]), (int(end_time/t_step), nx_))
mpc_obstacles = NP.resize(NP.array([]), (int(end_time / t_step),nx_))
# measured_bounds = NP.resize(NP.array([]),(end_time/t_step,nx_*2))
x_est_iter = NP.resize(NP.array([]), (nx_x))

mpc_control = NP.resize(NP.array([]),(int(end_time / t_step), nu))

mpc_alg = NP.resize(NP.array([]),(int(end_time / t_step), 1))
mpc_time = NP.resize(NP.array([]),(int(end_time / t_step)))
mpc_cost = NP.resize(NP.array([]),(int(end_time / t_step)))

mpc_ref = NP.resize(NP.array([]),(int(end_time / t_step)))
mpc_cpu = NP.resize(NP.array([]),(int(end_time / t_step)))
result = NP.resize(NP.array([]), (nx, nx))
mpc_parameters = NP.resize(NP.array([]),(int(end_time / t_step), np))
mpc_parameters_true = NP.resize(NP.array([]),(int(end_time / t_step), np))
mpc_states[0, :] = x0[0:nx_]
mpc_control[0, :] = u_init0

mpc_time[0] = 0
mpc_parameters[0, :] = p_real
mpc_cost[0] = 0
### Real uncertain parameter values
p_real_system_values = NP.resize(NP.array([]),(int(end_time / t_step), np_uncertain))
p_real_system_values[0, :] = p_real

u_mpc = NP.resize(NP.array([]), (nu))
vPrev = deepcopy(u_mpc)
BIAS = NP.resize(NP.array([]), (3))
x0_sim = x_init
#plt.ion()
index_mpc = 1
x0_sim = x0[0:nx_]

x1_sim = deepcopy(x_init)
x1_sim[0] = xInt[0]  # 2150/1000.0
x1_sim[1] = yInt[0]  # 1720/1000.0

currentPos = deepcopy(x0_sim)

x1_sim[2] = thetaInit[0]
x2_sim = deepcopy(x_init)
x2_sim[0] = xInt[1]
x2_sim[1] = yInt[1]
x1_sim[2] = thetaInit[1]
v1 = 0.0 / 1000.0
omega1 = 0.0 / 6
p_est_current = p_real
mpc_states[0, :] = x0_sim
mpc_obstacles[0, :] = x1_sim
mpc_obstacle = mpc_obstacles


count_vio = 0


currenttime = time.time()


# -------------------------------------------------------------
# -----------------------Start NMPC loop-----------------------
# -------------------------------------------------------------
# The stopping criterion is that the total amount of polymer necessary
# for ending the batch has been already produced

while (index_mpc * t_step < end_time):
    
    
    vel1Plant = u_mpc[0]
    omega1Plant = u_mpc[1]
    xpos.append(float(cur_pos[0]))
    ypos.append(float(cur_pos[1]))
    time1.append(float(time.time()-start_time))

    vPrev = deepcopy(u_mpc)
    # Solve the problem
    solver.evaluate()
    # Print the optimal cost
    print "optimal cost: ", float(solver.getOutput("f"))
    print "NMPC iteration: ", index_mpc, "out of: ", end_time / t_step
    # Retrieve the solution
    v_opt = NP.array(solver.getOutput("x"))

    # Extract the control input that will be injected to the plant
    u_mpc = NP.squeeze(v_opt[U_offset[0][0]:U_offset[0][0] + nu])

    print "NMPC input: ", u_mpc

    cmd_msg1.linear.x=u_mpc[0]
    cmd_msg1.angular.z=u_mpc[1]
    cmd_pub1.publish(cmd_msg1)
    p_real = NP.array([theta_err_real])
    # Integrate the model xdot_sim, for a sampling time
    integrator.setInput(u_mpc, INTEGRATOR_P)
    integrator.setInput(x0_sim, INTEGRATOR_X0)
    integrator.evaluate()
    t0_sim = tf_sim
    tf_sim = t0_sim + t_step
    integrator.setOption("t0", t0_sim)
    integrator.setOption("tf", tf_sim)
    # Get the next initial condition for next iteration at time t_0+t_step
    x0_sim = NP.squeeze(integrator.getOutput())
    # Introduce some noise on the measurement of the concentration
    x0_sim_opt = deepcopy(x0_sim)
    integrator.evaluate()
    j = 0



    obst_pos = []
    robot1Pos = NP.array([float(cur_pos[0]), float(cur_pos[1]), float(theta1[0])])
    u_mpcRobot1 = NP.array([u_mpc[0], u_mpc[1]])
    for i in range(nr_obst):
        xx = float(cur_pos[i+2])
        yy = float(cur_pos[i+3])
        tata=float(theta1[i+1])
        obst_pos.append(NP.array([xx,yy,tata]))


    if (sqrt((robot1Pos[0] - goal[0]) ** 2 + (robot1Pos[1] - goal[1]) ** 2) < 10 / 1000.0):
        print index_mpc
        break

    xsim = []
    if sim==0:
        x0_sim=deepcopy(robot1Pos)
        for i in range(nr_obst):
            xsim.append(deepcopy(obst_pos[i]))


    print(x0_sim)

    vars_lb[X_offset[0, 0]:X_offset[0, 0] + nx] = x0_sim
    vars_ub[X_offset[0, 0]:X_offset[0, 0] + nx] = x0_sim
    temp_p = 0
    

    p_scenario1 = [0.0, 0.0]
    p_lb = NP.array([-0.1, -0.1])
    p_ub = NP.array([0.1, +0.1])
    # use this to have only lower and upper bounds in the scenario tree and not the nominal case

    if v1 < -0.4:
        v1 = -0.4
    elif v1 > 0.4:
        v1 = 0.4
    if n_robust1 > 0:
        p_scenario1 = [NP.array([0.0, 0.0])]
        for p_name in uncertain:
            # Get the index of the uncertain parameter
            p_el = par_dict[p_name]

            # Create a new set of scenarios where the parameter has been set to its lower bound
            p_scenario1_lb = deepcopy(p_scenario1)
            for pp in p_scenario1_lb: pp[p_el] = p_lb[p_el]

            # Create a new set of scenarios where the parameter has been set to its upper bound
            p_scenario1_ub = deepcopy(p_scenario1)
            for pp in p_scenario1_ub: pp[p_el] = p_ub[p_el]

            # Append new scenarios to existing ones
            p_scenario1 += p_scenario1_lb
            p_scenario1 += p_scenario1_ub
    temp_p = 0
    temp_x = 0
    p_scenario1[0] = NP.array([0, 0])
    p_scenario1[1] = NP.array([-0.05, -0.1])
    p_scenario1[2] = NP.array([0.05, 0.1])
    for k in range(nk1):

        # For all scenarios
        for s in range(n_scenarios1[k]):
            # For all uncertainty realizations
            if k == 0:
                x1_sim = xsim[0] 
                z_obst[k, s] = x1_sim[0:nx]
                z_obst[k,s+2] = x1_sim[0:nx]
                V_est[k, s] = NP.array([0, 0])
            for b in range(n_branches1[k]):
                integrator.setInput(V_est[k, s], INTEGRATOR_P)
                smallest = 100
                for a in range(nr_obst):
                    if ((cur_pos[0]-cur_pos[a+2])**2+(cur_pos[1]-cur_pos[a+3])**2)<smallest:
                        smallest = ((cur_pos[0]-cur_pos[a+2])**2+(cur_pos[1]-cur_pos[a+3])**2)
                        integrator_input = xsim[a]
                integrator.setInput(integrator_input[0:nx], INTEGRATOR_X0)
                integrator.evaluate()
                integrator.setOption("t0", 0)
                integrator.setOption("tf", t_step)
                # Get the next initial condition for next iteration at time t_0+t_step
                z_obst[k + 1, child_scenario1[k][s][b]] = NP.squeeze(integrator.getOutput())
                V_est[k + 1, child_scenario1[k][s][b]] = V_est[k, s] + p_scenario1[b + branch_offset1[k][s]]
                if (V_est[k + 1, child_scenario1[k][s][b]][0] < -0.4):
                    V_est[k + 1, child_scenario1[k][s][b]][0] = -0.4
                elif (V_est[k + 1, child_scenario1[k][s][b]][0] > 0.4):
                    V_est[k + 1, child_scenario1[k][s][b]][0] = 0.4
                pk_ext_init0[temp_p:temp_p + nc_p] = NP.squeeze(integrator.getOutput())[0:nc_p]
                temp_p += nc_p
              
    pk_ext_init0[0] = cur_pos[2]
    pk_ext_init0[1] = cur_pos[3]
    pk_ext_init0[2] = cur_pos[4]
    pk_ext_init0[3] = cur_pos[5]
    pk_ext_init0[4] = cur_pos[6]
    pk_ext_init0[5] = cur_pos[7]

    if ((x0_sim[0] - x1_sim[0]) ** 2 + (x0_sim[1] - x1_sim[1]) ** 2) < 0.1156:
        count_vio = count_vio + 1
    solver.setInput(vars_lb, NLP_SOLVER_LBX)
    solver.setInput(vars_ub, NLP_SOLVER_UBX)
    # Set the last control input as parameter for the penalty term
    # Add x_pos2_act and y_pos2_act here the actual positions of the obstacle. If more obstacles are added,
    solver.setInput(vertcat((NP.array([0, 0]), pk_ext_init0)), NLP_SOLVER_P)
    # Set current solution as initial guess for next iteration
    solver.setInput(v_opt, NLP_SOLVER_X0)
    solver.setInput(solver.getOutput(NLP_SOLVER_LAM_X), NLP_SOLVER_LAM_X0)
    solver.setInput(solver.getOutput(NLP_SOLVER_LAM_G), NLP_SOLVER_LAM_G0)
    # Store the necessary information for the final MPC plots
    mpc_states[index_mpc, :] = x0_sim
    mpc_obstacles[index_mpc, :] = xsim[1]
    mpc_control[index_mpc, :] = u_mpc
    mpc_time[index_mpc] = t0_sim
    mpc_parameters_true[index_mpc, :] = p_real
    mpc_cost[index_mpc] = 0
    z_obstacle = NP.concatenate((z_obstacle, z_obst), axis=0)
    V_estimate = NP.concatenate((V_estimate, V_est), axis=0)
    if index_mpc == 1:
        v_optimized = deepcopy(v_opt)
    else:
        v_optimized = NP.concatenate((v_optimized, v_opt), axis=1)
  
    aux = solver.getStats()
    mpc_cpu[index_mpc] = aux['t_mainloop']
    # Update counter

    index_mpc += 1
  

u_mpc[0] = 0
u_mpc[1] = 0

cmd_msg1.linear.x=u_mpc[0]
cmd_msg1.angular.z=u_mpc[1]
cmd_pub1.publish(cmd_msg1)
