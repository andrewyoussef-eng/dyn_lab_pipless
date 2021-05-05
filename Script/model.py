

lin_vel=0.0
ang_vel=0.0
p_init = NP.array([lin_vel])

lin_vel =  SX.sym("lin_vel")
p = vertcat([lin_vel])


#p_init = NP.array([w_Water_Feed1,w_Mono_Feed1,w_Water_Feed2,w_Starter_Feed2,
                  #m_dot_C,d,m_R,m_C,A_p,A_t,A_ini,E_A_p,E_A_t,E_A_ini,k_U1,
                  #k_U2,delta_H_R,T_F,k_bar])

# Assume correct value +/- 1 %                  
p_lb = NP.array([-0.02])
p_ub = NP.array([0.02])


# Create list of parameter realization
p_scenario = [p_init]

# use this to have only lower and upper bounds in the scenario tree and not the nominal case

if n_robust>0:
  p_scenario = [p_init]	
  for p_name in uncertain:
    # Get the index of the uncertain parameter
    p_el = par_dict[p_name]

    # Create a new set of scenarios where the parameter has been set to its lower bound
    p_scenario_lb = deepcopy(p_scenario)
    for pp in p_scenario_lb: pp[p_el] = p_lb[p_el]
    
    # Create a new set of scenarios where the parameter has been set to its upper bound
    p_scenario_ub = deepcopy(p_scenario)
    for pp in p_scenario_ub: pp[p_el] = p_ub[p_el]
    
    # Append new scenarios to existing ones
    p_scenario += p_scenario_lb
    p_scenario += p_scenario_ub


### Transfer of states to internal variables
x_pos=SX.sym("x_pos")
y_pos=SX.sym("y_pos")
theta=SX.sym("theta")
### --- Inputs ---
vel      = SX.sym("vel")
omega  = SX.sym("omega")


## --- Differential equations ---
dx_pos=vel*cos(theta)
dy_pos=vel*sin(theta)
dtheta=omega

### Definition of states and inputs

x_p = vertcat([x_pos,y_pos,theta])

u = vertcat([vel,omega])

xdot_p = vertcat([dx_pos,dy_pos,dtheta])

x_c=x_p
xdot_c=xdot_p

x=x_c
xdot=xdot_c

u_d=u
x_d=x_p
xdot_d=xdot_p

