
## Collocation discretization
if state_discretization == 'collocation':

  # Choose collocation points
  if coll=='legendre':    # Legendre collocation points
    tau_root = (
      [0,0.500000],
      [0,0.211325,0.788675],
      [0,0.112702,0.500000,0.887298],
      [0,0.069432,0.330009,0.669991,0.930568],
      [0,0.046910,0.230765,0.500000,0.769235,0.953090])[deg-1]
  elif coll=='radau':     # Radau collocation points
    tau_root = (
      [0,1.000000],
      [0,0.333333,1.000000],
      [0,0.155051,0.644949,1.000000],
      [0,0.088588,0.409467,0.787659,1.000000],
      [0,0.057104,0.276843,0.583590,0.860240,1.000000])[deg-1]
  else:
    raise Exception('Unknown collocation scheme')
    
  # Size of the finite elements
  h = hk/ni

  # Coefficients of the collocation equation
  C_mpc = NP.zeros((deg+1,deg+1))

  # Coefficients of the continuity equation
  D = NP.zeros(deg+1)

  # Dimensionless time inside one control interval
  tau = SX.sym("tau")
    
  # All collocation time points
  T_mpc = NP.zeros((nk,ni,deg+1))
  for k in range(nk):
    for i in range(ni):
      for j in range(deg+1):
        T_mpc[k,i,j] = h*(k*ni + i + tau_root[j])

  # For all collocation points
  for j in range(deg+1):
    # Construct Lagrange polynomials to get the polynomial basis at the collocation point
    L = 1
    for r in range(deg+1):
      if r != j:
        L *= (tau-tau_root[r])/(tau_root[j]-tau_root[r])
    lfcn = SXFunction([tau],[L])
    lfcn.init()
    
    # Evaluate the polynomial at the final time to get the coefficients of the continuity equation
    lfcn.setInput(1.0)
    lfcn.evaluate()
    D[j] = lfcn.getOutput()
    tfcn = lfcn.tangent()
    tfcn.init()
    # Evaluate the time derivative of the polynomial at all collocation points to get the coefficients of the continuity equation
    for r in range(deg+1):
     tfcn.setInput(tau_root[r])
     tfcn.evaluate()
     C_mpc[j,r] = tfcn.getOutput()
  # Initial condition
  xk0 = MX.sym("xk0",nx)

  # Parameter
  pk = MX.sym("pk",np)
  
  # Control
  uk = MX.sym("uk",nu)
  #uk_prev = MX.sym ("uk_prev",nu)
  #Also add variables here that is consistent with the previous version
  pk_ext=MX.sym("pk_ext",(sum(n_scenarios1)*nc_p+nu+extra_obst*2))
  # State trajectory
  n_ik = ni*(deg+1)*nx
  ik = MX.sym("ik",n_ik)
  ik_split = NP.resize(NP.array([],dtype=MX),(ni,deg+1))

  # All variables with bounds and initial guess
  ik_lb = NP.zeros(n_ik)
  ik_ub = NP.zeros(n_ik)
  ik_init = NP.zeros(n_ik)
  offset = 0

  # Store initial condition
  ik_split[0,0] = xk0
  first_j = 1 # Skip allocating x for the first collocation point for the first finite element
  

  # For each finite element
  for i in range(ni):
    # For each collocation point
    for j in range(first_j,deg+1):
      # Get the expression for the state vector
      ik_split[i,j] = ik[offset:offset+nx]
      
      # Add the initial condition
      ik_init[offset:offset+nx] = x_init
      
      # Add bounds
      ik_lb[offset:offset+nx] = x_lb
      ik_ub[offset:offset+nx] = x_ub
      offset += nx
    
    # All collocation points in subsequent finite elements
    first_j = 0

  # Get the state at the end of the control interval
  xkf = ik[offset:offset+nx]
  
  # Add the initial condition
  ik_init[offset:offset+nx] = x_init
  
  # Add bounds
  ik_lb[offset:offset+nx] = x_lb
  ik_ub[offset:offset+nx] = x_ub
  offset += nx
  
  # Check offset for consistency
  assert(offset == n_ik)  
    
  # Constraints in the control interval
  gk = []
  lbgk = []
  ubgk = []

  # For all finite elements
  for i in range(ni):
  
    # For all collocation points
    for j in range(1,deg+1):
          
      # Get an expression for the state derivative at the collocation point
      xp_ij = 0
      for r in range (deg+1):
        xp_ij += C_mpc[r,j]*ik_split[i,r]
        
      # Add collocation equations to the NLPj
      f_ij = ffcn.call(daeIn(x=ik_split[i,j],p=vertcat((uk,pk))))[DAE_ODE]
      gk.append(h*f_ij - xp_ij)
      lbgk.append(NP.zeros(nx)) # equality constraints
      ubgk.append(NP.zeros(nx)) # equality constraints

    # Get an expression for the state at the end of the finite element
    xf_i = 0
    for r in range(deg+1):
      xf_i += D[r]*ik_split[i,r]

    # Add continuity equation to NLP
    x_next = ik_split[i+1,0] if i+1<ni else xkf
    gk.append(x_next - xf_i)
    lbgk.append(NP.zeros(nx))
    ubgk.append(NP.zeros(nx))
    
  # Concatenate constraints
  gk = vertcat(gk)
  lbgk = NP.concatenate(lbgk)
  ubgk = NP.concatenate(ubgk)
  assert(gk.size()==ik.size())
  
  # Create the integrator function
  ifcn = MXFunction([ik,xk0,pk,uk],[gk,xkf])
  ifcn.init()

elif state_discretization == 'multiple-shooting':
  
  # Create an integrator instance
  ifcn = Integ(ffcn)
  
  # Set options
  ifcn.setOption("tf",hk)
  
  # Set CVodesIntegrator specific options
  if Integ==CVodesIntegrator:
    ifcn.setOption("reltol",1e-8)
    ifcn.setOption("abstol",1e-8)
  
  # Set CollocationIntegrator specific options
  if Integ==CollocationIntegrator:
    ifcn.setOption("implicit_solver",KinsolSolver)
    ifcn.setOption("startup_integrator",CVodesIntegrator)
    ifcn.setOption("expand_f",True)
    ifcn.setOption("collocation_scheme","radau")
    ifcn.setOption("number_of_finite_elements",5)
    ifcn.setOption("interpolation_order",3)
  
  # Initialize the integrator
  ifcn.init()
  
  # No implicitly defined variables
  n_ik = 0
  
# Number of branches
n_branches = [len(p_scenario) if k<n_robust else 1 for k in range(nk)]
    
# Calculate the number of scenarios for x and u
n_scenarios = [len(p_scenario)**min(k,n_robust) for k in range(nk+1)]

# Scenaro tree structure
child_scenario = NP.resize(NP.array([-1],dtype=int),(nk,n_scenarios[-1],n_branches[0]))
parent_scenario = NP.resize(NP.array([-1],dtype=int),(nk+1,n_scenarios[-1]))
branch_offset = NP.resize(NP.array([-1],dtype=int),(nk,n_scenarios[-1]))
for k in range(nk):
  # Scenario counter
  scenario_counter = 0
  # For all scenarios
  for s in range(n_scenarios[k]):
    # For all uncertainty realizations
    for b in range(n_branches[k]):
      child_scenario[k][s][b] = scenario_counter
      parent_scenario[k+1][scenario_counter] = s
      scenario_counter += 1
    
    # Store the range of branches
    if n_robust==0:
      branch_offset[k][s] = 0
    elif k<n_robust:
      branch_offset[k][s] = 0
    else:
      branch_offset[k][s] = s % n_branches[0]
    
# Count the total number of variables
NV = len(p_scenario)*np
for k in range(nk):
  NV += n_scenarios[k]*(nu + nx + n_branches[k]*n_ik)
NV += n_scenarios[nk]*nx # End point
NV +=1
# Weighting factor for every scenario
omega = [1./n_scenarios[k+1] for k in range(nk)]
omega_delta_u = [1./n_scenarios[k+1] for k in range(nk)]
#omega_delta_u[0] =1./n_scenarios[0+1]

# NLP variable vector
V = MX.sym("V",NV)
  
# All variables with bounds and initial guess
vars_lb = NP.zeros(NV)
vars_ub = NP.zeros(NV)
vars_init = NP.zeros(NV)
offset = 0

# Get parameters
P = NP.resize(NP.array([],dtype=MX),(len(p_scenario)))
for b in range(len(p_scenario)):
  P[b] = V[offset:offset+np]
  vars_lb[offset:offset+np] = p_scenario[b]
  vars_ub[offset:offset+np] = p_scenario[b]
  offset += np

# Get collocated states and parametrized control
X = NP.resize(NP.array([],dtype=MX),(nk+1,n_scenarios[-1]))
if state_discretization=='collocation':
  I = NP.resize(NP.array([],dtype=MX),(nk,n_scenarios[-1],n_branches[0]))
U = NP.resize(NP.array([],dtype=MX),(nk,n_scenarios[-1]))
X_offset = NP.resize(NP.array([-1],dtype=int),X.shape)
U_offset = NP.resize(NP.array([-1],dtype=int),U.shape)
for k in range(nk):
  # For all scenarios
  for s in range(n_scenarios[k]):
    # Get the expression for the state vector
    X[k,s] = V[offset:offset+nx]
    X_offset[k,s] = offset

    # Add the initial condition
    vars_init[offset:offset+nx] = x_init
    
    # Add bounds. Free initial condition for the kite problem
    #if free_initial == 1:
	#	aux_free_initial = 0
	#else:
	#	aux_free_initial = 0
    if k==0:
      vars_lb[offset:offset+nx] = x0
      vars_ub[offset:offset+nx] = x0
      #vars_lb[offset+nx-1:offset+nx] = x0[3]
      #vars_ub[offset+nx-1:offset+nx] = x0[3]
    else:
      vars_lb[offset:offset+nx] = x_lb
      vars_ub[offset:offset+nx] = x_ub
    offset += nx
      
    # State trajectory if collocation
    if state_discretization=='collocation':

      # For all uncertainty realizations
      for b in range(n_branches[k]):
        # Get an expression for the implicitly defined variables
        I[k,s,b] = V[offset:offset+n_ik]
            
        # Add the initial condition and bounds
        vars_init[offset:offset+n_ik] = ik_init
        vars_lb[offset:offset+n_ik] = ik_lb
        vars_ub[offset:offset+n_ik] = ik_ub
        offset += n_ik
          
    # Parametrized controls
    U[k,s] = V[offset:offset+nu]
    U_offset[k,s] = offset
    vars_lb[offset:offset+nu] = u_lb
    vars_ub[offset:offset+nu] = u_ub
    vars_init[offset:offset+nu] = u_init(tgrid[k])
    offset += nu
  
# State at end time (for all x scenarios)
for s in range(n_scenarios[nk]):
  X[nk,s] = V[offset:offset+nx]
  X_offset[nk,s] = offset
  vars_lb[offset:offset+nx] = x_lbf
  vars_ub[offset:offset+nx] = x_ubf
  vars_init[offset:offset+nx] = x_init
  offset += nx
EPSILON = NP.resize(NP.array([],dtype=MX),(1)) 
EPSILON = V[offset:offset+1]
E_offset = offset
vars_lb[offset:offset+1] = epsilon_lbf
vars_ub[offset:offset+1] = epsilon_ubf
vars_init[offset:offset+1] = epsilon_init
offset +=1
# Check offset for consistency
assert(offset == NV)

# Constraint function for the NLP
g = []
lbg = []
ubg = []

# Objective function in the NLP
J = 0
temp_p=6
# For all control intervals
for k in range(nk):
  # For all scenarios
  for s in range(n_scenarios[k]):
    
    # Initial state and control 
    X_ks = X[k,s]
    U_ks = U[k,s]
    
    # For all uncertainty realizations
    for b in range(n_branches[k]):
      
      # Parameter realization
      P_ksb = pk_ext[temp_p:temp_p+np]
      if state_discretization=='collocation':
        
        # Call the inlined integrator
        [g_ksb,xf_ksb] = ifcn.call([I[k,s,b],X_ks,P_ksb,U_ks])

        # Add equations defining the implicitly defined variables (i.e. collocation and continuity equations) to the NLP
        g.append(g_ksb)
        lbg.append(NP.zeros(n_ik)) # equality constraints
        ubg.append(NP.zeros(n_ik)) # equality constraints
      
      elif state_discretization == 'multiple-shooting':
        
        # Call the integrator
        ifcn_out = ifcn.call(integratorIn(x0=X_ks,p=U_ks))
        xf_ksb = ifcn_out[INTEGRATOR_XF]
      #[xf_ksb]=EKF_fcn.call([xf_ksb1,U_ks,P_ksb])
      
      # Add continuity equation to NLP
      g.append(X[k+1,child_scenario[k][s][b]] - xf_ksb)
      lbg.append(NP.zeros(nx))
      ubg.append(NP.zeros(nx))
      
      if k>=1 and k<nk1 and b==0:
       for temp_i in range(n_scenarios1[k]/n_scenarios[k]):
        Pksb = pk_ext[nu+temp_p:nu+temp_p+nc_p]
        temp_p=temp_p+nc_p
        [residual] = cfcn.call([X_ks,Pksb,pk_ext[-extra_obst*2:],EPSILON])  # TODO: line gives error
        g.append(residual)
        lbg.append(cons_lb)
        ubg.append(cons_ub)
      
      # Add extra constraints depending on other states
     
	  # Add contribution to the cost
      #[J_ksb] = mfcn.call([xf_ksb,U_ks,P_ksb,EPSILON])
      # Pass a different BIAS term for each realization of the uncertainty
      if k>0:
          [J_ksb] = mfcn.call([xf_ksb,U_ks,EPSILON])
          J += omega[k]*J_ksb
          # Penalize deviations in u
      s_parent = parent_scenario[k][s]
      u_prev = U[k-1,s_parent] if k>0 else pk_ext[0:nu]
      [du_k] = dfcn.call([u_prev,U[k,s]])
      J += omega_delta_u[k]*n_branches[k]*du_k

if open_loop==1:
 for k in range(nk):
  # For all scenarios
  for s in range(n_scenarios[k]-1):
   if k>n_robust:
      g.append(U[k,s+1] - U[k,s])
      lbg.append(NP.zeros(nu))
      ubg.append(NP.zeros(nu))
# Concatenate constraints
g = vertcat(g)
lbg = NP.concatenate(lbg)
ubg = NP.concatenate(ubg)
  
nlp_fcn = MXFunction(nlpIn(x=V,p=pk_ext),nlpOut(f=J,g=g))
