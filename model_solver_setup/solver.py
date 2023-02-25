#!/usr/bin/env python3
from acados_template import*
import numpy as np
import scipy.linalg

def mpc_solver(model):
    N = 10
    Tf = 1
    Ts = Tf/N
    Q = np.array([10, 10, 10, 10]) ## 
    R = np.array([1, 1])

    ocp = AcadosOcp()
    ocp.model = model
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu

    #ocp dimensions
    ocp.dims.N = N
    ocp.dims.nx = nx
    ocp.dims.nbx = nx
    ocp.dims.nbu = nu
    ocp.dims.nbx_e = nx
    ocp.dims.nu = model.u.size()[0]
    
    #state and input
    x = ocp.model.x
    u = ocp.model.u

    #set cost type
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'

    #set cost reference vector
    ocp.model.cost_y_expr = vertcat(x,u)
    ocp.model.cost_y_expr_e = x

    #set cost diagonal matrix
    ocp.cost.W = scipy.linalg.block_diag(Q, R)
    ocp.cost.W_e = Q
    #possibly add y_expr_0, W_0???

    #set bounds
    lim_ = 100
    ocp.constraints.ubx = lim_*np.ones((4,))
    ocp.constraints.lbx = -lim_*np.ones((4,))
    ocp.constraints.ubx_0 = lim_*np.ones((4,))
    ocp.constraints.lbx_0 = -lim_*np.ones((4,))
    ocp.constraints.ubx_e = lim_*np.ones((4,))
    ocp.constraints.lbx_e = -lim_*np.ones((4,))
    ocp.constraints.ubu = np.array([100, 180])
    ocp.constraints.lbu = np.array([-100,-180])
    ocp.constraints.idxbx = np.array([0,1,2,3])
    ocp.constraints.idxbx_0 = np.array([0,1,2,3])
    ocp.constraints.idxbx_e = np.array([0,1,2,3])
    ocp.constraints.idxbu = np.array([0,1])

         # set QP solver
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_max_iter = 500


    # set prediction horizon
    ocp.solver_options.tf = Tf
    # ocp.solver_options.nlp_solver_type = 'SQP'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'

    acados_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp.json')
    print('MPC built')
    return acados_solver,ocp
