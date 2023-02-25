import sys
  
# adding model/solver to the system path

sys.path.insert(0, '../model_solver_setup')
from model_solver_setup.bicycle_model import mpc_model
from model_solver_setup.solver import mpc_solver
from MPCControl import MPCControl

model = mpc_model()
solver, ocp = mpc_solver(model)
controller = MPCControl(model, solver, ocp)

#controller is built now do stuff with it

'''
while calculating_control:
    state estimate -> get current state and inputs
    guess -> state and input guesses
    set ref traj -> 

    calculated control at current state = controller.calculateControl(current state, xu_guess, ref_traj)
    update guess with solution --> shiftGuess

'''
