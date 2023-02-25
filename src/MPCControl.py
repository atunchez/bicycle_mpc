from acados_template import*
import numpy as np

class MPCControl:
    def __init__(self, model, acados_solver, ocp):
        self.model = model
        self.solver = acados_solver
        self.N = ocp.dims.N
        self.nx = ocp.dims.nx
        self.nu = ocp.dims.nu
        self.current_inputs = np.zeros((self.nu,))
        self.current_states = np.zeros((self.nx,))

    def calculateControl(self, current_state, xu_guess, ref_traj):
        self.solver.set(0,"x", current_state)
        self.solver.set(0, "ubx", current_state)
        self.solver.set(0, "lbx", current_state)

        setGuess(xu_guess)
        setRefTraj(des_traj)

        status = self.solver.solve()
        if status != 0:
            print("!!!! NO SOLUTION !!!!")
        
        return solver.get(0,"u")

    def shiftGuess(self, xu_guess):
        for j in range(self.N+1):
            if j < self.N:
                xu_guess[0:4,j] = self.solver.get(j+1,"x")
            else:
                xu_guess[0:4,j] = self.solver.get(j,"x")
            if j < self.N-1:
                xu_guess[4::,j] = self.solver.get(j+1,"u")
            elif j == self.N-1:
                xu_guess[4::,j] = self.solver.get(j,"u")
        return xu_guess

    def setGuess(self, xu_guess):
        for i in range(self.N + 1):
            self.solver.set(i, "x", xu_guess[0:4, i])
            if i < self.N:
                self.solver.set(i, "u", xu_guess[0:4, i])
    
    def setRefTraj(self, ref_traj):
        for i in range(self.N):
            self.solver.set(i, "y_ref", ref_traj[:, i])
        self.solver.set(self.N, "y_ref_e", ref_traj[:, self.N])
