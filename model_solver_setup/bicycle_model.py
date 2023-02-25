from acados_template import *

def mpc_model():
    model_name = 'model'
    # set up states
    x1 = SX.sym('x1') #xpos
    x2 = SX.sym('x2') #ypos
    x3 = SX.sym('x3') #vehicle heading 
    x4 = SX.sym('x4') #steering angle
    x = vertcat(x1, x2, x3, x4)

    # set up controls
    u1 = SX.sym('u1') #speed
    u2 = SX.sym('u2') #steering angle dot
    u = vertcat(u1, u2)
    
    # xdot - do I need this?
    xdot1 = SX.sym('xdot1') #xpos_dot
    xdot2 = SX.sym('xdot2') #ypos_dot
    xdot3 = SX.sym('xdot3') #vehicle heading dot
    xdot4 = SX.sym('xdot4') #steering angle dot
    xdot = vertcat(xdot1, xdot2, xdot3, xdot4)
    
    l = 0.5 #front axle to COM
    L = 1.0 #wheel to wheel

    beta = atan2(l*tan(x4)/L)
    f_expl = vertcat(u1*cos(x3 + beta),
                    u1*sin(x3 + beta),
                    u1*tan(x4)*cos(beta)/L,
                    u2)

    f_impl = xdot - f_expl
    
    # algebraic variables
    z = []
    # parameters
    p = [] #can use parameters for more complex cost function

    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.u = u
    model.xdot = xdot
    model.z = z
    model.p = p
    model.name = model_name

    return model
