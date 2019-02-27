import sympy
sympy.init_printing(use_latex=True)

# 1. define sympy symbols
u_phi, phi, y_dot, y, dt = sympy.symbols(
'u_phi, phi, y_dot, y, dt')

# 2. define the state variable
x = sympy.Matrix([
    phi,
    y_dot,
    y])

# 3. define state transition function
g = sympy.Matrix([
    u_phi,
    y_dot - sympy.sin(phi) * dt,
    y + y_dot * dt
])

# 4. take jacobian of g with respect to x
print(g.jacobian(x))