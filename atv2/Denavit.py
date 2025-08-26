import math
import numpy as np
import matplotlib.pyplot as plt

def dh_matrix(theta, d, a, alpha):
    """Matriz de transformação homogênea DH (4x4)."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ]
    ])

def forward_kinematics_DH(params):
    """
    Cinemática direta usando parâmetros DH.
    
    params: lista de tuplas (theta, d, a, alpha)
            theta em graus (se junta rotacional), d/a/alpha em unidades apropriadas (alpha em graus).
    
    Retorna:
    - (x, y, z): posição do end-effector
    - pontos: lista das posições (x,y,z) de cada junta para plot
    """
    T = np.eye(4)
    pontos = [(0, 0, 0)]
    
    for (theta_deg, d, a, alpha_deg) in params:
        theta = math.radians(theta_deg)
        alpha = math.radians(alpha_deg)
        T_i = dh_matrix(theta, d, a, alpha)
        T = T @ T_i   # acumula transformações
        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        pontos.append((x, y, z))
    
    return (x, y, z), pontos

# -------- Exemplo: braço plano com 3 juntas (todos α=0, d=0) --------
print("=== Braço Robótico (DH) com 3 juntas ===")
a = []
q = []
params = []
n = 3
for i in range(n):
    a_i = float(input(f"Digite o comprimento do elo a{i+1}: "))
    q_i = float(input(f"Digite o ângulo da junta q{i+1} (em graus): "))
    a.append(a_i)
    q.append(q_i)
    params.append((q_i, 0, a_i, 0))  # (θ, d, a, α)

# Cinemática direta com DH
(x, y, z), pontos = forward_kinematics_DH(params)

print(f"\nEnd-effector:")
print(f"x = {x:.3f}, y = {y:.3f}, z = {z:.3f}")

# --------------------------
# Plotar (somente XY pois é plano)
xs = [p[0] for p in pontos]
ys = [p[1] for p in pontos]

plt.figure(figsize=(6,6))
plt.plot(xs, ys, '-o', color='blue', markersize=8, linewidth=4)
plt.plot(x, y, 'ro', markersize=10, label="End-effector")
plt.axhline(0, color='gray', linewidth=0.5)
plt.axvline(0, color='gray', linewidth=0.5)
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.title("Braço Robótico com 3 Juntas (DH)")
plt.show()
