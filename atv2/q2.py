import math
import matplotlib.pyplot as plt

def forward_kinematics(a, q):
    """
    Cinemática direta de um braço robótico plano com n juntas rotacionais.
    
    a : lista de comprimentos dos elos [a1, a2, ..., an]
    q : lista de ângulos das juntas [q1, q2, ..., qn] em graus
    
    Retorna:
    - (x, y, psi) posição e orientação do end-effector
    - xs, ys: listas com as posições das juntas (para plot)
    """
    n = len(a)
    # converter ângulos para radianos
    q = [math.radians(angle) for angle in q]

    x, y = 0, 0
    theta = 0
    xs, ys = [0], [0]  # origem da base

    for i in range(n):
        theta += q[i]
        x += a[i] * math.cos(theta)
        y += a[i] * math.sin(theta)
        xs.append(x)
        ys.append(y)

    psi = math.degrees(theta)  # orientação final em graus
    return (x, y, psi, xs, ys)


# --------------------------
# Programa principal
n = 3   # número de juntas
a = []
q = []

print("=== Braço Robótico com 3 Juntas Rotacionais ===")
for i in range(n):
    a_i = float(input(f"Digite o comprimento do elo a{i+1}: "))
    q_i = float(input(f"Digite o ângulo da junta q{i+1} (em graus): "))
    a.append(a_i)
    q.append(q_i)

# Cinemática direta
x, y, psi, xs, ys = forward_kinematics(a, q)

# Resultados no console
print(f"\nEnd-effector:")
print(f"x = {x:.3f}, y = {y:.3f}, orientação = {psi:.2f}°")

# --------------------------
# Plotar o braço
plt.figure(figsize=(6,6))
plt.plot(xs, ys, '-o', color='blue', markersize=8, linewidth=4)  # elos e juntas
plt.plot(x, y, 'ro', markersize=10, label="End-effector")        # ponta

# Eixos
plt.axhline(0, color='gray', linewidth=0.5)
plt.axvline(0, color='gray', linewidth=0.5)
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.title("Braço Robótico com 3 Juntas Rotacionais")
plt.show()
