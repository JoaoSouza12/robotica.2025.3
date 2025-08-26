# planner.py  -- Python 3
# Uso:
#  - editar as constantes abaixo (GRID_INPUT, START, GOAL, CELL_MM)
#  - rodar: python3 planner.py
#  - opção: digite "s" para apenas simular, "e" para exportar path.json para executor

import json
import math, heapq
import matplotlib.pyplot as plt

# ========== CONFIG ==========
# GRID_INPUT: 9 valores 0/1 em ordem de linhas (3x3)
GRID_INPUT = "0,0,0, 1,1,0, 0,0,0"
START = (0,0)
GOAL  = (2,2)
CELL_MM = 300   # distância por célula em mm

# ========== GRID / A* ==========
def parse_grid3x3(csv_9_vals):
    vals = [int(x.strip()) for x in csv_9_vals.replace(";", ",").split(",") if x.strip() != ""]
    if len(vals) != 9 or any(v not in (0,1) for v in vals):
        raise ValueError("GRID_INPUT deve ter 9 valores 0/1 (3x3).")
    return [vals[0:3], vals[3:6], vals[6:9]]

def in_bounds(rc):
    r,c = rc
    return 0 <= r < 3 and 0 <= c < 3

def passable(grid, rc):
    r,c = rc
    return grid[r][c] == 0

DIRS = [(-1,0),(1,0),(0,-1),(0,1)]
DIR_NAME = {(-1,0):'N', (1,0):'S', (0,-1):'W', (0,1):'E'}

def neighbors(grid, rc):
    r,c = rc
    for dr,dc in DIRS:
        nr,nc = r+dr, c+dc
        if in_bounds((nr,nc)) and passable(grid,(nr,nc)):
            yield (nr,nc)

def manhattan(a,b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def astar(grid, start, goal):
    open_heap = []
    g = {start:0}
    came = {}
    heapq.heappush(open_heap, (manhattan(start,goal), start))
    closed = set()
    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in closed: continue
        closed.add(current)
        if current == goal:
            path = [current]
            while current in came:
                current = came[current]
                path.append(current)
            path.reverse()
            return path
        for nb in neighbors(grid, current):
            tentative = g[current] + 1
            if tentative < g.get(nb, math.inf):
                g[nb] = tentative
                came[nb] = current
                f = tentative + manhattan(nb, goal)
                heapq.heappush(open_heap, (f, nb))
    return None

# ========== PLOT ==========
def plot_path(grid, path, start, goal, title="A* path (3x3)"):
    fig, ax = plt.subplots(figsize=(5,5))
    for r in range(3):
        for c in range(3):
            color = "black" if grid[r][c] == 1 else "white"
            ax.add_patch(plt.Rectangle((c, 2-r), 1, 1, facecolor=color, edgecolor="black"))
    if path:
        xs = [c+0.5 for (_,c) in path]
        ys = [2-r+0.5 for (r,_) in path]
        ax.plot(xs, ys, "o-", linewidth=2, markersize=8, label="path")
    # start/goal
    ax.text(start[1]+0.5, 2-start[0]+0.5, "S", color="green", ha="center", va="center", fontsize=14, fontweight="bold")
    ax.text(goal[1]+0.5, 2-goal[0]+0.5, "G", color="red", ha="center", va="center", fontsize=14, fontweight="bold")
    ax.set_xlim(0,3); ax.set_ylim(0,3)
    ax.set_xticks(range(4)); ax.set_yticks(range(4))
    ax.set_aspect("equal"); ax.grid(True); ax.legend()
    ax.set_title(title)
    plt.show()

# ========== HELPERS / EXPORT ==========
def path_to_dict(path):
    # transforma coords em lista simples
    return [{"r": int(r), "c": int(c)} for (r,c) in path] if path else []

def save_json_files(grid, start, goal, path, cell_mm):
    # salva grid.json e path.json
    grid_obj = {"grid": grid, "start": {"r":start[0], "c":start[1]}, "goal": {"r":goal[0], "c":goal[1]}, "cell_mm": cell_mm}
    with open("grid.json","w") as f:
        json.dump(grid_obj, f, indent=2)
    path_obj = {"path": path_to_dict(path), "start": {"r":start[0], "c":start[1]}, "goal":{"r":goal[0],"c":goal[1]}, "cell_mm": cell_mm}
    with open("path.json","w") as f:
        json.dump(path_obj, f, indent=2)
    print("Arquivos 'grid.json' e 'path.json' gravados. (executor_py2.py lê path.json)")

# ========== MAIN ==========
def main():
    grid = parse_grid3x3(GRID_INPUT)
    if not in_bounds(START) or not in_bounds(GOAL):
        raise ValueError("START/GOAL fora da grade 3x3.")
    if not passable(grid, START) or not passable(grid, GOAL):
        raise ValueError("START ou GOAL está bloqueado (valor 1).")

    path = astar(grid, START, GOAL)
    print("Caminho encontrado:", path)
    plot_path(grid, path, START, GOAL)

    choice = input("Digite 's' para só simular, 'e' para exportar path.json para o executor (Python2): ").strip().lower()
    if choice == 'e':
        save_json_files(grid, START, GOAL, path, CELL_MM)
    else:
        print("Encerrando (modo simulação).")

if __name__ == "__main__":
    main()
