# -*- coding: utf-8 -*-
# A* com simulação visual (grid 3x3) + execução no mOway (opcional)

import matplotlib.pyplot as plt
import math, heapq

# ========= CONFIG =========
GRID_INPUT = "0,0,0, 0,1,0, 0,0,0"   # 9 valores 0/1 (linhas da grade, 3x3)
START = (0, 0)                       # posição inicial (linha, coluna)
GOAL = (2, 2)                        # destino (linha, coluna)
CELL_MM = 300                        # 30 cm por célula

# ========= GRID =========
def parse_grid3x3(csv_9_vals):
    vals = [int(x.strip()) for x in csv_9_vals.replace(";", ",").split(",") if x.strip() != ""]
    if len(vals) != 9 or any(v not in (0,1) for v in vals):
        raise ValueError("GRID_INPUT deve ter 9 valores 0/1 (3x3).")
    return [vals[0:3], vals[3:6], vals[6:9]]

def in_bounds(rc): 
    r, c = rc
    return 0 <= r < 3 and 0 <= c < 3

def passable(grid, rc):
    r, c = rc
    return grid[r][c] == 0

DIRS = [(-1,0),(1,0),(0,-1),(0,1)]

def neighbors(grid, rc):
    r, c = rc
    for dr, dc in DIRS:
        nr, nc = r+dr, c+dc
        if in_bounds((nr,nc)) and passable(grid, (nr,nc)):
            yield (nr,nc)

def manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def astar(grid, start, goal):
    open_heap = []
    g = {start: 0}
    came = {}
    heapq.heappush(open_heap, (manhattan(start, goal), start))
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

# ========= PLOT =========
def plot_path(grid, path, start, goal, return_path=None):
    fig, ax = plt.subplots(figsize=(5,5))

    # desenhar células
    for r in range(3):
        for c in range(3):
            if grid[r][c] == 1:
                ax.add_patch(plt.Rectangle((c, 2-r), 1, 1, facecolor="black"))  # obstáculo
            else:
                ax.add_patch(plt.Rectangle((c, 2-r), 1, 1, facecolor="white", edgecolor="black"))

    # desenhar caminho de ida
    if path:
        xs, ys = zip(*[(c+0.5, 2-r+0.5) for r,c in path])
        ax.plot(xs, ys, "o-", color="blue", label="Ida")

    # desenhar caminho de volta (se existir)
    if return_path:
        xs, ys = zip(*[(c+0.5, 2-r+0.5) for r,c in return_path])
        ax.plot(xs, ys, "o--", color="red", label="Volta")

    # marcar start e goal
    ax.text(start[1]+0.5, 2-start[0]+0.5, "S", color="green",
            ha="center", va="center", fontsize=14, fontweight="bold")
    ax.text(goal[1]+0.5, 2-goal[0]+0.5, "G", color="red",
            ha="center", va="center", fontsize=14, fontweight="bold")

    ax.set_xlim(0,3)
    ax.set_ylim(0,3)
    ax.set_xticks(range(4))
    ax.set_yticks(range(4))
    ax.set_aspect("equal")
    ax.grid(True)
    ax.legend()
    plt.show()

# ========= MAIN =========
if __name__ == "__main__":
    grid = parse_grid3x3(GRID_INPUT)
    path = astar(grid, START, GOAL)
    return_path = astar(grid, GOAL, START)  # caminho de volta

    print("Caminho de ida:", path)
    print("Caminho de volta:", return_path)

    plot_path(grid, path, START, GOAL, return_path)
