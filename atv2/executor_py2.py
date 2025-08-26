# -*- coding: utf-8 -*-
# executor_py2.py  -- Python 2.7


import sys
import json
import atexit
from time import sleep

# ========== CONFIG ==========
GRID_FILE = "grid.json"
PATH_FILE = "path.json"
REPLAN_FILE = "replan_request.json"
CELL_MM_DEFAULT = 300
CHANNEL = 7
SPEED = 100
OBSTACLE_THRESHOLD = 0   # >0 considera obstáculo (ajuste se necessário)

# ========== MOWAY INIT (Python 2) ==========
sys.path.append("../lib/")
from moway_lib import *

atexit.register(exit_mow)

def init_moway():
    ret = init_moway(CHANNEL)
    if ret != 0:
        print "Moway RFUSB não conectado. Retorno:", ret
        sys.exit(1)
    print "Moway RFUSB Conectado."
    set_speed(SPEED)
    set_rotation_axis(CENTER)
# ========== IO ==========
def load_json(fname):
    with open(fname, "r") as f:
        return json.load(f)

def save_grid_json(grid_obj):
    with open(GRID_FILE, "w") as f:
        json.dump(grid_obj, f, indent=2)

def write_replan_request(current, blocked):
    payload = {"current": {"r": current[0], "c": current[1]}, "blocked": {"r": blocked[0], "c": blocked[1]}}
    with open(REPLAN_FILE, "w") as f:
        json.dump(payload, f, indent=2)
    print "Replan request written to", REPLAN_FILE

# ========== HELPERS ==========
def needed_turn_and_execute(curr_heading, move_vec):
    headings = {'N':0,'E':1,'S':2,'W':3}
    inv_head = {v:k for k,v in headings.items()}
    target_idx = {(-1,0):0, (0,1):1, (1,0):2, (0,-1):3}[move_vec]
    curr_idx = headings[curr_heading]
    diff = (target_idx - curr_idx) % 4
    if diff == 0:
        return curr_heading
    elif diff == 1:
        set_rotation(90)
        set_rotation_axis(CENTER)
        command_moway(CMD_ROTATERIGHT, 0)
        wait_mot_end(0)
        return inv_head[target_idx]
    elif diff == 2:
        set_rotation(180)
        set_rotation_axis(CENTER)
        command_moway(CMD_ROTATERIGHT, 0)
        wait_mot_end(0)
        return inv_head[target_idx]
    elif diff == 3:
        set_rotation(90)
        set_rotation_axis(CENTER)
        command_moway(CMD_ROTATELEFT, 0)
        wait_mot_end(0)
        return inv_head[target_idx]

def obstacle_ahead():
    cl = get_obs_center_left()
    cr = get_obs_center_right()
    print "Sensor values (center left, center right):", cl, cr
    return (cl + cr) > OBSTACLE_THRESHOLD

# ========== EXECUTION ==========
def execute_path_with_checks():
    # lê arquivos
    path_obj = load_json(PATH_FILE)
    grid_obj = load_json(GRID_FILE)

    path = [(p['r'], p['c']) for p in path_obj.get("path", [])]
    cell_mm = path_obj.get("cell_mm", CELL_MM_DEFAULT)
    if not path:
        print "Path vazio em", PATH_FILE
        return

    # inicia moway
    init_moway()

    heading = 'N'  # assumir norte inicial; ajuste se necessário
    pos = path[0]
    print "Iniciando execução. posição inicial:", pos, "heading:", heading

    for nxt in path[1:]:
        move = (nxt[0] - pos[0], nxt[1] - pos[1])
        # gira conforme necessário
        heading = needed_turn_and_execute(heading, move)

        # checa sensores antes de avançar
        if obstacle_ahead():
            print "Obstáculo detectado antes de avançar para", nxt
            # marca a célula nxt como bloqueada no grid_obj
            r, c = nxt
            if 0 <= r < 3 and 0 <= c < 3:
                grid_obj['grid'][r][c] = 1
                save_grid_json(grid_obj)
                write_replan_request(pos, nxt)
            print "Executor: encerrando para replanejamento."
            # parar o robô por segurança
            command_moway(CMD_STOP, 0)
            return

        # avança 1 célula
        set_distance(int(cell_mm))
        set_speed(SPEED)
        command_moway(CMD_GO, 0)
        wait_mot_end(0)
        pos = nxt
        print "Posição atual:", pos

    print "Caminho completado com sucesso!"
    # comando de parada final
    command_moway(CMD_STOP, 0)

if __name__ == "__main__":
    execute_path_with_checks()
