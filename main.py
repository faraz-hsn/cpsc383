from aegis_game.stub import *
import heapq

def chebyshev_distance(a, b):
    dx = abs(a.x - b.x)
    dy = abs(a.y - b.y)
    return dx if dx > dy else dy

def get_all_directions():
    return [
        Direction.NORTH, Direction.NORTHEAST, Direction.EAST, Direction.SOUTHEAST,
        Direction.SOUTH, Direction.SOUTHWEST, Direction.WEST, Direction.NORTHWEST
    ]

def simple_astar(start, goal):
    if start.x == goal.x and start.y == goal.y:
        return []
    open_set = []
    heapq.heappush(open_set, (0, 0, start))
    came_from = {}
    g = {(start.x, start.y): 0}
    c = 0
    while open_set:
        _, _, cur = heapq.heappop(open_set)
        ck = (cur.x, cur.y)
        if cur.x == goal.x and cur.y == goal.y:
            path = []
            while ck in came_from:
                prev, d = came_from[ck]
                path.append(d)
                ck = (prev.x, prev.y)
            path.reverse()
            return path
        for d in get_all_directions():
            try:
                nxt = cur.add(d)
                if not on_map(nxt):
                    continue
                cell = get_cell_info_at(nxt)
                if cell.is_killer_cell():
                    continue
                mv = int(cell.move_cost)
                if mv <= 0:
                    mv = 1
            except:
                continue
            nk = (nxt.x, nxt.y)
            cand = g[ck] + mv
            if (nk not in g) or (cand < g[nk]):
                came_from[nk] = (cur, d)
                g[nk] = cand
                f = cand + chebyshev_distance(nxt, goal)
                c = c + 1
                heapq.heappush(open_set, (f, c, nxt))
    return None

def estimate_path_cost(start, path):
    if not path:
        return 0
    total = 0
    cur = start
    for d in path:
        try:
            nxt = cur.add(d)
            cell = get_cell_info_at(nxt)
            mv = int(cell.move_cost)
            if mv <= 0:
                mv = 1
            total = total + mv
            cur = nxt
        except:
            total = total + 1
    return total

class AgentState:
    def __init__(self):
        self.my_id = None
        self.current_path = []
        self.current_target = None
        self.last_location = None
        self.stuck_count = 0
        self.known_saved = set()
        self.need_charging = False
        self.charging_now = False
        self.claims = {}  # (x,y)->agent_id

STATE = AgentState()

def pick_best_target_and_path(loc, energy, survivors, round_num):
    candidates = []
    for s in survivors:
        if (s.x, s.y) in STATE.known_saved:
            continue
        claim_owner = STATE.claims.get((s.x, s.y))
        if claim_owner is not None and claim_owner != STATE.my_id:
            continue
        path = simple_astar(loc, s)
        if path is None:
            continue
        cost = estimate_path_cost(loc, path)
        candidates.append((cost, chebyshev_distance(loc, s), s, path))
    if not candidates:
        return None, None, None
    candidates.sort(key=lambda t: (t[0], t[1]))
    best = candidates[0]
    # claim it so others skip
    STATE.claims[(best[2].x, best[2].y)] = STATE.my_id
    send_message("CLAIM|" + str(best[2].x) + "|" + str(best[2].y) + "|" + str(STATE.my_id), [])
    return best[2], best[3], best[0]

def think():
    global STATE
    try:
        t = get_round_number()
        loc = get_location()
        energy = get_energy_level()

        if STATE.my_id is None:
            STATE.my_id = get_id()
            move(Direction.CENTER)
            return

        # read claim messages
        try:
            msgs = read_messages()
            for m in msgs:
                p = []
                try:
                    p = m.message.split("|")
                except:
                    p = []
                if len(p) >= 4 and p[0] == "CLAIM":
                    x = int(p[1]); y = int(p[2]); ag = int(p[3])
                    STATE.claims[(x, y)] = ag
        except:
            pass

        try:
            survivors = get_survs()
        except:
            survivors = []

        # save if on survivor
        try:
            cell_here = get_cell_info_at(loc)
            top_here = cell_here.top_layer
            if isinstance(top_here, Survivor):
                save()
                STATE.known_saved.add((loc.x, loc.y))
                STATE.current_target = None
                STATE.current_path = []
                STATE.need_charging = False
                return
        except:
            pass

        # dig if on rubble
        try:
            cell_here = get_cell_info_at(loc)
            top_here = cell_here.top_layer
            if isinstance(top_here, Rubble):
                req = top_here.agents_required
                if req == 1:
                    dig()
                    return
                else:
                    count_here = 1
                    try:
                        count_here = len(cell_here.agents)
                    except:
                        count_here = 1
                    if count_here >= 2:
                        dig()
                        return
                    move(Direction.CENTER)
                    return
        except:
            pass

        # charging tile behavior — dynamic energy goal
        try:
            cell_here = get_cell_info_at(loc)
            if "CHARGING" in str(cell_here.type).upper():
                goal_cost = 0
                if STATE.current_path:
                    goal_cost = estimate_path_cost(loc, STATE.current_path)
                needed = goal_cost + 10
                if energy < needed:
                    recharge()
                    return
                STATE.charging_now = False
                STATE.need_charging = False
                STATE.current_path = []
        except:
            pass

        if not survivors:
            move(Direction.CENTER)
            return

        # stuck detection
        if STATE.last_location is not None:
            if (STATE.last_location.x == loc.x) and (STATE.last_location.y == loc.y):
                STATE.stuck_count = STATE.stuck_count + 1
                if STATE.stuck_count > 3:
                    STATE.current_path = []
                    STATE.current_target = None
                    STATE.need_charging = False
                    STATE.stuck_count = 0
            else:
                STATE.stuck_count = 0
        STATE.last_location = loc

        if STATE.current_target is None or STATE.need_charging:
            s, path, cost = pick_best_target_and_path(loc, energy, survivors, t)
            if s is None:
                move(Direction.CENTER)
                return
            STATE.current_target = s
            STATE.current_path = path

        if not STATE.current_path:
            s, path, cost = pick_best_target_and_path(loc, energy, survivors, t)
            if s is None:
                move(Direction.CENTER)
                return
            STATE.current_target = s
            STATE.current_path = path

        # energy-aware reroute
        if STATE.current_path:
            path_cost = estimate_path_cost(loc, STATE.current_path)
            if path_cost >= energy - 5:
                chargers = []
                try:
                    chargers = get_charging_cells()
                except:
                    chargers = []
                best = None
                best_path = None
                best_cost = 10**9
                for c_loc in chargers:
                    p = simple_astar(loc, c_loc)
                    if p is None:
                        continue
                    cst = estimate_path_cost(loc, p)
                    if cst < best_cost and cst < energy - 5:
                        best = c_loc
                        best_path = p
                        best_cost = cst
                if best is not None:
                    STATE.current_path = best_path
                    STATE.need_charging = True

        # execute step
        if STATE.current_path:
            d = STATE.current_path[0]
            STATE.current_path = STATE.current_path[1:]
            move(d)
        else:
            STATE.current_target = None
            move(Direction.CENTER)
    except:
        try:
            move(Direction.CENTER)
        except:
            pass
