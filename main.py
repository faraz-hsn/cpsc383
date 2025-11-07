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
        dirs = get_all_directions()
        for d in dirs:
            ok = True
            nxt = None
            cell = None
            try:
                nxt = cur.add(d)
                if not on_map(nxt):
                    ok = False
                if ok:
                    cell = get_cell_info_at(nxt)
                    if cell.is_killer_cell():
                        ok = False
            except:
                ok = False
            if not ok:
                continue
            step = 1
            try:
                mv = int(cell.move_cost)
                if mv > 0:
                    step = mv
            except:
                step = 1
            nk = (nxt.x, nxt.y)
            cand = g[ck] + step
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
            step = 1
            try:
                mv = int(cell.move_cost)
                if mv > 0:
                    step = mv
            except:
                step = 1
            total = total + step
            cur = nxt
        except:
            total = total + 1
    return total

def tiny_split_bias(agent_id, sx, sy):
    # Deterministic, very small positive bias different per (agent, survivor).
    # Keeps nearest target dominant but breaks ties on tick 1 (before CLAIMs arrive).
    try:
        h = (agent_id * 73856093) ^ (sx * 19349663) ^ (sy * 83492791)
    except:
        h = (sx * 19349663) ^ (sy * 83492791)
    # Map to [0, 0.999] and scale down to < 0.5 energy so it never beats a real cost gap.
    val = h % 1000
    return float(val) / 2000.0  # bias in [0.0, 0.4995]

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
    i = 0
    while i < len(survivors):
        s = survivors[i]
        if (s.x, s.y) in STATE.known_saved:
            i = i + 1
            continue
        # Respect claims if another agent already announced
        owner = STATE.claims.get((s.x, s.y))
        if owner is not None and owner != STATE.my_id:
            i = i + 1
            continue
        path = simple_astar(loc, s)
        if path is not None:
            cost = estimate_path_cost(loc, path)
            # Add tiny deterministic bias so agents split immediately
            bias = tiny_split_bias(STATE.my_id if STATE.my_id is not None else 0, s.x, s.y)
            ranked = cost + bias
            tie1 = chebyshev_distance(loc, s)
            tie2 = abs(s.x - loc.x)
            candidates.append((ranked, tie1, tie2, s, path, cost))
        i = i + 1
    if not candidates:
        return None, None, None
    candidates.sort(key=lambda t: (t[0], t[1], t[2]))
    best = candidates[0]
    target = best[3]
    path = best[4]
    true_cost = best[5]
    # Broadcast a simple CLAIM so late joiners defer (arrives next tick)
    try:
        send_message("CLAIM|" + str(target.x) + "|" + str(target.y) + "|" + str(STATE.my_id), [])
    except:
        pass
    STATE.claims[(target.x, target.y)] = STATE.my_id
    return target, path, true_cost

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

        # Ingest claims (1-tick latency); keeps others from piling on our pick next tick
        try:
            msgs = read_messages()
            j = 0
            while j < len(msgs):
                m = msgs[j]
                parts = []
                try:
                    parts = m.message.split("|")
                except:
                    parts = []
                if len(parts) >= 4 and parts[0] == "CLAIM":
                    x = int(parts[1]); y = int(parts[2]); ag = int(parts[3])
                    STATE.claims[(x, y)] = ag
                j = j + 1
        except:
            pass

        try:
            survivors = get_survs()
        except:
            survivors = []

        # Save if standing on survivor
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

        # Dig if on rubble (handles single vs pair)
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

        # Dynamic recharge while on a charger: only until enough for current plan (+ buffer)
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

        # Stuck detection -> replan
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

        # Choose target (with bias) if none or after charging
        if STATE.current_target is None or STATE.need_charging:
            s, path, cost = pick_best_target_and_path(loc, energy, survivors, t)
            if s is None:
                move(Direction.CENTER)
                return
            STATE.current_target = s
            STATE.current_path = path

        # If we lost a path (blocked by killer wall discovery), reselect
        if not STATE.current_path:
            s, path, cost = pick_best_target_and_path(loc, energy, survivors, t)
            if s is None:
                move(Direction.CENTER)
                return
            STATE.current_target = s
            STATE.current_path = path

        # Energy-aware reroute: if not enough energy to finish current path, detour to a charger
        if STATE.current_path:
            path_cost = estimate_path_cost(loc, STATE.current_path)
            # simple margin 5: ensures reaching charger safely
            if path_cost >= energy - 5:
                chargers = []
                try:
                    chargers = get_charging_cells()
                except:
                    chargers = []
                best = None
                best_path = None
                best_cost = 1000000000
                k = 0
                while k < len(chargers):
                    c_loc = chargers[k]
                    p = simple_astar(loc, c_loc)
                    if p is not None:
                        cst = estimate_path_cost(loc, p)
                        if cst < best_cost and cst < energy - 5:
                            best = c_loc
                            best_path = p
                            best_cost = cst
                    k = k + 1
                if best is not None:
                    STATE.current_path = best_path
                    STATE.need_charging = True

        # One step execution
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
