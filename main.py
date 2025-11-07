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
        i = 0
        while i < len(dirs):
            d = dirs[i]
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
            if ok:
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
            i = i + 1
    return None

def estimate_path_cost(start, path):
    if not path:
        return 0
    total = 0
    cur = start
    i = 0
    while i < len(path):
        d = path[i]
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
        i = i + 1
    return total

def estimate_next_step_cost(cur_loc, direction):
    try:
        nxt = cur_loc.add(direction)
        if not on_map(nxt):
            return None
        cell = get_cell_info_at(nxt)
        mv = 1
        try:
            mv2 = int(cell.move_cost)
            if mv2 > 0:
                mv = mv2
        except:
            mv = 1
        return mv
    except:
        return None

def tiny_split_bias(agent_id, sx, sy):
    try:
        h = (agent_id * 73856093) ^ (sx * 19349663) ^ (sy * 83492791)
    except:
        h = (sx * 19349663) ^ (sy * 83492791)
    v = h % 1000
    return float(v) / 2000.0  # [0..0.4995]

def required_agents_for_xy(sx, sy):
    try:
        cell = get_cell_info_at(Location(sx, sy))
        top = cell.top_layer
        if isinstance(top, Rubble):
            try:
                r = int(top.agents_required)
                if r >= 1:
                    return r
            except:
                return 2
        return 1
    except:
        pass
    try:
        layers = drone_scan(Location(sx, sy))
        i = 0
        while i < len(layers):
            L = layers[i]
            try:
                if isinstance(L, Rubble):
                    try:
                        r = int(L.agents_required)
                        if r >= 1:
                            return r
                    except:
                        return 2
            except:
                pass
            i = i + 1
        return 1
    except:
        return 2

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
        self.claims_cap = {}   # (x,y) -> {"max":k, "taken": {agent_id:1}, "round": int}
        self.help_open = {}    # (x,y) -> {"need":k_remaining, "round":int}
        # Energy tracking for logs
        self.energy_init = None
        self.energy_last = None
        self.last_action = "spawn"
        self.last_spend = 0

STATE = AgentState()

def register_claim(x, y, agent_id, k, round_num):
    key = (x, y)
    entry = STATE.claims_cap.get(key)
    if entry is None:
        entry = {"max": k, "taken": {}, "round": round_num}
    else:
        if k > entry.get("max", 1):
            entry["max"] = k
        entry["round"] = round_num
    taken = entry.get("taken", {})
    taken[agent_id] = 1
    entry["taken"] = taken
    STATE.claims_cap[key] = entry

def claims_taken_for(x, y):
    entry = STATE.claims_cap.get((x, y))
    if entry is None:
        return 0
    return len(entry.get("taken", {}))

def claims_max_for(x, y):
    entry = STATE.claims_cap.get((x, y))
    if entry is None:
        return 0
    return entry.get("max", 1)

def register_help(x, y, needed, round_num):
    STATE.help_open[(x, y)] = {"need": needed, "round": round_num}

def consume_help_slot(x, y):
    entry = STATE.help_open.get((x, y))
    if entry is None:
        return
    need = entry.get("need", 0)
    if need > 1:
        entry["need"] = need - 1
        STATE.help_open[(x, y)] = entry
    else:
        try:
            del STATE.help_open[(x, y)]
        except:
            pass

def clean_old(current_round, ttl):
    klist = list(STATE.claims_cap.keys())
    i = 0
    while i < len(klist):
        k = klist[i]
        try:
            r = STATE.claims_cap[k].get("round", 0)
            if current_round - r > ttl:
                del STATE.claims_cap[k]
        except:
            pass
        i = i + 1
    hlist = list(STATE.help_open.keys())
    j = 0
    while j < len(hlist):
        k = hlist[j]
        try:
            r = STATE.help_open[k].get("round", 0)
            if current_round - r > ttl:
                del STATE.help_open[k]
        except:
            pass
        j = j + 1

def pick_best_target_and_path(loc, energy, survivors, round_num, agent_id):
    candidates = []

    # 1) Open HELP first (nearest free agent goes)
    hkeys = list(STATE.help_open.keys())
    j = 0
    while j < len(hkeys):
        hx, hy = hkeys[j]
        entry = STATE.help_open[(hx, hy)]
        need = entry.get("need", 0)
        if need > 0:
            path = simple_astar(loc, Location(hx, hy))
            if path is not None:
                base_cost = estimate_path_cost(loc, path)
                bias = tiny_split_bias(agent_id, hx, hy)
                ranked = base_cost + bias - 0.25
                tie1 = chebyshev_distance(loc, Location(hx, hy))
                tie2 = abs(hx - loc.x)
                candidates.append((ranked, tie1, tie2, Location(hx, hy), path, base_cost, 1, True))
        j = j + 1

    # 2) Fresh survivor targets with capacity-k
    i = 0
    while i < len(survivors):
        s = survivors[i]
        if (s.x, s.y) in STATE.known_saved:
            i = i + 1
            continue
        kreq = claims_max_for(s.x, s.y)
        if kreq <= 0:
            kreq = required_agents_for_xy(s.x, s.y)
        taken = claims_taken_for(s.x, s.y)
        if kreq > 0 and taken >= kreq:
            i = i + 1
            continue
        path = simple_astar(loc, s)
        if path is not None:
            base_cost = estimate_path_cost(loc, path)
            bias = tiny_split_bias(agent_id, s.x, s.y)
            ranked = base_cost + bias
            tie1 = chebyshev_distance(loc, s)
            tie2 = abs(s.x - loc.x)
            candidates.append((ranked, tie1, tie2, s, path, base_cost, kreq, False))
        i = i + 1

    if not candidates:
        return None, None, None, None, False

    candidates.sort(key=lambda t: (t[0], t[1], t[2]))
    best = candidates[0]
    target_loc = best[3]
    path = best[4]
    true_cost = best[5]
    kreq = best[6]
    from_help = best[7]

    if from_help:
        consume_help_slot(target_loc.x, target_loc.y)
        register_claim(target_loc.x, target_loc.y, agent_id, 2, round_num)
        try:
            send_message("CLAIM2|" + str(target_loc.x) + "|" + str(target_loc.y) + "|" + str(agent_id) + "|" + str(2) + "|" + str(round_num), [])
        except:
            pass
    else:
        if kreq <= 0:
            kreq = required_agents_for_xy(target_loc.x, target_loc.y)
        register_claim(target_loc.x, target_loc.y, agent_id, kreq, round_num)
        try:
            send_message("CLAIM2|" + str(target_loc.x) + "|" + str(target_loc.y) + "|" + str(agent_id) + "|" + str(kreq) + "|" + str(round_num), [])
        except:
            pass

    return target_loc, path, true_cost, kreq, from_help

def think():
    global STATE
    try:
        t = get_round_number()
        loc = get_location()
        energy = get_energy_level()

        if STATE.my_id is None:
            STATE.my_id = get_id()
            STATE.energy_init = energy
            STATE.energy_last = energy
            log("[A" + str(STATE.my_id) + " R" + str(t) + "] INIT energy=" + str(energy))
            move(Direction.CENTER)
            return

        # energy delta since last round (what actually happened after last_action)
        if STATE.energy_last is None:
            STATE.energy_last = energy
        delta = STATE.energy_last - energy
        STATE.last_spend = delta
        log("[A" + str(STATE.my_id) + " R" + str(t) + "] LAST_ACTION=" + str(STATE.last_action) +
            " | ΔE=" + str(delta) + " | E_now=" + str(energy))
        STATE.energy_last = energy
        STATE.last_action = "idle"

        # ingest CLAIM2 and HELP2
        try:
            msgs = read_messages()
            m = 0
            while m < len(msgs):
                msg = msgs[m]
                parts = []
                try:
                    parts = msg.message.split("|")
                except:
                    parts = []
                if len(parts) >= 6 and parts[0] == "CLAIM2":
                    sx = int(parts[1]); sy = int(parts[2])
                    ag = int(parts[3]); kreq = int(parts[4]); r = int(parts[5])
                    register_claim(sx, sy, ag, kreq, r)
                elif len(parts) >= 5 and parts[0] == "HELP2":
                    hx = int(parts[1]); hy = int(parts[2])
                    need = int(parts[3]); r = int(parts[4])
                    register_help(hx, hy, need, r)
                m = m + 1
        except:
            pass
        clean_old(t, 6)

        try:
            survivors = get_survs()
        except:
            survivors = []

        # save if standing on survivor
        try:
            cell_here = get_cell_info_at(loc)
            top_here = cell_here.top_layer
            if isinstance(top_here, Survivor):
                log("[A" + str(STATE.my_id) + " R" + str(t) + "] ACTION save()")
                save()
                STATE.last_action = "save"
                STATE.known_saved.add((loc.x, loc.y))
                STATE.current_target = None
                STATE.current_path = []
                STATE.need_charging = False
                return
        except:
            pass

        # dig if on rubble (pair when needed)
        try:
            cell_here = get_cell_info_at(loc)
            top_here = cell_here.top_layer
            if isinstance(top_here, Rubble):
                req = 1
                try:
                    req = int(top_here.agents_required)
                except:
                    req = 1
                if req == 1:
                    log("[A" + str(STATE.my_id) + " R" + str(t) + "] ACTION dig() req=1")
                    dig()
                    STATE.last_action = "dig"
                    return
                else:
                    count_here = 1
                    try:
                        count_here = len(cell_here.agents)
                    except:
                        count_here = 1
                    if count_here >= 2:
                        log("[A" + str(STATE.my_id) + " R" + str(t) + "] ACTION dig() req=2 with partner")
                        dig()
                        STATE.last_action = "dig"
                        return
                    remaining = req - count_here
                    if remaining < 1:
                        remaining = 1
                    try:
                        send_message("HELP2|" + str(loc.x) + "|" + str(loc.y) + "|" + str(remaining) + "|" + str(t), [])
                        send_message("CLAIM2|" + str(loc.x) + "|" + str(loc.y) + "|" + str(STATE.my_id) + "|" + str(req) + "|" + str(t), [])
                    except:
                        pass
                    register_help(loc.x, loc.y, remaining, t)
                    register_claim(loc.x, loc.y, STATE.my_id, req, t)
                    move(Direction.CENTER)
                    STATE.last_action = "hold"
                    return
        except:
            pass

        # recharge on charger until plan cost + buffer
        try:
            cell_here = get_cell_info_at(loc)
            if "CHARGING" in str(cell_here.type).upper():
                plan_cost = 0
                if STATE.current_path:
                    plan_cost = estimate_path_cost(loc, STATE.current_path)
                need_e = plan_cost + 10
                if energy < need_e:
                    log("[A" + str(STATE.my_id) + " R" + str(t) + "] ACTION recharge() targetE>=" + str(need_e))
                    recharge()
                    STATE.last_action = "recharge"
                    return
                STATE.charging_now = False
                STATE.need_charging = False
                STATE.current_path = []
        except:
            pass

        if not survivors:
            move(Direction.CENTER)
            STATE.last_action = "hold"
            return

        # stuck detection -> replan
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

        # choose / re-choose with capacity + help awareness
        if STATE.current_target is None or STATE.need_charging:
            s, path, cost, kreq, used_help = pick_best_target_and_path(loc, energy, survivors, t, STATE.my_id)
            if s is None:
                move(Direction.CENTER)
                STATE.last_action = "hold"
                return
            STATE.current_target = s
            STATE.current_path = path
            log("[A" + str(STATE.my_id) + " R" + str(t) + "] PLAN target=(" + str(s.x) + "," + str(s.y) + ") cost=" + str(cost))

        if not STATE.current_path:
            s, path, cost, kreq, used_help = pick_best_target_and_path(loc, energy, survivors, t, STATE.my_id)
            if s is None:
                move(Direction.CENTER)
                STATE.last_action = "hold"
                return
            STATE.current_target = s
            STATE.current_path = path
            log("[A" + str(STATE.my_id) + " R" + str(t) + "] PLAN reselect target=(" + str(s.x) + "," + str(s.y) + ") cost=" + str(cost))

        # energy-aware detour to nearest charger if needed
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
                best_cost = 1000000000
                i = 0
                while i < len(chargers):
                    c_loc = chargers[i]
                    p = simple_astar(loc, c_loc)
                    if p is not None:
                        cst = estimate_path_cost(loc, p)
                        if (cst < best_cost) and (cst < energy - 5):
                            best = c_loc
                            best_path = p
                            best_cost = cst
                    i = i + 1
                if best is not None:
                    STATE.current_path = best_path
                    STATE.need_charging = True
                    log("[A" + str(STATE.my_id) + " R" + str(t) + "] PLAN detour->charger at (" + str(best.x) + "," + str(best.y) + ") cost=" + str(best_cost))

        # execute one step; log BEFORE moving with est step cost
        if STATE.current_path:
            d = STATE.current_path[0]
            est_step = estimate_next_step_cost(loc, d)
            log("[A" + str(STATE.my_id) + " R" + str(t) + "] MOVE " + str(d) +
                (" | estStepCost=" + str(est_step) if est_step is not None else "") +
                " | E_now=" + str(energy))
            STATE.current_path = STATE.current_path[1:]
            STATE.last_action = "move"
            move(d)
        else:
            STATE.current_target = None
            move(Direction.CENTER)
            STATE.last_action = "hold"
    except Exception as e:
        try:
            log("[A" + (str(STATE.my_id) if (STATE.my_id is not None) else "?") + " R" + (str(t) if ('t' in locals()) else "?") + "] ERROR " + str(e))
        except:
            pass
        try:
            move(Direction.CENTER)
            STATE.last_action = "hold"
        except:
            pass
