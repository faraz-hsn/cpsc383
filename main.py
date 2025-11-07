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

def tiny_split_bias(agent_id, sx, sy):
    try:
        h = (agent_id * 73856093) ^ (sx * 19349663) ^ (sy * 83492791)
    except:
        h = (sx * 19349663) ^ (sy * 83492791)
    v = h % 1000
    return float(v) / 2000.0  # [0..0.4995], only breaks first-tick ties

def required_agents_for_xy(sx, sy):
    # Try visible top layer first
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
        # survivor or empty on top -> 1
        return 1
    except:
        pass
    # Try drone scan if exposed in this build
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
        # Conservative default (safer to over-allocate for rubble)
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
        # capacity-aware claims: (x,y) -> {"max":k, "taken": {agent_id:1}, "round": int}
        self.claims_cap = {}
        # help requests: (x,y) -> {"need":k_needed_remaining, "round":int}
        self.help_open = {}

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
    # claims
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
    # help
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
    # Build candidate list = survivors with free claim capacity + open HELP2 slots
    candidates = []

    # 1) HELP slots first (so nearest free agent goes to assist waiting rubble)
    hkeys = list(STATE.help_open.keys())
    j = 0
    while j < len(hkeys):
        (hx, hy) = hkeys[j]
        entry = STATE.help_open[(hx, hy)]
        need = entry.get("need", 0)
        if need > 0:
            path = simple_astar(loc, Location(hx, hy))
            if path is not None:
                base_cost = estimate_path_cost(loc, path)
                bias = tiny_split_bias(agent_id, hx, hy)
                ranked = base_cost + bias - 0.25  # slight bonus to favor HELP over fresh picks
                tie1 = chebyshev_distance(loc, Location(hx, hy))
                tie2 = abs(hx - loc.x)
                candidates.append((ranked, tie1, tie2, Location(hx, hy), path, base_cost, 1, True))
        j = j + 1

    # 2) Fresh survivor targets with capacity-k claims
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
        # consume one help slot and treat as claim of that slot (k=1)
        consume_help_slot(target_loc.x, target_loc.y)
        register_claim(target_loc.x, target_loc.y, agent_id, 2, round_num)  # cap stays >=2 to not block partner
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
            move(Direction.CENTER)
            return

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

        # if standing on survivor -> save
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

        # if standing on rubble -> dig or ask for help
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
                    # Not enough partners here: advertise HELP2 (remaining needed)
                    remaining = req - count_here
                    if remaining < 1:
                        remaining = 1
                    try:
                        send_message("HELP2|" + str(loc.x) + "|" + str(loc.y) + "|" + str(remaining) + "|" + str(t), [])
                        # also re-broadcast CLAIM2 with k=req to keep capacity visible
                        send_message("CLAIM2|" + str(loc.x) + "|" + str(loc.y) + "|" + str(STATE.my_id) + "|" + str(req) + "|" + str(t), [])
                    except:
                        pass
                    register_help(loc.x, loc.y, remaining, t)
                    register_claim(loc.x, loc.y, STATE.my_id, req, t)
                    move(Direction.CENTER)
                    return
        except:
            pass

        # dynamic recharge on charger: until plan cost + buffer
        try:
            cell_here = get_cell_info_at(loc)
            if "CHARGING" in str(cell_here.type).upper():
                plan_cost = 0
                if STATE.current_path:
                    plan_cost = estimate_path_cost(loc, STATE.current_path)
                need_e = plan_cost + 10
                if energy < need_e:
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
                return
            STATE.current_target = s
            STATE.current_path = path

        if not STATE.current_path:
            s, path, cost, kreq, used_help = pick_best_target_and_path(loc, energy, survivors, t, STATE.my_id)
            if s is None:
                move(Direction.CENTER)
                return
            STATE.current_target = s
            STATE.current_path = path

        # If not enough energy to finish current plan, detour to nearest charger
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

        # execute one step
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
