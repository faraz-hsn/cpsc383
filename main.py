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
        # NEW: simple auction state
        self.claims = {}      # (sx,sy) -> {"agent": int, "eta": int, "round": int}
        self.claim_ttl = 5

STATE = AgentState()

def better_claim(new_eta, new_agent, new_round, old):
    if old is None:
        return True
    if new_eta < old["eta"]:
        return True
    if new_eta > old["eta"]:
        return False
    if new_agent < old["agent"]:
        return True
    if new_agent > old["agent"]:
        return False
    return new_round < old["round"]

def prune_expired_claims(current_round):
    keys = list(STATE.claims.keys())
    i = 0
    while i < len(keys):
        k = keys[i]
        try:
            if current_round - STATE.claims[k]["round"] > STATE.claim_ttl:
                del STATE.claims[k]
        except:
            pass
        i = i + 1

def pick_best_target_and_path_auction(loc, energy, survivors, round_num):
    candidates = []
    i = 0
    while i < len(survivors):
        s = survivors[i]
        if (s.x, s.y) in STATE.known_saved:
            i = i + 1
            continue
        path = simple_astar(loc, s)
        if path is not None:
            eta = estimate_path_cost(loc, path)
            claim = STATE.claims.get((s.x, s.y))
            if claim is not None:
                # if someone else has a strictly better claim, skip this survivor
                if not better_claim(eta, STATE.my_id, round_num, claim):
                    i = i + 1
                    continue
            tie1 = chebyshev_distance(loc, s)
            tie2 = abs(s.x - loc.x)
            candidates.append((eta, tie1, tie2, s, path))
        i = i + 1
    if not candidates:
        return None, None, None
    candidates.sort(key=lambda t: (t[0], t[1], t[2]))
    best = candidates[0]
    # broadcast our claim so peers defer; RestrictedPython-safe string building
    try:
        send_message("CLAIM|" + str(best[3].x) + "|" + str(best[3].y) + "|" + str(STATE.my_id) + "|" + str(best[0]) + "|" + str(round_num), [])
    except:
        pass
    return best[3], best[4], best[0]

def think():
    global STATE
    try:
        t = get_round_number()
        loc = get_location()
        energy = get_energy_level()

        if STATE.my_id is None:
            STATE.my_id = get_id()
            log("[A" + str(STATE.my_id) + " T" + str(t) + "] STATUS spawn at (" + str(loc.x) + "," + str(loc.y) + ") E=" + str(energy))
            move(Direction.CENTER)
            return

        # NEW: read incoming CLAIM messages and maintain TTL
        try:
            msgs = read_messages()
            k = 0
            while k < len(msgs):
                m = msgs[k]
                parts = []
                try:
                    parts = m.message.split("|")
                except:
                    parts = []
                if len(parts) >= 6 and parts[0] == "CLAIM":
                    sx = int(parts[1])
                    sy = int(parts[2])
                    ag = int(parts[3])
                    eta = int(parts[4])
                    rnd = int(parts[5])
                    key = (sx, sy)
                    old = STATE.claims.get(key)
                    if better_claim(eta, ag, rnd, old):
                        STATE.claims[key] = {"agent": ag, "eta": eta, "round": rnd}
                k = k + 1
        except:
            pass
        prune_expired_claims(t)

        try:
            survivors = get_survs()
        except:
            survivors = []

        # Save if standing on survivor
        try:
            cell_here = get_cell_info_at(loc)
            top_here = cell_here.top_layer
            if isinstance(top_here, Survivor):
                log("[A" + str(STATE.my_id) + " T" + str(t) + "] INTENT save at (" + str(loc.x) + "," + str(loc.y) + ")")
                save()
                log("[A" + str(STATE.my_id) + " T" + str(t) + "] ACTION save() -> done")
                STATE.known_saved.add((loc.x, loc.y))
                STATE.current_target = None
                STATE.current_path = []
                STATE.need_charging = False
                return
        except:
            pass

        # Dig if on rubble
        try:
            cell_here = get_cell_info_at(loc)
            top_here = cell_here.top_layer
            if isinstance(top_here, Rubble):
                req = top_here.agents_required
                if req == 1:
                    log("[A" + str(STATE.my_id) + " T" + str(t) + "] INTENT dig1 at (" + str(loc.x) + "," + str(loc.y) + ")")
                    dig()
                    log("[A" + str(STATE.my_id) + " T" + str(t) + "] ACTION dig()")
                    return
                else:
                    count_here = 1
                    try:
                        count_here = len(cell_here.agents)
                    except:
                        count_here = 1
                    if count_here >= 2:
                        log("[A" + str(STATE.my_id) + " T" + str(t) + "] INTENT dig2 with partner at (" + str(loc.x) + "," + str(loc.y) + ")")
                        dig()
                        log("[A" + str(STATE.my_id) + " T" + str(t) + "] ACTION dig() (paired)")
                        return
                    else:
                        log("[A" + str(STATE.my_id) + " T" + str(t) + "] STATUS waiting-for-partner at (" + str(loc.x) + "," + str(loc.y) + ")")
                        move(Direction.CENTER)
                        log("[A" + str(STATE.my_id) + " T" + str(t) + "] ACTION hold CENTER")
                        return
        except:
            pass

        # Recharge behavior
        try:
            cell_here = get_cell_info_at(loc)
            if "CHARGING" in str(cell_here.type).upper():
                if energy < 80:
                    log("[A" + str(STATE.my_id) + " T" + str(t) + "] INTENT recharge at (" + str(loc.x) + "," + str(loc.y) + ") E=" + str(energy))
                    recharge()
                    log("[A" + str(STATE.my_id) + " T" + str(t) + "] ACTION recharge() -> E+5")
                    STATE.charging_now = True
                    return
                else:
                    log("[A" + str(STATE.my_id) + " T" + str(t) + "] STATUS charged E=" + str(energy) + ", resume")
                    STATE.charging_now = False
                    STATE.need_charging = False
                    STATE.current_path = []
        except:
            pass

        if not survivors:
            log("[A" + str(STATE.my_id) + " T" + str(t) + "] STATUS no-survivors-seen, idle")
            move(Direction.CENTER)
            log("[A" + str(STATE.my_id) + " T" + str(t) + "] ACTION hold CENTER")
            return

        # Stuck detection
        if STATE.last_location is not None:
            if (STATE.last_location.x == loc.x) and (STATE.last_location.y == loc.y):
                STATE.stuck_count = STATE.stuck_count + 1
                if STATE.stuck_count > 3:
                    log("[A" + str(STATE.my_id) + " T" + str(t) + "] STATUS stuck>3, replan")
                    STATE.current_path = []
                    STATE.current_target = None
                    STATE.need_charging = False
                    STATE.stuck_count = 0
            else:
                STATE.stuck_count = 0
        STATE.last_location = loc

        # Choose or re-choose target with auction to prevent clumping
        if (STATE.current_target is None) or STATE.need_charging:
            s, path, cost = pick_best_target_and_path_auction(loc, energy, survivors, t)
            if s is None:
                log("[A" + str(STATE.my_id) + " T" + str(t) + "] STATUS no-reachable-targets, idle")
                move(Direction.CENTER)
                log("[A" + str(STATE.my_id) + " T" + str(t) + "] ACTION hold CENTER")
                return
            STATE.current_target = s
            STATE.current_path = path
            log("[A" + str(STATE.my_id) + " T" + str(t) + "] PLAN target=(" + str(s.x) + "," + str(s.y) + ") cost=" + str(cost) + " from (" + str(loc.x) + "," + str(loc.y) + ")")

        if not STATE.current_path:
            s, path, cost = pick_best_target_and_path_auction(loc, energy, survivors, t)
            if s is None:
                log("[A" + str(STATE.my_id) + " T" + str(t) + "] STATUS lost-path and no-reachable-targets, idle")
                move(Direction.CENTER)
                log("[A" + str(STATE.my_id) + " T" + str(t) + "] ACTION hold CENTER")
                return
            STATE.current_target = s
            STATE.current_path = path
            log("[A" + str(STATE.my_id) + " T" + str(t) + "] PLAN reselect target=(" + str(s.x) + "," + str(s.y) + ") cost=" + str(cost) + ")")

        # Energy gate → optional charger detour
        if STATE.current_path:
            path_cost = estimate_path_cost(loc, STATE.current_path)
            if path_cost > (energy * 0.7):
                log("[A" + str(STATE.my_id) + " T" + str(t) + "] ENERGY low, seek charger")
                chargers = []
                tried = False
                try:
                    chargers = get_charging_cells()
                    tried = True
                except:
                    chargers = []
                if chargers:
                    best = None
                    best_path = None
                    best_cost = 1000000000
                    j = 0
                    while j < len(chargers):
                        c_loc = chargers[j]
                        p = simple_astar(loc, c_loc)
                        if p is not None:
                            cst = estimate_path_cost(loc, p)
                            if (cst < (energy - 10)) and (cst < best_cost):
                                best = c_loc
                                best_path = p
                                best_cost = cst
                        j = j + 1
                    if best is not None:
                        STATE.current_path = best_path
                        STATE.need_charging = True
                        log("[A" + str(STATE.my_id) + " T" + str(t) + "] PLAN route->charger (" + str(best.x) + "," + str(best.y) + ") len=" + str(len(best_path)) + " cost=" + str(best_cost))
                else:
                    if tried:
                        log("[A" + str(STATE.my_id) + " T" + str(t) + "] ENERGY no chargers listed; proceed anyway")

        # Yield to a better claim that arrived after we picked (prevents last-second pile-up)
        if STATE.current_target is not None:
            cl = STATE.claims.get((STATE.current_target.x, STATE.current_target.y))
            if cl is not None and cl["agent"] != STATE.my_id:
                my_eta = estimate_path_cost(loc, STATE.current_path) if STATE.current_path else 10**9
                if not better_claim(my_eta, STATE.my_id, t, cl):
                    log("[A" + str(STATE.my_id) + " T" + str(t) + "] STATUS yield-to-better-claim for (" + str(STATE.current_target.x) + "," + str(STATE.current_target.y) + ")")
                    STATE.current_target = None
                    STATE.current_path = []

        # Execute one step
        if STATE.current_path:
            d = STATE.current_path[0]
            STATE.current_path = STATE.current_path[1:]
            log("[A" + str(STATE.my_id) + " T" + str(t) + "] ACTION move " + str(d))
            move(d)
        else:
            STATE.current_target = None
            log("[A" + str(STATE.my_id) + " T" + str(t) + "] STATUS no-path, idle")
            move(Direction.CENTER)
            log("[A" + str(STATE.my_id) + " T" + str(t) + "] ACTION hold CENTER")
    except Exception as e:
        try:
            log("[A" + (str(STATE.my_id) if (STATE.my_id is not None) else "?") + " T" + (str(t) if ('t' in locals()) else "?") + "] ERROR " + str(e))
        except:
            pass
        try:
            move(Direction.CENTER)
            try:
                log("[A" + str(STATE.my_id) + "] ACTION hold CENTER (on-error)")
            except:
                pass
        except:
            pass
