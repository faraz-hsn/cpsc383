from aegis_game.stub import *
import heapq

def chebyshev_distance(a, b):
    return max(abs(a.x - b.x), abs(a.y - b.y))

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
                step_val = int(cell.move_cost)
                if step_val > 0:
                    step = step_val
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
            cost = 1
            try:
                mc = int(cell.move_cost)
                if mc > 0:
                    cost = mc
            except:
                cost = 1
            total = total + cost
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

STATE = AgentState()

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

        try:
            survivors = get_survs()
        except:
            survivors = []

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
                    here_count = 0
                    try:
                        here_count = len(cell_here.agents)
                    except:
                        here_count = 1
                    if here_count >= 2:
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

        if (STATE.current_target is None) or STATE.need_charging:
            available = []
            for s in survivors:
                if (s.x, s.y) not in STATE.known_saved:
                    available.append(s)
            if not available:
                log("[A" + str(STATE.my_id) + " T" + str(t) + "] STATUS all-known survivors saved, idle")
                move(Direction.CENTER)
                log("[A" + str(STATE.my_id) + " T" + str(t) + "] ACTION hold CENTER")
                return
            closest = available[0]
            i = 1
            best_dist = chebyshev_distance(loc, closest)
            while i < len(available):
                d = chebyshev_distance(loc, available[i])
                if d < best_dist:
                    best_dist = d
                    closest = available[i]
                i = i + 1
            STATE.current_target = closest
            STATE.current_path = []
            log("[A" + str(STATE.my_id) + " T" + str(t) + "] PLAN target=(" + str(closest.x) + "," + str(closest.y) + ") from (" + str(loc.x) + "," + str(loc.y) + ")")

        if not STATE.current_path:
            path = simple_astar(loc, STATE.current_target)
            if not path:
                log("[A" + str(STATE.my_id) + " T" + str(t) + "] STATUS unreachable target=(" + str(STATE.current_target.x) + "," + str(STATE.current_target.y) + "), drop")
                STATE.current_target = None
                move(Direction.CENTER)
                log("[A" + str(STATE.my_id) + " T" + str(t) + "] ACTION hold CENTER")
                return
            cost = estimate_path_cost(loc, path)
            log("[A" + str(STATE.my_id) + " T" + str(t) + "] PLAN path_len=" + str(len(path)) + " cost=" + str(cost) + " E=" + str(energy))
            if cost > (energy * 0.7):
                log("[A" + str(STATE.my_id) + " T" + str(t) + "] ENERGY low, seek charger")
                chargers = []
                try:
                    chargers = get_charging_cells()
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
                        if p:
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
                        STATE.current_path = path
                        STATE.need_charging = False
                        log("[A" + str(STATE.my_id) + " T" + str(t) + "] ENERGY no-reachable-charger, proceed to target")
                else:
                    STATE.current_path = path
                    STATE.need_charging = False
                    log("[A" + str(STATE.my_id) + " T" + str(t) + "] ENERGY no chargers in map, proceed to target")
            else:
                STATE.current_path = path
                STATE.need_charging = False
                log("[A" + str(STATE.my_id) + " T" + str(t) + "] PLAN proceed-to-target")

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
