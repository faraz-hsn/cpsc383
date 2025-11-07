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
        for d in get_all_directions():
            try:
                nxt = cur.add(d)
                if not on_map(nxt):
                    continue
                cell = get_cell_info_at(nxt)
                if cell.is_killer_cell():
                    continue
                try:
                    step = int(cell.move_cost)
                    if step <= 0: step = 1
                except:
                    step = 1
                nk = (nxt.x, nxt.y)
                cand = g[ck] + step
                if nk not in g or cand < g[nk]:
                    came_from[nk] = (cur, d)
                    g[nk] = cand
                    f = cand + chebyshev_distance(nxt, goal)
                    c += 1
                    heapq.heappush(open_set, (f, c, nxt))
            except:
                continue
    return None

def estimate_path_cost(start, path):
    if not path: return 0
    total = 0
    cur = start
    for d in path:
        try:
            nxt = cur.add(d)
            cell = get_cell_info_at(nxt)
            cost = int(cell.move_cost)
            if cost <= 0: cost = 1
            total += cost
            cur = nxt
        except:
            total += 1
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
            log(f"[A{STATE.my_id} T{t}] STATUS spawn at ({loc.x},{loc.y}) E={energy}")
            move(Direction.CENTER)
            return

        try:
            survivors = get_survs()
        except:
            survivors = []

        try:
            cell = get_cell_info_at(loc)
            top = cell.top_layer
            if isinstance(top, Survivor):
                log(f"[A{STATE.my_id} T{t}] INTENT save at ({loc.x},{loc.y})")
                save()
                log(f"[A{STATE.my_id} T{t}] ACTION save() -> done")
                STATE.known_saved.add((loc.x, loc.y))
                STATE.current_target = None
                STATE.current_path = []
                STATE.need_charging = False
                return
        except:
            pass

        try:
            cell = get_cell_info_at(loc)
            top = cell.top_layer
            if isinstance(top, Rubble):
                req = top.agents_required
                if req == 1:
                    log(f"[A{STATE.my_id} T{t}] INTENT dig1 at ({loc.x},{loc.y})")
                    dig()
                    log(f"[A{STATE.my_id} T{t}] ACTION dig()")
                    return
                else:
                    here = len(cell.agents)
                    if here >= 2:
                        log(f"[A{STATE.my_id} T{t}] INTENT dig2 with partner at ({loc.x},{loc.y})")
                        dig()
                        log(f"[A{STATE.my_id} T{t}] ACTION dig() (paired)")
                        return
                    else:
                        log(f"[A{STATE.my_id} T{t}] STATUS waiting-for-partner at ({loc.x},{loc.y})")
                        move(Direction.CENTER)
                        log(f"[A{STATE.my_id} T{t}] ACTION hold CENTER")
                        return
        except:
            pass

        try:
            cell = get_cell_info_at(loc)
            if 'CHARGING' in str(cell.type).upper():
                if energy < 80:
                    log(f"[A{STATE.my_id} T{t}] INTENT recharge at ({loc.x},{loc.y}) E={energy}")
                    recharge()
                    log(f"[A{STATE.my_id} T{t}] ACTION recharge() -> E+5")
                    STATE.charging_now = True
                    return
                else:
                    log(f"[A{STATE.my_id} T{t}] STATUS charged E={energy}, resume")
                    STATE.charging_now = False
                    STATE.need_charging = False
                    STATE.current_path = []
        except:
            pass

        if not survivors:
            log(f"[A{STATE.my_id} T{t}] STATUS no-survivors-seen, idle")
            move(Direction.CENTER)
            log(f"[A{STATE.my_id} T{t}] ACTION hold CENTER")
            return

        if STATE.last_location:
            if STATE.last_location.x == loc.x and STATE.last_location.y == loc.y:
                STATE.stuck_count += 1
                if STATE.stuck_count > 3:
                    log(f"[A{STATE.my_id} T{t}] STATUS stuck>3, replan")
                    STATE.current_path = []
                    STATE.current_target = None
                    STATE.need_charging = False
                    STATE.stuck_count = 0
            else:
                STATE.stuck_count = 0
        STATE.last_location = loc

        if STATE.current_target is None or STATE.need_charging:
            available = [s for s in survivors if (s.x, s.y) not in STATE.known_saved]
            if not available:
                log(f"[A{STATE.my_id} T{t}] STATUS all-known survivors saved, idle")
                move(Direction.CENTER)
                log(f"[A{STATE.my_id} T{t}] ACTION hold CENTER")
                return
            closest = min(available, key=lambda s: chebyshev_distance(loc, s))
            STATE.current_target = closest
            STATE.current_path = []
            log(f"[A{STATE.my_id} T{t}] PLAN target=({closest.x},{closest.y}) from ({loc.x},{loc.y})")

        if not STATE.current_path:
            path = simple_astar(loc, STATE.current_target)
            if not path:
                log(f"[A{STATE.my_id} T{t}] STATUS unreachable target=({STATE.current_target.x},{STATE.current_target.y}), drop")
                STATE.current_target = None
                move(Direction.CENTER)
                log(f"[A{STATE.my_id} T{t}] ACTION hold CENTER")
                return
            cost = estimate_path_cost(loc, path)
            log(f"[A{STATE.my_id} T{t}] PLAN path_len={len(path)} cost={cost} E={energy}")
            if cost > energy * 0.7:
                log(f"[A{STATE.my_id} T{t}] ENERGY low, seek charger")
                try:
                    chargers = get_charging_cells()
                    if chargers:
                        best = None
                        best_path = None
                        best_cost = 10**9
                        for c in chargers:
                            p = simple_astar(loc, c)
                            if p:
                                cst = estimate_path_cost(loc, p)
                                if cst < energy - 10 and cst < best_cost:
                                    best, best_path, best_cost = c, p, cst
                        if best:
                            STATE.current_path = best_path
                            STATE.need_charging = True
                            log(f"[A{STATE.my_id} T{t}] PLAN route->charger ({best.x},{best.y}) len={len(best_path)} cost={best_cost}")
                        else:
                            STATE.current_path = path
                            STATE.need_charging = False
                            log(f"[A{STATE.my_id} T{t}] ENERGY no-reachable-charger, proceed to target")
                    else:
                        STATE.current_path = path
                        STATE.need_charging = False
                        log(f"[A{STATE.my_id} T{t}] ENERGY no chargers in map, proceed to target")
                except:
                    STATE.current_path = path
                    STATE.need_charging = False
                    log(f"[A{STATE.my_id} T{t}] ENERGY charger check failed, proceed to target")
            else:
                STATE.current_path = path
                STATE.need_charging = False
                log(f"[A{STATE.my_id} T{t}] PLAN proceed-to-target")

        if STATE.current_path:
            d = STATE.current_path.pop(0)
            log(f"[A{STATE.my_id} T{t}] ACTION move {d}")
            move(d)
        else:
            STATE.current_target = None
            log(f"[A{STATE.my_id} T{t}] STATUS no-path, idle")
            move(Direction.CENTER)
            log(f"[A{STATE.my_id} T{t}] ACTION hold CENTER")
    except Exception as e:
        log(f"[A{STATE.my_id if STATE.my_id else '?'} T{get_round_number() if 'get_round_number' in globals() else '?'}] ERROR {e}")
        try:
            move(Direction.CENTER)
            log(f"[A{STATE.my_id}] ACTION hold CENTER (on-error)")
        except:
            pass
