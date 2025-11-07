from aegis_game.stub import *
import heapq

def chebyshev_distance(loc1, loc2):
    return max(abs(loc1.x - loc2.x), abs(loc1.y - loc2.y))

def get_all_directions():
    return [
        Direction.NORTH,
        Direction.NORTHEAST,
        Direction.EAST,
        Direction.SOUTHEAST,
        Direction.SOUTH,
        Direction.SOUTHWEST,
        Direction.WEST,
        Direction.NORTHWEST
    ]

def simple_astar(start, goal):
    if start.x == goal.x and start.y == goal.y:
        return []
    open_set = []
    heapq.heappush(open_set, (0, 0, start))
    came_from = {}
    g_score = {(start.x, start.y): 0}
    counter = 0
    while open_set:
        _, _, current = heapq.heappop(open_set)
        current_key = (current.x, current.y)
        if current.x == goal.x and current.y == goal.y:
            path = []
            while current_key in came_from:
                prev_loc, direction = came_from[current_key]
                path.append(direction)
                current_key = (prev_loc.x, prev_loc.y)
            path.reverse()
            return path
        for direction in get_all_directions():
            try:
                neighbor = current.add(direction)
                if not on_map(neighbor):
                    continue
                cell_info = get_cell_info_at(neighbor)
                if cell_info.is_killer_cell():
                    continue
                try:
                    cost = int(cell_info.move_cost)
                    if cost <= 0:
                        cost = 1
                except:
                    cost = 1
                neighbor_key = (neighbor.x, neighbor.y)
                tentative_g = g_score[current_key] + cost
                if neighbor_key not in g_score or tentative_g < g_score[neighbor_key]:
                    came_from[neighbor_key] = (current, direction)
                    g_score[neighbor_key] = tentative_g
                    f_score = tentative_g + chebyshev_distance(neighbor, goal)
                    counter = counter + 1
                    heapq.heappush(open_set, (f_score, counter, neighbor))
            except:
                continue
    return None

def estimate_path_cost(start, path):
    if not path:
        return 0
    total = 0
    current = start
    for direction in path:
        try:
            next_loc = current.add(direction)
            cell_info = get_cell_info_at(next_loc)
            cost = int(cell_info.move_cost)
            if cost <= 0:
                cost = 1
            total = total + cost
            current = next_loc
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
        self.known_saved_survivors = set()
        self.claimed_survivors = {}
        self.need_charging = False
        self.charging_now = False

STATE = AgentState()

def think():
    global STATE
    try:
        round_num = get_round_number()
        my_loc = get_location()
        my_energy = get_energy_level()
        if STATE.my_id is None:
            STATE.my_id = get_id()
            if round_num == 1:
                move(Direction.CENTER)
                send_message(f"READY|{STATE.my_id}|{my_loc.x},{my_loc.y}", [])
                return
        try:
            survivors = get_survs()
        except:
            survivors = []
        try:
            messages = read_messages()
            for msg in messages:
                parts = msg.message.split("|")
                if len(parts) < 2:
                    continue
                msg_type = parts[0]
                if msg_type == "SAVED":
                    try:
                        x, y = int(parts[1]), int(parts[2])
                        STATE.known_saved_survivors.add((x, y))
                    except:
                        pass
                elif msg_type == "CLAIM":
                    try:
                        x, y = int(parts[1]), int(parts[2])
                        agent_id = int(parts[3])
                        STATE.claimed_survivors[(x, y)] = agent_id
                    except:
                        pass
        except:
            pass
        try:
            cell = get_cell_info_at(my_loc)
            top_layer = cell.top_layer
            if isinstance(top_layer, Survivor):
                save()
                STATE.known_saved_survivors.add((my_loc.x, my_loc.y))
                STATE.current_target = None
                STATE.current_path = []
                STATE.need_charging = False
                send_message(f"SAVED|{my_loc.x}|{my_loc.y}", [])
                return
        except:
            pass
        try:
            cell = get_cell_info_at(my_loc)
            top_layer = cell.top_layer
            if isinstance(top_layer, Rubble):
                agents_required = top_layer.agents_required
                if agents_required == 1:
                    dig()
                    return
                elif agents_required == 2:
                    agents_here = cell.agents
                    if len(agents_here) >= 2:
                        dig()
                        return
                    else:
                        send_message(f"HELP|{my_loc.x}|{my_loc.y}", [])
                        move(Direction.CENTER)
                        return
        except:
            pass
        try:
            cell = get_cell_info_at(my_loc)
            cell_type_str = str(cell.type)
            if 'CHARGING' in cell_type_str.upper():
                if my_energy < 80:
                    recharge()
                    STATE.charging_now = True
                    return
                else:
                    STATE.need_charging = False
                    STATE.charging_now = False
                    STATE.current_path = []
        except:
            pass
        if not survivors:
            move(Direction.CENTER)
            return
        if STATE.last_location:
            if STATE.last_location.x == my_loc.x and STATE.last_location.y == my_loc.y:
                STATE.stuck_count = STATE.stuck_count + 1
                if STATE.stuck_count > 3:
                    STATE.current_path = []
                    STATE.current_target = None
                    STATE.need_charging = False
                    STATE.stuck_count = 0
            else:
                STATE.stuck_count = 0
        STATE.last_location = my_loc
        if STATE.current_target is None or STATE.need_charging:
            available_survivors = []
            for surv in survivors:
                surv_key = (surv.x, surv.y)
                if surv_key in STATE.known_saved_survivors:
                    continue
                if surv_key in STATE.claimed_survivors:
                    if STATE.claimed_survivors[surv_key] != STATE.my_id:
                        continue
                available_survivors.append(surv)
            if not available_survivors:
                move(Direction.CENTER)
                return
            unclaimed = [s for s in available_survivors if (s.x, s.y) not in STATE.claimed_survivors]
            if unclaimed:
                closest = min(unclaimed, key=lambda s: chebyshev_distance(my_loc, s))
            else:
                closest = min(available_survivors, key=lambda s: chebyshev_distance(my_loc, s))
            STATE.current_target = closest
            STATE.current_path = []
            send_message(f"CLAIM|{closest.x}|{closest.y}|{STATE.my_id}", [])
            STATE.claimed_survivors[(closest.x, closest.y)] = STATE.my_id
        if not STATE.current_path:
            direct_path = simple_astar(my_loc, STATE.current_target)
            if direct_path is None:
                surv_key = (STATE.current_target.x, STATE.current_target.y)
                if surv_key in STATE.claimed_survivors:
                    del STATE.claimed_survivors[surv_key]
                STATE.current_target = None
                move(Direction.CENTER)
                return
            path_cost = estimate_path_cost(my_loc, direct_path)
            if path_cost > my_energy * 0.7:
                try:
                    chargers = get_charging_cells()
                    if chargers:
                        best_charger = None
                        best_charger_path = None
                        best_charger_cost = 999999
                        for charger in chargers:
                            charger_path = simple_astar(my_loc, charger)
                            if charger_path:
                                charger_cost = estimate_path_cost(my_loc, charger_path)
                                if charger_cost < my_energy - 10 and charger_cost < best_charger_cost:
                                    best_charger = charger
                                    best_charger_path = charger_path
                                    best_charger_cost = charger_cost
                        if best_charger:
                            STATE.current_path = best_charger_path
                            STATE.need_charging = True
                        else:
                            STATE.current_path = direct_path
                            STATE.need_charging = False
                    else:
                        STATE.current_path = direct_path
                        STATE.need_charging = False
                except:
                    STATE.current_path = direct_path
                    STATE.need_charging = False
            else:
                STATE.current_path = direct_path
                STATE.need_charging = False
        if STATE.current_path:
            next_dir = STATE.current_path.pop(0)
            move(next_dir)
        else:
            STATE.current_target = None
            move(Direction.CENTER)
    except:
        try:
            move(Direction.CENTER)
        except:
            pass
