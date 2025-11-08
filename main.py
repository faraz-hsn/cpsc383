# CPSC 383 Fall 2025
# Assignment 2
# November 7th, 2025

# Mishela Alam - UCID: 30009432 - T05
# Faraz Hosseini - UCID: 30224164 - T05
# Mahdiyar Ashrafioun - UCID: 30232243 -T01
# Mainga Musana - UCID: 30154346 - T01

from aegis_game.stub import *
import heapq

# PATHFINDING FUNCTIONS
# Calculates Chebyshev distance between two points.
def chebyshev_distance(a, b):
    dx = abs(a.x - b.x)
    dy = abs(a.y - b.y)
    return dx if dx > dy else dy


# Returns all possible movement directions
def get_all_directions():
    return [
        Direction.NORTH, Direction.NORTHEAST, Direction.EAST, Direction.SOUTHEAST,
        Direction.SOUTH, Direction.SOUTHWEST, Direction.WEST, Direction.NORTHWEST
    ]

# A-star algorithm
def simple_astar(start, goal):

    # If already at goal, return empty path
    if start.x == goal.x and start.y == goal.y:
        return []

    open_set = []
    heapq.heappush(open_set, (0, 0, start))

    # Track path reconstruction and movement costs
    came_from = {}
    g = {(start.x, start.y): 0}
    c = 0

    # Get node with lowest f_score from priority queue
    while open_set:
        _, _, cur = heapq.heappop(open_set)
        ck = (cur.x, cur.y)

        # Check if reached goal
        if cur.x == goal.x and cur.y == goal.y:

            # Reconstruct path by backtracking
            path = []
            while ck in came_from:
                prev, d = came_from[ck]
                path.append(d)
                ck = (prev.x, prev.y)
            path.reverse()
            return path

        # Explore all possible directions
        dirs = get_all_directions()
        i = 0
        while i < len(dirs):
            d = dirs[i]
            ok = True
            nxt = None
            cell = None

            try:
                # Calculate next position and validate if on map
                nxt = cur.add(d)
                if not on_map(nxt):
                    ok = False
                if ok:
                    cell = get_cell_info_at(nxt)

                    # Avoid dangerous cells
                    if cell.is_killer_cell():
                        ok = False
            except:
                ok = False

            if ok:
                # Get movement cost from cell
                step = 1
                try:
                    mv = int(cell.move_cost)
                    if mv > 0:
                        step = mv
                except:
                    step = 1

                nk = (nxt.x, nxt.y)  # Next location key
                cand = g[ck] + step  # Candidate g_score

                # If found better path to this node
                if (nk not in g) or (cand < g[nk]):
                    came_from[nk] = (cur, d)  # Record path
                    g[nk] = cand

                    # Calculate f_score (g_score + heuristic)
                    f = cand + chebyshev_distance(nxt, goal)
                    c = c + 1
                    heapq.heappush(open_set, (f, c, nxt))  # Add to open set

            i = i + 1

    return None  # No path found

# Estimate total movement cost for a given path.
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
            total = total + 1  # Default cost if error occurs
        i = i + 1

    return total

# Estimate cost to move in a specific direction from current location.
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

# HELPER FUNCTIONS
# Generate a small deterministic bias value based on agent ID and coordinates.
def tiny_split_bias(agent_id, sx, sy):
    try:
        # Create hash using prime numbers for good distribution
        h = (agent_id * 73856093) ^ (sx * 19349663) ^ (sy * 83492791)
    except:
        h = (sx * 19349663) ^ (sy * 83492791)

    v = h % 1000  # Get value in range 0-999
    return float(v) / 2000.0  # Convert to range 0-0.5

# Determine how many agents are needed to clear rubble at given coordinates.
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

    # Fallback: try drone scan if direct cell info fails
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

 #  Tracks navigation, coordination, and energy management for an Agent
class AgentState:

    def __init__(self):
        self.my_id = None
        self.current_path = []
        self.current_target = None  # target location
        self.last_location = None
        self.stuck_count = 0
        self.known_saved = set()  # Set of saved survivor coordinates
        self.need_charging = False  # Flag indicating low energy
        self.charging_now = False  # Flag indicating currently charging
        self.claims_cap = {}  # Tracks agent claims on targets: {(x,y): {max_agents, taken_agents, round}}
        self.help_open = {}  # Tracks help requests: {(x,y): {needed_agents, round}}
        self.energy_init = None
        self.energy_last = None
        self.last_action = "spawn"  # Last action performed
        self.last_spend = 0
        self.stage_charger = None
        self.recharge_ticks_needed = 0
        self.waiting_for_help = False
        self.waiting_location = None
        self.waiting_since = 0
        self.help_requests_sent = set()  # Track sent help requests to avoid duplicates


# Global state instance
STATE = AgentState()

# Register an agent's claim on a target location
def register_claim(x, y, agent_id, k, round_num):


    key = (x, y)
    entry = STATE.claims_cap.get(key)

    if entry is None:
        # Create new claim entry
        entry = {"max": k, "taken": {}, "round": round_num}
    else:
        # Update existing entry
        if k > entry.get("max", 1):
            entry["max"] = k
        entry["round"] = round_num

    # Add this agent to taken set
    taken = entry.get("taken", {})
    taken[agent_id] = 1
    entry["taken"] = taken
    STATE.claims_cap[key] = entry

# Get number of agents that have claimed a target.
def claims_taken_for(x, y):

    entry = STATE.claims_cap.get((x, y))
    if entry is None:
        return 0
    return len(entry.get("taken", {}))

# Get maximum number of agents needed at a target.
def claims_max_for(x, y):

    entry = STATE.claims_cap.get((x, y))
    if entry is None:
        return 0
    return entry.get("max", 1)

# Register a help request for a location.
def register_help(x, y, needed, round_num):

    STATE.help_open[(x, y)] = {"need": needed, "round": round_num}

# Reduce the help needed count at a location when an agent responds.
def consume_help_slot(x, y):

    entry = STATE.help_open.get((x, y))
    if entry is None:
        return

    need = entry.get("need", 0)
    if need > 1:
        entry["need"] = need - 1
        STATE.help_open[(x, y)] = entry
    else:
        # Remove entry when no more help needed
        try:
            del STATE.help_open[(x, y)]
        except:
            pass


# Clean up expired claims and help requests.
def clean_old(current_round, ttl):

    # Clean old claims
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

    # Clean old help requests
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


# Calculate ceiling division (a/b rounded up).
def ceil_div(a, b):

    if a <= 0:
        return 0
    return (a + b - 1) // b


# Find the best charging station to use as staging point for a long journey.
def best_staging_charger(cur_loc, target_loc, cur_energy, action_energy):

    try:
        chargers = get_charging_cells()
    except:
        chargers = []

    if not chargers:
        return None, None, 0

    best = None
    best_path_to_c = None
    best_total_time = None

    i = 0
    while i < len(chargers):
        c_loc = chargers[i]
        # Find path to charger
        p1 = simple_astar(cur_loc, c_loc)
        if p1 is not None:
            # Find path from charger to target
            p2 = simple_astar(c_loc, target_loc)
            if p2 is not None:
                cost1 = estimate_path_cost(cur_loc, p1)  # Cost to reach charger
                cost2 = estimate_path_cost(c_loc, p2)  # Cost from charger to target

                # Calculate energy situation
                energy_at_c = cur_energy - cost1  # Energy remaining when reaching charger
                need_for_leg = cost2 + action_energy  # Energy needed for final leg + action
                deficit = need_for_leg - energy_at_c  # Energy deficit

                # Calculate recharge ticks needed (5 energy per tick)
                ticks = ceil_div(deficit, 5)
                total_time = len(p1) + ticks + len(p2)  # Total time: travel + recharge + travel

                # Select charger with minimum total time
                if (best_total_time is None) or (total_time < best_total_time):
                    best = c_loc
                    best_path_to_c = p1
                    best_total_time = total_time
        i = i + 1

    if best is None:
        return None, None, 0

    # Recalculate final values for the best charger
    cost1b = estimate_path_cost(cur_loc, best_path_to_c)
    p2b = simple_astar(best, target_loc)
    cost2b = estimate_path_cost(best, p2b) if p2b is not None else 0
    energy_at_c2 = cur_energy - cost1b
    need_for_leg2 = cost2b + action_energy
    ticks2 = ceil_div(need_for_leg2 - energy_at_c2, 5)

    return best, best_path_to_c, ticks2


# Select the best target from available survivors and help requests.
def pick_best_target_and_path(loc, energy, survivors, round_num, agent_id):

    candidates = []

    # First consider help requests (higher priority)
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
                ranked = base_cost + bias - 2.0  # Bias toward help requests
                tie1 = chebyshev_distance(loc, Location(hx, hy))
                tie2 = abs(hx - loc.x)
                candidates.append((ranked, tie1, tie2, Location(hx, hy), path, base_cost, 1, True))
        j = j + 1

    # Then consider regular survivors
    i = 0
    while i < len(survivors):
        s = survivors[i]
        # Skip already saved survivors
        if (s.x, s.y) in STATE.known_saved:
            i = i + 1
            continue

        # Check if target is already fully claimed
        kreq = claims_max_for(s.x, s.y)
        if kreq <= 0:
            kreq = required_agents_for_xy(s.x, s.y)
        taken = claims_taken_for(s.x, s.y)
        if kreq > 0 and taken >= kreq:
            i = i + 1
            continue

        # Find path to survivor
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

    # Sort by primary cost, then tie-breakers
    candidates.sort(key=lambda t: (t[0], t[1], t[2]))
    best = candidates[0]

    target_loc = best[3]
    path = best[4]
    true_cost = best[5]
    kreq = best[6]
    from_help = best[7]

    # Register claim and notify other agents
    if from_help:
        consume_help_slot(target_loc.x, target_loc.y)
        register_claim(target_loc.x, target_loc.y, agent_id, 2, round_num)
        try:
            send_message("CLAIM2|" + str(target_loc.x) + "|" + str(target_loc.y) + "|" + str(agent_id) + "|" + str(
                2) + "|" + str(round_num), [])
        except:
            pass
    else:
        if kreq <= 0:
            kreq = required_agents_for_xy(target_loc.x, target_loc.y)
        register_claim(target_loc.x, target_loc.y, agent_id, kreq, round_num)
        try:
            send_message("CLAIM2|" + str(target_loc.x) + "|" + str(target_loc.y) + "|" + str(agent_id) + "|" + str(
                kreq) + "|" + str(round_num), [])
        except:
            pass

    return target_loc, path, true_cost, kreq, from_help


# MAIN AGENT
# Coordinates movement, digging, saving, charging, and agent cooperation.
def think():

    global STATE

    try:
        # Get current game state
        t = get_round_number()
        loc = get_location()
        energy = get_energy_level()

        # Initializes agent state on first call
        if STATE.my_id is None:
            STATE.my_id = get_id()
            STATE.energy_init = energy
            STATE.energy_last = energy
            move(Direction.CENTER)
            return

        # Tracks energy usage
        if STATE.energy_last is None:
            STATE.energy_last = energy
        delta = STATE.energy_last - energy
        STATE.last_spend = delta
        STATE.energy_last = energy
        STATE.last_action = "idle"

        # Processes incoming messages from other agents
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

                # Processes claim messages
                if len(parts) >= 6 and parts[0] == "CLAIM2":
                    sx = int(parts[1]);
                    sy = int(parts[2])
                    ag = int(parts[3]);
                    kreq = int(parts[4]);
                    r = int(parts[5])
                    register_claim(sx, sy, ag, kreq, r)

                # Processes help request messages
                elif len(parts) >= 5 and parts[0] == "HELP2":
                    hx = int(parts[1]);
                    hy = int(parts[2])
                    need = int(parts[3]);
                    r = int(parts[4])
                    register_help(hx, hy, need, r)

                m = m + 1
        except:
            pass

        # Cleans up old coordination data
        clean_old(t, 8)

        # Gets list of survivors
        try:
            survivors = get_survs()
        except:
            survivors = []

        # Checks if standing on a survivor that needs saving
        try:
            cell_here = get_cell_info_at(loc)
            top_here = cell_here.top_layer
            if isinstance(top_here, Survivor):
                save()  # Perform save action
                STATE.last_action = "save"
                STATE.known_saved.add((loc.x, loc.y))  # Mark as saved
                # Reset navigation state
                STATE.current_target = None
                STATE.current_path = []
                STATE.need_charging = False
                STATE.stage_charger = None
                STATE.recharge_ticks_needed = 0
                STATE.waiting_for_help = False
                return
        except:
            pass

        # Checks if standing on rubble that needs digging
        try:
            cell_here = get_cell_info_at(loc)
            top_here = cell_here.top_layer
            if isinstance(top_here, Rubble):
                req = 1
                try:
                    req = int(top_here.agents_required)
                except:
                    req = 1

                # Single-agent rubble
                if req == 1:
                    dig()
                    STATE.last_action = "dig"
                    STATE.waiting_for_help = False
                    return
                else:
                    # Multi-agent rubble - check if enough agents present
                    count_here = 1
                    try:
                        count_here = len(cell_here.agents)
                    except:
                        count_here = 1

                    if count_here >= req:
                        # Enough agents - start digging
                        dig()
                        STATE.last_action = "dig"
                        STATE.waiting_for_help = False
                        return
                    else:
                        # Not enough agents - request help
                        help_key = (loc.x, loc.y)
                        if help_key not in STATE.help_requests_sent:
                            STATE.help_requests_sent.add(help_key)
                            remaining = req - count_here
                            try:
                                # Broadcast help request and claim
                                send_message(
                                    "HELP2|" + str(loc.x) + "|" + str(loc.y) + "|" + str(remaining) + "|" + str(t), [])
                                send_message(
                                    "CLAIM2|" + str(loc.x) + "|" + str(loc.y) + "|" + str(STATE.my_id) + "|" + str(
                                        req) + "|" + str(t), [])
                            except:
                                pass
                            register_help(loc.x, loc.y, remaining, t)
                            register_claim(loc.x, loc.y, STATE.my_id, req, t)

                        # Waits for help, but timeout after 5 rounds
                        if t - STATE.waiting_since > 5:
                            STATE.waiting_for_help = False
                            STATE.current_target = None
                            STATE.current_path = []
                        else:
                            STATE.waiting_for_help = True
                            STATE.waiting_location = loc
                            if STATE.waiting_since == 0:
                                STATE.waiting_since = t
                            move(Direction.CENTER)  # Stay in place
                            STATE.last_action = "hold"
                            return
        except:
            pass

        # Handles charging station logic
        # Only does charging when necessary, and only when the agent needs more energy
        try:
            cell_here = get_cell_info_at(loc)
            if "CHARGING" in str(cell_here.type).upper():
                if STATE.recharge_ticks_needed > 0:
                    # Continue recharging
                    recharge()
                    STATE.recharge_ticks_needed = STATE.recharge_ticks_needed - 1
                    STATE.last_action = "recharge"
                    return
                else:
                    # Finished charging - resume path to target
                    if STATE.stage_charger is not None and STATE.current_target is not None:
                        p2 = simple_astar(loc, STATE.current_target)
                        if p2 is not None:
                            STATE.current_path = p2
                        else:
                            STATE.current_path = []
                        STATE.stage_charger = None
        except:
            pass

        # Timeout for waiting for help
        if STATE.waiting_for_help and t - STATE.waiting_since > 5:
            STATE.waiting_for_help = False
            STATE.current_target = None
            STATE.current_path = []

        # If no survivors and no current target, stay put
        if not survivors and STATE.current_target is None:
            move(Direction.CENTER)
            STATE.last_action = "hold"
            return

        # Stuck detection - reset if not moving for several ticks
        if STATE.last_location is not None:
            if (STATE.last_location.x == loc.x) and (STATE.last_location.y == loc.y):
                STATE.stuck_count = STATE.stuck_count + 1
                if STATE.stuck_count > 3:
                    # Reset navigation state if stuck
                    STATE.current_path = []
                    STATE.need_charging = False
                    STATE.stuck_count = 0
                    STATE.waiting_for_help = False
                    STATE.current_target = None
            else:
                STATE.stuck_count = 0

        STATE.last_location = loc

        # Selects new target if none current or waiting for help
        if STATE.current_target is None or STATE.waiting_for_help:
            s, path, cost, kreq, used_help = pick_best_target_and_path(loc, energy, survivors, t, STATE.my_id)
            if s is None:
                move(Direction.CENTER)
                STATE.last_action = "hold"
                return

            ACTION_ENERGY = 1  # Energy cost for final action (dig/save)
            total_need = cost + ACTION_ENERGY

            if total_need <= energy:
                # Enough energy - go directly to target
                STATE.current_target = s
                STATE.current_path = path
                STATE.stage_charger = None
                STATE.recharge_ticks_needed = 0
                STATE.waiting_for_help = False
            else:
                # Not enough energy - find staging charger
                c_loc, p_to_c, ticks = best_staging_charger(loc, s, energy, ACTION_ENERGY)
                STATE.current_target = s
                if c_loc is not None and p_to_c is not None:
                    STATE.current_path = p_to_c
                    STATE.stage_charger = c_loc
                    STATE.recharge_ticks_needed = ticks
                    STATE.waiting_for_help = False
                else:
                    # No charger found - proceed anyway
                    STATE.current_path = path
                    STATE.stage_charger = None
                    STATE.recharge_ticks_needed = 0
                    STATE.waiting_for_help = False

        # Recalculates path if current path is empty but target exists
        if not STATE.current_path and STATE.current_target is not None:
            ACTION_ENERGY = 1
            direct = simple_astar(loc, STATE.current_target)
            if direct is not None:
                cost = estimate_path_cost(loc, direct)
                total_need = cost + ACTION_ENERGY

                if total_need <= energy:
                    STATE.current_path = direct
                    STATE.stage_charger = None
                    STATE.recharge_ticks_needed = 0
                else:
                    # Find charger for energy boost
                    c_loc, p_to_c, ticks = best_staging_charger(loc, STATE.current_target, energy, ACTION_ENERGY)
                    if c_loc is not None and p_to_c is not None:
                        STATE.current_path = p_to_c
                        STATE.stage_charger = c_loc
                        STATE.recharge_ticks_needed = ticks
                    else:
                        STATE.current_path = direct
                        STATE.stage_charger = None
                        STATE.recharge_ticks_needed = 0
            else:
                STATE.current_target = None

        # Final fallback: try to find any target
        if STATE.current_target is None:
            if not STATE.current_path:
                s, path, cost, kreq, used_help = pick_best_target_and_path(loc, energy, survivors, t, STATE.my_id)
                if s is None:
                    move(Direction.CENTER)
                    STATE.last_action = "hold"
                    return

                ACTION_ENERGY = 1
                total_need = cost + ACTION_ENERGY

                if total_need <= energy:
                    STATE.current_target = s
                    STATE.current_path = path
                    STATE.stage_charger = None
                    STATE.recharge_ticks_needed = 0
                else:
                    c_loc, p_to_c, ticks = best_staging_charger(loc, s, energy, ACTION_ENERGY)
                    STATE.current_target = s
                    if c_loc is not None and p_to_c is not None:
                        STATE.current_path = p_to_c
                        STATE.stage_charger = c_loc
                        STATE.recharge_ticks_needed = ticks
                    else:
                        STATE.current_path = path
                        STATE.stage_charger = None
                        STATE.recharge_ticks_needed = 0

        # Executes next movement step if path exists
        if STATE.current_path:
            d = STATE.current_path[0]
            STATE.current_path = STATE.current_path[1:]
            STATE.last_action = "move"
            move(d)
        else:
            # No movement possible - stay in place
            move(Direction.CENTER)
            STATE.last_action = "hold"

    except:
        # Emergency fallback - stay in place on error
        try:
            move(Direction.CENTER)
            STATE.last_action = "hold"
        except:
            pass
