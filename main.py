"""
Course: CPSC 383
Name: 
Semester: Fall 2025
Date: November 6, 2025


Assignment 2: Multi-Agent Cooperative System
"""

from aegis_game.stub import *
import heapq


# ============================================================================
# Helper Functions
# ============================================================================

def chebyshev_distance(loc1, loc2):
    """Calculate Chebyshev distance (8-connected grid)"""
    return max(abs(loc1.x - loc2.x), abs(loc1.y - loc2.y))


def get_all_directions():
    """Return list of 8 directions in priority order"""
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


# ============================================================================
# A* Pathfinding
# ============================================================================

def simple_astar(start, goal):
    """
    Simple A* pathfinding returning list of directions.
    Returns None if no path found.
    """
    if start.x == goal.x and start.y == goal.y:
        return []

    # Priority queue: (f_score, counter, location)
    open_set = []
    heapq.heappush(open_set, (0, 0, start))
    came_from = {}
    g_score = {(start.x, start.y): 0}
    counter = 0

    while open_set:
        _, _, current = heapq.heappop(open_set)
        current_key = (current.x, current.y)

        # Goal reached
        if current.x == goal.x and current.y == goal.y:
            # Reconstruct path
            path = []
            while current_key in came_from:
                prev_loc, direction = came_from[current_key]
                path.append(direction)
                current_key = (prev_loc.x, prev_loc.y)
            path.reverse()
            return path

        # Explore neighbors
        for direction in get_all_directions():
            try:
                neighbor = current.add(direction)

                if not on_map(neighbor):
                    continue

                cell_info = get_cell_info_at(neighbor)

                # Skip killer cells
                if cell_info.is_killer_cell():
                    continue

                # Get move cost
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
    """Estimate total energy cost of a path"""
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


# ============================================================================
# Agent State
# ============================================================================

class AgentState:
    """Tracks agent state across rounds"""
    def __init__(self):
        self.my_id = None
        self.current_path = []
        self.current_target = None
        self.last_location = None
        self.stuck_count = 0
        self.known_saved_survivors = set()
        self.claimed_survivors = {}  # survivor_loc -> agent_id
        self.need_charging = False
        self.charging_now = False


STATE = AgentState()


# ============================================================================
# Main Think Function
# ============================================================================

def think():
    """Main decision function with improved coordination"""
    global STATE

    try:
        # Get basic info
        round_num = get_round_number()
        my_loc = get_location()
        my_energy = get_energy_level()

        # Initialize on first round
        if STATE.my_id is None:
            STATE.my_id = get_id()
            log(f"[Agent {STATE.my_id}] Starting, Energy: {my_energy}")

            if round_num == 1:
                move(Direction.CENTER)
                send_message(f"READY|{STATE.my_id}|{my_loc.x},{my_loc.y}", [])
                return

        # Get survivors
        try:
            survivors = get_survs()
        except:
            survivors = []

        log(f"[Agent {STATE.my_id}] R{round_num} Pos:({my_loc.x},{my_loc.y}) Energy:{my_energy} Survs:{len(survivors)}")

        # Read messages - update shared knowledge
        try:
            messages = read_messages()
            for msg in messages:
                parts = msg.message.split("|")
                if len(parts) < 2:
                    continue

                msg_type = parts[0]

                # Track saved survivors
                if msg_type == "SAVED":
                    try:
                        x, y = int(parts[1]), int(parts[2])
                        STATE.known_saved_survivors.add((x, y))
                        log(f"[Agent {STATE.my_id}] Agent {msg.sender_id} saved ({x},{y})")
                    except:
                        pass

                # Track claimed survivors
                elif msg_type == "CLAIM":
                    try:
                        x, y = int(parts[1]), int(parts[2])
                        agent_id = int(parts[3])
                        STATE.claimed_survivors[(x, y)] = agent_id
                    except:
                        pass
        except:
            pass

        # Check if standing on survivor
        try:
            cell = get_cell_info_at(my_loc)
            top_layer = cell.top_layer

            if isinstance(top_layer, Survivor):
                log(f"[Agent {STATE.my_id}] On survivor - SAVING!")
                save()
                STATE.known_saved_survivors.add((my_loc.x, my_loc.y))
                STATE.current_target = None
                STATE.current_path = []
                STATE.need_charging = False
                send_message(f"SAVED|{my_loc.x}|{my_loc.y}", [])
                return
        except Exception as e:
            log(f"[Agent {STATE.my_id}] Error checking survivor: {e}")

        # Check if standing on rubble
        try:
            cell = get_cell_info_at(my_loc)
            top_layer = cell.top_layer

            if isinstance(top_layer, Rubble):
                agents_required = top_layer.agents_required

                if agents_required == 1:
                    log(f"[Agent {STATE.my_id}] Digging light rubble")
                    dig()
                    return
                elif agents_required == 2:
                    agents_here = cell.agents
                    log(f"[Agent {STATE.my_id}] Heavy rubble - {len(agents_here)} agents here")

                    if len(agents_here) >= 2:
                        log(f"[Agent {STATE.my_id}] Digging heavy rubble with partner")
                        dig()
                        return
                    else:
                        log(f"[Agent {STATE.my_id}] Waiting for partner")
                        send_message(f"HELP|{my_loc.x}|{my_loc.y}", [])
                        move(Direction.CENTER)
                        return
        except Exception as e:
            log(f"[Agent {STATE.my_id}] Error checking rubble: {e}")

        # Check if on charging station
        try:
            cell = get_cell_info_at(my_loc)
            cell_type_str = str(cell.type)

            if 'CHARGING' in cell_type_str.upper():
                if my_energy < 80:  # Charge until reasonably full
                    log(f"[Agent {STATE.my_id}] Recharging ({my_energy} -> {my_energy + 5})")
                    recharge()
                    STATE.charging_now = True
                    return
                else:
                    log(f"[Agent {STATE.my_id}] Fully charged, resuming mission")
                    STATE.need_charging = False
                    STATE.charging_now = False
                    STATE.current_path = []  # Replan from here
        except:
            pass

        # If no survivors, wait
        if not survivors:
            log(f"[Agent {STATE.my_id}] No survivors - waiting")
            move(Direction.CENTER)
            return

        # Detect if stuck - clear path and replan
        if STATE.last_location:
            if STATE.last_location.x == my_loc.x and STATE.last_location.y == my_loc.y:
                STATE.stuck_count = STATE.stuck_count + 1
                if STATE.stuck_count > 3:
                    log(f"[Agent {STATE.my_id}] STUCK - replanning completely")
                    STATE.current_path = []
                    STATE.current_target = None
                    STATE.need_charging = False
                    STATE.stuck_count = 0
            else:
                STATE.stuck_count = 0
        STATE.last_location = my_loc

        # IMPROVED: Choose target with task distribution
        if STATE.current_target is None or STATE.need_charging:
            # Filter out saved and claimed survivors
            available_survivors = []
            for surv in survivors:
                surv_key = (surv.x, surv.y)

                # Skip if saved
                if surv_key in STATE.known_saved_survivors:
                    continue

                # Skip if claimed by another agent (unless stuck)
                if surv_key in STATE.claimed_survivors:
                    if STATE.claimed_survivors[surv_key] != STATE.my_id:
                        continue

                available_survivors.append(surv)

            if not available_survivors:
                log(f"[Agent {STATE.my_id}] All survivors saved/claimed - waiting")
                move(Direction.CENTER)
                return

            # IMPROVED: Prefer survivors NO ONE is going to
            unclaimed = [s for s in available_survivors if (s.x, s.y) not in STATE.claimed_survivors]
            if unclaimed:
                # Pick closest unclaimed survivor
                closest = min(unclaimed, key=lambda s: chebyshev_distance(my_loc, s))
            else:
                # Pick closest available
                closest = min(available_survivors, key=lambda s: chebyshev_distance(my_loc, s))

            STATE.current_target = closest
            STATE.current_path = []

            # Claim this survivor
            send_message(f"CLAIM|{closest.x}|{closest.y}|{STATE.my_id}", [])
            STATE.claimed_survivors[(closest.x, closest.y)] = STATE.my_id

            log(f"[Agent {STATE.my_id}] New target: ({closest.x},{closest.y})")

        # Plan path if needed
        if not STATE.current_path:
            log(f"[Agent {STATE.my_id}] Planning path to ({STATE.current_target.x},{STATE.current_target.y})")

            # IMPROVED: Check if we need charging FIRST
            direct_path = simple_astar(my_loc, STATE.current_target)

            if direct_path is None:
                log(f"[Agent {STATE.my_id}] No path to target - unclaiming and finding new target")
                # Unclaim this survivor
                surv_key = (STATE.current_target.x, STATE.current_target.y)
                if surv_key in STATE.claimed_survivors:
                    del STATE.claimed_survivors[surv_key]
                STATE.current_target = None
                move(Direction.CENTER)
                return

            path_cost = estimate_path_cost(my_loc, direct_path)
            log(f"[Agent {STATE.my_id}] Direct path cost: {path_cost}, Current energy: {my_energy}")

            # IMPROVED: Need charging if path cost > 70% of current energy
            if path_cost > my_energy * 0.7:
                log(f"[Agent {STATE.my_id}] Need charging! ({path_cost} > {my_energy * 0.7})")

                try:
                    chargers = get_charging_cells()
                    if chargers:
                        # Find nearest reachable charger
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
                            log(f"[Agent {STATE.my_id}] Routing to charger at ({best_charger.x},{best_charger.y})")
                            STATE.current_path = best_charger_path
                            STATE.need_charging = True
                        else:
                            log(f"[Agent {STATE.my_id}] No reachable charger - trying direct path anyway")
                            STATE.current_path = direct_path
                            STATE.need_charging = False
                    else:
                        log(f"[Agent {STATE.my_id}] No chargers available - trying direct path")
                        STATE.current_path = direct_path
                        STATE.need_charging = False
                except Exception as e:
                    log(f"[Agent {STATE.my_id}] Error finding charger: {e}")
                    STATE.current_path = direct_path
                    STATE.need_charging = False
            else:
                # Have enough energy
                STATE.current_path = direct_path
                STATE.need_charging = False

        # Execute next step
        if STATE.current_path:
            next_dir = STATE.current_path.pop(0)
            log(f"[Agent {STATE.my_id}] Moving {next_dir} (steps left: {len(STATE.current_path)})")
            move(next_dir)
        else:
            log(f"[Agent {STATE.my_id}] No path - replanning")
            STATE.current_target = None
            move(Direction.CENTER)

    except Exception as e:
        import traceback
        log(f"[Agent {STATE.my_id if STATE.my_id else '?'}] ERROR: {e}")
        for line in traceback.format_exc().splitlines():
            log(line)
        try:
            move(Direction.CENTER)
        except:
            pass

