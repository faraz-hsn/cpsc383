"""
Course: CPSC 383
Name:
Semester: Fall 2025
Date: November 6, 2025


Assignment 2: Multi-Agent Cooperative System - IMPROVED VERSION
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


def find_best_charger(my_loc, my_energy):
    """Find the best reachable charging station"""
    try:
        chargers = get_charging_cells()
        if not chargers:
            return None, None

        best_charger = None
        best_charger_path = None
        best_score = 999999

        for charger in chargers:
            charger_path = simple_astar(my_loc, charger)
            if charger_path:
                charger_cost = estimate_path_cost(my_loc, charger_path)
                # Must be reachable with current energy (with safety margin)
                if charger_cost < my_energy - 15:
                    # Prefer closer chargers
                    distance = chebyshev_distance(my_loc, charger)
                    score = distance + charger_cost * 0.5
                    if score < best_score:
                        best_charger = charger
                        best_charger_path = charger_path
                        best_score = score

        return best_charger, best_charger_path
    except:
        return None, None


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
        self.claimed_survivors = {}  # survivor_loc -> (agent_id, round_claimed)
        self.unreachable_survivors = set()  # Survivors with no path
        self.need_charging = False
        self.charging_now = False
        self.last_energy_check = 0
        self.agent_positions = {}  # agent_id -> (x, y, round)
        self.help_requests = {}  # location -> (round, agents_needed)
        self.last_broadcast_round = 0


STATE = AgentState()


# ============================================================================
# Message Handling
# ============================================================================

def process_messages(round_num):
    """Process all messages and update shared knowledge"""
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
                    # Remove from claimed if it was there
                    if (x, y) in STATE.claimed_survivors:
                        del STATE.claimed_survivors[(x, y)]
                except:
                    pass

            # Track claimed survivors
            elif msg_type == "CLAIM":
                try:
                    x, y = int(parts[1]), int(parts[2])
                    agent_id = int(parts[3])
                    surv_key = (x, y)

                    # Only update if not already claimed or if older claim
                    if surv_key not in STATE.claimed_survivors:
                        STATE.claimed_survivors[surv_key] = (agent_id, round_num)
                    else:
                        existing_agent, existing_round = STATE.claimed_survivors[surv_key]
                        # Tie-break: lower agent ID wins
                        if agent_id < existing_agent:
                            STATE.claimed_survivors[surv_key] = (agent_id, round_num)
                except:
                    pass

            # Track unreachable survivors
            elif msg_type == "UNREACHABLE":
                try:
                    x, y = int(parts[1]), int(parts[2])
                    agent_id = int(parts[3])
                    surv_key = (x, y)
                    STATE.unreachable_survivors.add(surv_key)
                    # Remove claim if this agent had claimed it
                    if surv_key in STATE.claimed_survivors:
                        claimed_agent, _ = STATE.claimed_survivors[surv_key]
                        if claimed_agent == agent_id:
                            del STATE.claimed_survivors[surv_key]
                except:
                    pass

            # Track agent positions
            elif msg_type == "POS":
                try:
                    agent_id = int(parts[1])
                    x, y = int(parts[2]), int(parts[3])
                    STATE.agent_positions[agent_id] = (x, y, round_num)
                except:
                    pass

            # Help requests for heavy rubble
            elif msg_type == "HELP":
                try:
                    x, y = int(parts[1]), int(parts[2])
                    agents_needed = int(parts[3]) if len(parts) > 3 else 2
                    STATE.help_requests[(x, y)] = (round_num, agents_needed)
                except:
                    pass

            # Recharged message
            elif msg_type == "RECHARGED":
                try:
                    agent_id = int(parts[1])
                    x, y = int(parts[2]), int(parts[3])
                    log(f"[Agent {STATE.my_id}] Agent {agent_id} recharged at ({x},{y})")
                except:
                    pass

    except Exception as e:
        log(f"[Agent {STATE.my_id}] Error processing messages: {e}")


def broadcast_position(my_loc, round_num):
    """Periodically broadcast position for coordination"""
    # Broadcast every 5 rounds
    if round_num - STATE.last_broadcast_round >= 5:
        send_message(f"POS|{STATE.my_id}|{my_loc.x}|{my_loc.y}", [])
        STATE.last_broadcast_round = round_num


# ============================================================================
# Target Selection
# ============================================================================

def select_best_target(my_loc, survivors, my_energy, round_num):
    """
    Select the best survivor to target with improved coordination.
    Returns (target, is_help_request)
    """
    # Filter survivors
    available_survivors = []

    for surv in survivors:
        surv_key = (surv.x, surv.y)

        # Skip if saved
        if surv_key in STATE.known_saved_survivors:
            continue

        # Skip if unreachable
        if surv_key in STATE.unreachable_survivors:
            continue

        # Check if claimed by someone else
        if surv_key in STATE.claimed_survivors:
            claimed_agent, claim_round = STATE.claimed_survivors[surv_key]
            # Only skip if claimed by another agent recently (within 20 rounds)
            if claimed_agent != STATE.my_id and (round_num - claim_round) < 20:
                continue

        available_survivors.append(surv)

    if not available_survivors:
        return None, False

    # PRIORITY 1: Check for help requests (heavy rubble)
    for loc, (req_round, agents_needed) in list(STATE.help_requests.items()):
        # Only respond to recent requests (within 10 rounds)
        if round_num - req_round > 10:
            del STATE.help_requests[loc]
            continue

        # Check if this location has a survivor
        for surv in available_survivors:
            if surv.x == loc[0] and surv.y == loc[1]:
                # Check if we can reach it
                path = simple_astar(my_loc, surv)
                if path is not None:
                    path_cost = estimate_path_cost(my_loc, path)
                    # Check energy with charging consideration
                    if path_cost < my_energy * 0.6:
                        log(f"[Agent {STATE.my_id}] Responding to help request at ({surv.x},{surv.y})")
                        return surv, True

    # PRIORITY 2: Unclaimed survivors (best for distribution)
    unclaimed = []
    for surv in available_survivors:
        surv_key = (surv.x, surv.y)
        if surv_key not in STATE.claimed_survivors:
            # Check if reachable
            path = simple_astar(my_loc, surv)
            if path is not None:
                unclaimed.append((surv, path))

    if unclaimed:
        # Score by: distance + path_cost, prefer closer and cheaper
        # Add tie-breaker so different agents pick different survivors
        best = None
        best_score = 999999

        for surv, path in unclaimed:
            distance = chebyshev_distance(my_loc, surv)
            path_cost = estimate_path_cost(my_loc, path)

            # Factor in energy - penalize expensive paths if low on energy
            energy_factor = 1.0
            if path_cost > my_energy * 0.5:
                energy_factor = 2.0

            # Add agent-specific tie-breaker
            tie_breaker = (STATE.my_id * 0.01)
            score = distance + path_cost * 0.5 * energy_factor + tie_breaker

            if score < best_score:
                best = surv
                best_score = score

        if best:
            return best, False

    # PRIORITY 3: Claimed survivors (old claims or our own)
    claimed_available = []
    for surv in available_survivors:
        surv_key = (surv.x, surv.y)
        if surv_key in STATE.claimed_survivors:
            claimed_agent, claim_round = STATE.claimed_survivors[surv_key]
            # Take old claims or our own claims
            if claimed_agent == STATE.my_id or (round_num - claim_round) >= 20:
                path = simple_astar(my_loc, surv)
                if path is not None:
                    claimed_available.append((surv, path))

    if claimed_available:
        # Pick closest
        best = min(claimed_available,
                   key=lambda sp: chebyshev_distance(my_loc, sp[0]))
        return best[0], False

    return None, False


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
                send_message(f"POS|{STATE.my_id}|{my_loc.x},{my_loc.y}", [])
                return

        # Get survivors
        try:
            survivors = get_survs()
        except:
            survivors = []

        log(f"[Agent {STATE.my_id}] R{round_num} Pos:({my_loc.x},{my_loc.y}) Energy:{my_energy} Survs:{len(survivors)}")

        # Process messages from other agents
        process_messages(round_num)

        # Broadcast position periodically
        broadcast_position(my_loc, round_num)

        # =================================================================
        # CHECK IF ON SURVIVOR - SAVE IT
        # =================================================================
        try:
            cell = get_cell_info_at(my_loc)
            top_layer = cell.top_layer

            if isinstance(top_layer, Survivor):
                log(f"[Agent {STATE.my_id}] On survivor - SAVING!")
                save()
                STATE.known_saved_survivors.add((my_loc.x, my_loc.y))
                send_message(f"SAVED|{my_loc.x}|{my_loc.y}", [])

                # Clear current target and look for next one
                STATE.current_target = None
                STATE.current_path = []
                STATE.need_charging = False
                STATE.stuck_count = 0

                # Remove from claimed if it was there
                if (my_loc.x, my_loc.y) in STATE.claimed_survivors:
                    del STATE.claimed_survivors[(my_loc.x, my_loc.y)]

                log(f"[Agent {STATE.my_id}] Survivor saved! Looking for next target...")
                return
        except Exception as e:
            log(f"[Agent {STATE.my_id}] Error checking survivor: {e}")

        # =================================================================
        # CHECK IF ON RUBBLE - DIG IT
        # =================================================================
        try:
            cell = get_cell_info_at(my_loc)
            top_layer = cell.top_layer

            if isinstance(top_layer, Rubble):
                agents_required = top_layer.agents_required

                if agents_required == 1:
                    log(f"[Agent {STATE.my_id}] Digging light rubble")
                    dig()
                    return

                elif agents_required >= 2:
                    agents_here = cell.agents
                    log(f"[Agent {STATE.my_id}] Heavy rubble - {len(agents_here)}/{agents_required} agents here")

                    if len(agents_here) >= agents_required:
                        log(f"[Agent {STATE.my_id}] Digging heavy rubble with team")
                        dig()
                        return
                    else:
                        log(f"[Agent {STATE.my_id}] Waiting for team members ({len(agents_here)}/{agents_required})")
                        send_message(f"HELP|{my_loc.x}|{my_loc.y}|{agents_required}", [])
                        move(Direction.CENTER)
                        return
        except Exception as e:
            log(f"[Agent {STATE.my_id}] Error checking rubble: {e}")

        # =================================================================
        # CHECK IF ON CHARGING STATION
        # =================================================================
        try:
            cell = get_cell_info_at(my_loc)
            cell_type_str = str(cell.type)

            if 'CHARGING' in cell_type_str.upper():
                # Charge until reasonably full (80%)
                if my_energy < 80:
                    log(f"[Agent {STATE.my_id}] Recharging ({my_energy} -> {my_energy + 5})")
                    recharge()
                    STATE.charging_now = True
                    return
                else:
                    # Done charging - clear flags and return to replan next round
                    log(f"[Agent {STATE.my_id}] Fully charged ({my_energy}), leaving charger")
                    STATE.need_charging = False
                    STATE.charging_now = False
                    STATE.current_path = []  # Clear path so we replan
                    STATE.current_target = None  # Clear target to select new one
                    send_message(f"RECHARGED|{STATE.my_id}|{my_loc.x}|{my_loc.y}", [])
                    move(Direction.CENTER)  # Move to leave the charger
                    return
        except:
            pass

        # =================================================================
        # CHECK FOR CRITICAL LOW ENERGY
        # =================================================================
        if my_energy < 30 and not STATE.charging_now:
            log(f"[Agent {STATE.my_id}] CRITICAL LOW ENERGY! Seeking charger immediately")
            charger, charger_path = find_best_charger(my_loc, my_energy)

            if charger and charger_path:
                log(f"[Agent {STATE.my_id}] Emergency route to charger at ({charger.x},{charger.y})")
                STATE.current_path = charger_path
                STATE.current_target = None
                STATE.need_charging = True

                if STATE.current_path:
                    next_dir = STATE.current_path.pop(0)
                    move(next_dir)
                    return
            else:
                log(f"[Agent {STATE.my_id}] NO REACHABLE CHARGER - Moving carefully")
                move(Direction.CENTER)
                return

        # If no survivors, wait
        if not survivors:
            log(f"[Agent {STATE.my_id}] No survivors - mission complete")
            move(Direction.CENTER)
            return

        # =================================================================
        # DETECT IF STUCK - CLEAR PATH AND REPLAN
        # =================================================================
        if STATE.last_location:
            if STATE.last_location.x == my_loc.x and STATE.last_location.y == my_loc.y:
                STATE.stuck_count = STATE.stuck_count + 1
                if STATE.stuck_count > 2:
                    log(f"[Agent {STATE.my_id}] STUCK - replanning completely")

                    # Unclaim current target if we have one
                    if STATE.current_target:
                        surv_key = (STATE.current_target.x, STATE.current_target.y)
                        if surv_key in STATE.claimed_survivors:
                            claimed_agent, _ = STATE.claimed_survivors[surv_key]
                            if claimed_agent == STATE.my_id:
                                del STATE.claimed_survivors[surv_key]

                    STATE.current_path = []
                    STATE.current_target = None
                    STATE.need_charging = False
                    STATE.stuck_count = 0
            else:
                STATE.stuck_count = 0
        STATE.last_location = my_loc

        # =================================================================
        # SELECT TARGET SURVIVOR
        # =================================================================
        if STATE.current_target is None or STATE.need_charging:
            target, is_help = select_best_target(my_loc, survivors, my_energy, round_num)

            if target is None:
                log(f"[Agent {STATE.my_id}] No available targets - waiting")
                move(Direction.CENTER)
                return

            STATE.current_target = target
            STATE.current_path = []

            # Claim this survivor (unless responding to help request)
            if not is_help:
                send_message(f"CLAIM|{target.x}|{target.y}|{STATE.my_id}", [])
                STATE.claimed_survivors[(target.x, target.y)] = (STATE.my_id, round_num)

            log(f"[Agent {STATE.my_id}] New target: ({target.x},{target.y}){' [HELP]' if is_help else ''}")

        # =================================================================
        # PLAN PATH WITH ENERGY MANAGEMENT
        # =================================================================
        if not STATE.current_path:
            log(f"[Agent {STATE.my_id}] Planning path to ({STATE.current_target.x},{STATE.current_target.y})")

            # Check if target is still reachable
            direct_path = simple_astar(my_loc, STATE.current_target)

            if direct_path is None:
                log(f"[Agent {STATE.my_id}] No path to target - marking unreachable")

                # Unclaim this survivor and mark unreachable
                surv_key = (STATE.current_target.x, STATE.current_target.y)
                STATE.unreachable_survivors.add(surv_key)

                if surv_key in STATE.claimed_survivors:
                    claimed_agent, _ = STATE.claimed_survivors[surv_key]
                    if claimed_agent == STATE.my_id:
                        del STATE.claimed_survivors[surv_key]

                send_message(f"UNREACHABLE|{surv_key[0]}|{surv_key[1]}|{STATE.my_id}", [])
                STATE.current_target = None
                move(Direction.CENTER)
                return

            path_cost = estimate_path_cost(my_loc, direct_path)
            log(f"[Agent {STATE.my_id}] Path cost: {path_cost}, Current energy: {my_energy}")

            #CHARGING STRATEGY: Need charging if:
            # 1. Path cost > 50% of current energy (more conservative), OR
            # 2. Current energy < 50 and path cost > 25% of energy
            needs_charge = (path_cost > my_energy * 0.5) or (my_energy < 50 and path_cost > my_energy * 0.25)

            if needs_charge and not STATE.charging_now:
                log(f"[Agent {STATE.my_id}] Need charging before target!")

                charger, charger_path = find_best_charger(my_loc, my_energy)

                if charger and charger_path:
                    log(f"[Agent {STATE.my_id}] Routing to charger at ({charger.x},{charger.y})")
                    STATE.current_path = charger_path
                    STATE.need_charging = True
                else:
                    log(f"[Agent {STATE.my_id}] No reachable charger - trying direct (risky!)")
                    STATE.current_path = direct_path
                    STATE.need_charging = False
            else:
                # Have enough energy for direct path
                STATE.current_path = direct_path
                STATE.need_charging = False

        # =================================================================
        # EXECUTE NEXT MOVE
        # =================================================================
        if STATE.current_path:
            next_dir = STATE.current_path.pop(0)
            log(f"[Agent {STATE.my_id}] Moving {next_dir} (steps left: {len(STATE.current_path)})")
            move(next_dir)
        else:
            log(f"[Agent {STATE.my_id}] No path - waiting")
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

