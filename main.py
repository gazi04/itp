import traci
import sumolib
from collections import defaultdict
import numpy as np

# Simple Q-learning parameters
Q_TABLE = defaultdict(lambda: np.zeros(3))  # Actions: [extend, reduce, switch]
LEARNING_RATE = 0.1
DISCOUNT_FACTOR = 0.9
EXPLORATION_RATE = 0.1

def run_simulation(config_file, optimized=False):
    try:
        sumo_cmd = ["sumo", "-c", config_file]
        traci.start(sumo_cmd)
        
        # Initialize performance metrics and history
        metrics = []
        speed_history = []
        waiting_history = []
        avg_speed_history = []
        state_history = []  # For RL state tracking

        net_file = config_file.replace('.sumocfg', '.net.xml')
        
        # Main simulation loop (1200 seconds as per sumocfg)
        while (traci.simulation.getMinExpectedNumber() > 0 and 
               traci.simulation.getTime() < 1200):
            traci.simulationStep()
            
            if optimized and traci.simulation.getTime() >= 0:  # Start immediately
                if traci.simulation.getTime() % 10 == 0:
                    current_metrics = calculate_performance_metrics()
                    current_avg_speed = current_metrics['avg_speed']
                    current_avg_waiting = current_metrics['avg_waiting_time']
                    current_co2 = current_metrics['total_co2']
                    
                    # Define state (simplified: avg_speed, avg_waiting bins)
                    state = (int(current_avg_speed // 1), int(current_avg_waiting // 1))
                    state_history.append(state)
                    
                    # Choose action using epsilon-greedy
                    if np.random.random() < EXPLORATION_RATE:
                        action = np.random.randint(3)  # 0: extend, 1: reduce, 2: switch
                    else:
                        action = np.argmax(Q_TABLE[state])
                    
                    # Apply action to traffic lights
                    optimize_traffic_lights(action, net_file)
                    optimize_roundabout_flow(net_file)
                    optimize_routes()
                    
                    # Calculate reward
                    new_metrics = calculate_performance_metrics()
                    new_avg_speed = new_metrics['avg_speed']
                    new_avg_waiting = new_metrics['avg_waiting_time']
                    new_co2 = new_metrics['total_co2']
                    reward = new_avg_speed - 2 * new_avg_waiting - new_co2 / 10000
                    
                    # Update Q-table
                    if len(state_history) > 1:
                        old_state = state_history[-2]
                        Q_TABLE[old_state][action] += LEARNING_RATE * (
                            reward + DISCOUNT_FACTOR * np.max(Q_TABLE[state]) - Q_TABLE[old_state][action]
                        )
                    
                    if new_avg_speed < current_avg_speed * 0.95:
                        print(f"Optimization reduced speed significantly: {current_avg_speed} to {new_avg_speed}")
                    avg_speed_history.append(new_avg_speed)

            if traci.simulation.getTime() % 60 == 0:
                print(f"Simulation progress: {traci.simulation.getTime()//60} minutes")           

            # Collect data for analysis
            if traci.simulation.getTime() % 10 == 0:
                current_metrics = calculate_performance_metrics()
                collect_traffic_data()
                metrics.append(current_metrics)
                speed_history.append(current_metrics['avg_speed'])
                waiting_history.append(current_metrics['avg_waiting_time'])

        return metrics
        
    except Exception as e:
        print(f"Error during simulation: {e}")
    finally:
        traci.close()

def collect_traffic_data():
    data = {
        'time': traci.simulation.getTime(),
        'edges': {},
        'vehicles': traci.vehicle.getIDCount(),
        'avg_speed': 0
    }
    
    total_speed = 0
    vehicle_count = 0
    
    for edge_id in traci.edge.getIDList():
        vehicles = traci.edge.getLastStepVehicleNumber(edge_id)
        speed = traci.edge.getLastStepMeanSpeed(edge_id)
        waiting = traci.edge.getWaitingTime(edge_id)
        
        data['edges'][edge_id] = {
            'vehicles': vehicles,
            'speed': speed,
            'waiting': waiting
        }
        
        total_speed += speed * vehicles
        vehicle_count += vehicles
    
    if vehicle_count > 0:
        data['avg_speed'] = total_speed / vehicle_count
    
    return data

def optimize_traffic_lights(action, net_file):
    tl_ids = traci.trafficlight.getIDList()
    for tl_id in tl_ids:
        current_phase = traci.trafficlight.getPhase(tl_id)
        phase_duration = traci.trafficlight.getPhaseDuration(tl_id)
        programs = traci.trafficlight.getAllProgramLogics(tl_id)
        current_program = next((p for p in programs if p.programID == traci.trafficlight.getProgram(tl_id)), None)
        if not current_program:
            continue

        num_phases = len(current_program.getPhases())
        controlled_lanes = traci.trafficlight.getControlledLanes(tl_id)
        lane_metrics = {lane: {'count': traci.lane.getLastStepVehicleNumber(lane),
                              'waiting': traci.lane.getWaitingTime(lane) / 60,
                              'speed': traci.lane.getLastStepMeanSpeed(lane)} 
                        for lane in controlled_lanes}

        green_lanes = set()
        for i, link in enumerate(traci.trafficlight.getControlledLinks(tl_id)):
            if link and current_program.getPhases()[current_phase].state[i] in ['G', 'g']:
                green_lanes.add(link[0][0])

        green_demand = sum(lane_metrics.get(lane, {'count': 0, 'waiting': 0})['count'] + 
                          lane_metrics.get(lane, {'count': 0, 'waiting': 0})['waiting'] * 2 
                          for lane in green_lanes) / max(1, len(green_lanes))
        red_demand = sum(lane_metrics.get(lane, {'count': 0, 'waiting': 0})['count'] + 
                        lane_metrics.get(lane, {'count': 0, 'waiting': 0})['waiting'] * 2 
                        for lane in lane_metrics if lane not in green_lanes) / max(1, len(lane_metrics) - len(green_lanes))

        if action == 0 and green_demand > 20 and phase_duration < 30:  # Extend
            extension = min(5, 30 - phase_duration)
            traci.trafficlight.setPhaseDuration(tl_id, phase_duration + extension)
            print(f"Extended phase at {tl_id} by {extension}s (green: {green_demand}, red: {red_demand})")
        elif action == 1 and green_demand < 10 and phase_duration > 5:  # Reduce
            reduction = max(-5, 5 - phase_duration)
            traci.trafficlight.setPhaseDuration(tl_id, phase_duration + reduction)
            print(f"Reduced phase at {tl_id} by {reduction}s (green: {green_demand}, red: {red_demand})")
        elif action == 2 and red_demand > 20:  # Switch
            traci.trafficlight.setPhase(tl_id, (current_phase + 1) % num_phases)
            print(f"Switched phase early at {tl_id} (green: {green_demand}, red: {red_demand})")

def optimize_roundabout_flow(net_file):
    try:
        roundabout_edges = get_roundabout_edges(net_file)
        for edge_id in roundabout_edges:
            if edge_id not in traci.edge.getIDList():
                continue
            num_lanes = traci.edge.getLaneNumber(edge_id)
            vehicle_threshold = num_lanes * 2
            speed_threshold = 0.6 * 15

            for lane_index in range(num_lanes):
                lane_id = f"{edge_id}_{lane_index}"
                if lane_id not in traci.lane.getIDList():
                    continue
                vehicle_count = traci.lane.getLastStepVehicleNumber(lane_id)
                mean_speed = traci.lane.getLastStepMeanSpeed(lane_id)

                if vehicle_count > vehicle_threshold and mean_speed < speed_threshold:
                    traci.lane.setParameter(lane_id, "stopOffset", "-0.5")
                    print(f"Adjusted stopOffset at {lane_id} to -0.5")
                else:
                    traci.lane.setParameter(lane_id, "stopOffset", "0")
    except Exception as e:
        if traci.simulation.getTime() % 10 == 0:
            print(f"Roundabout optimization warning: {e}")

def optimize_routes():
    vehicles = traci.vehicle.getIDList()
    reroute_counts = defaultdict(int)
    max_reroute_threshold = 0.05

    for veh_id in vehicles:
        current_edge = traci.vehicle.getRoadID(veh_id)
        route = traci.vehicle.getRoute(veh_id)
        try:
            current_index = route.index(current_edge)
            next_edges = route[current_index+1:]
        except ValueError:
            continue

        if next_edges and is_congested(next_edges[0]) and traci.vehicle.getWaitingTime(veh_id) > 120:
            destination = route[-1]
            alternative_route = find_least_congested_route(current_edge, destination)
            if (alternative_route and alternative_route != route and 
                reroute_counts[tuple(alternative_route)] / len(traci.edge.getIDList()) < max_reroute_threshold):
                traci.vehicle.setRoute(veh_id, alternative_route)
                reroute_counts[tuple(alternative_route)] += 1
                print(f"Rerouted {veh_id} to {alternative_route}")

def is_congested(edge_id):
    vehicle_count = traci.edge.getLastStepVehicleNumber(edge_id)
    mean_speed = traci.edge.getLastStepMeanSpeed(edge_id)
    waiting_time = traci.edge.getWaitingTime(edge_id)
    max_speed = 15
    congestion_score = (vehicle_count * 0.5 + (waiting_time / 60 * 0.3) + ((max_speed - mean_speed) / max_speed * 0.2))
    return congestion_score > 8

def find_least_congested_route(current_edge, destination):
    try:
        def get_edge_weight(edge):
            vehicle_count = traci.edge.getLastStepVehicleNumber(edge)
            mean_speed = traci.edge.getLastStepMeanSpeed(edge)
            waiting_time = traci.edge.getWaitingTime(edge)
            max_speed = 15
            return (vehicle_count * 0.5 + (waiting_time / 60 * 0.3) + ((max_speed - mean_speed) / max_speed * 0.2))
        
        routes = traci.simulation.findRoute(current_edge, destination, routingMode=1)
        if routes and len(routes.edges) > 0:
            routes.edges = [e for e in routes.edges if e in traci.edge.getIDList()]
            return routes.edges
    except traci.TraCIException:
        pass
    return None

def get_roundabout_edges(net_file):
    net = sumolib.net.readNet(net_file)
    roundabout_edges = set()
    for r in net.getRoundabouts():
        for edge_id in r.getEdges():
            roundabout_edges.add(edge_id)
    return list(roundabout_edges)

def calculate_performance_metrics():
    metrics = {
        'timestamp': traci.simulation.getTime(),
        'vehicle_count': traci.vehicle.getIDCount(),
        'total_travel_time': 0,
        'total_waiting_time': 0,
        'total_co2': 0,
        'avg_speed': 0
    }
    
    if metrics['vehicle_count'] == 0:
        return metrics
    
    total_speed = 0
    current_time = traci.simulation.getTime()
    
    for veh_id in traci.vehicle.getIDList():
        depart_time = traci.vehicle.getDeparture(veh_id)
        if depart_time >= 0:
            metrics['total_travel_time'] += current_time - depart_time
        metrics['total_waiting_time'] += traci.vehicle.getWaitingTime(veh_id)
        metrics['total_co2'] += traci.vehicle.getCO2Emission(veh_id)
        total_speed += traci.vehicle.getSpeed(veh_id)
    
    metrics.update({
        'avg_travel_time': metrics['total_travel_time'] / metrics['vehicle_count'],
        'avg_waiting_time': metrics['total_waiting_time'] / metrics['vehicle_count'],
        'avg_speed': total_speed / metrics['vehicle_count']
    })
    
    return metrics

if __name__ == "__main__":
    performance_metrics = run_simulation("gjilaniData/gjilani.sumocfg")
    
    if performance_metrics:
        print("Simulation completed successfully. Performance metrics:")
        for i, metric in enumerate(performance_metrics):
            print(f"Time {i*10}s: {metric}")
    else:
        print("Simulation failed or was interrupted")
