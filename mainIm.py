import traci
import sumolib
from collections import defaultdict
import numpy as np
from sklearn.linear_model import LinearRegression

def run_separate_simulations():
    # Original simulation
    print("Running original simulation...")
    
    # Original simulation command (no optimization)
    sumo_cmd_original = [
        "sumo", 
        "-c", "gjilaniData/gjilani.sumocfg",
        "--emission-output", "gjilaniData/output_emissions.xml",
        "--fcd-output", "gjilaniData/output_fcd.xml",
        "--queue-output", "gjilaniData/output_queues.xml",
        "--summary-output", "gjilaniData/output_summary.xml",
        "--duration", "1200"  # Run for 20 minutes
    ]
    
    traci.start(sumo_cmd_original)
    
    # Run original simulation (without optimization)
    while (traci.simulation.getMinExpectedNumber() > 0 and 
           traci.simulation.getTime() < 1200):  # 20 minutes
        traci.simulationStep()
        
        if traci.simulation.getTime() % 60 == 0:
            print(f"Original simulation progress: {traci.simulation.getTime()//60} minutes")
    
    traci.close()
    
    # Optimized simulation
    print("\nRunning optimized simulation...")
    
    # Optimized simulation command (with optimization)
    sumo_cmd_optimized = [
        "sumo", 
        "-c", "gjilaniData/gjilani.sumocfg",
        "--emission-output", "gjilaniData/optimized_output_emissions.xml",
        "--fcd-output", "gjilaniData/optimized_output_fcd.xml",
        "--queue-output", "gjilaniData/optimized_output_queues.xml",
        "--summary-output", "gjilaniData/optimized_output_summary.xml",
        "--duration", "1200"  # Run for 20 minutes
    ]
    
    traci.start(sumo_cmd_optimized)
    
    # Initialize metrics tracking
    metrics = []
    speed_history = []
    waiting_history = []
    
    # Run optimized simulation (with optimization)
    while (traci.simulation.getMinExpectedNumber() > 0 and 
           traci.simulation.getTime() < 1200):  # 20 minutes
        traci.simulationStep()
        
        # Apply optimizations after initial period
        if traci.simulation.getTime() >= 30:  # Start optimization after 30 seconds
            if traci.simulation.getTime() % 10 == 0:  # Check every 10 seconds
                try:
                    current_metrics = calculate_performance_metrics()
                    current_avg_speed = current_metrics['avg_speed']
                    current_avg_waiting = current_metrics['avg_waiting_time']
                    
                    # Apply more aggressive optimizations
                    if len(speed_history) > 5:
                        # Make optimization more aggressive
                        threshold_factor = 0.8  # Lower threshold = more optimization
                        
                        if current_avg_waiting > 0.5 or current_avg_speed < 10.0:
                            threshold_factor = 0.6  # Even more aggressive if poor conditions
                            
                        # Apply optimizations
                        optimize_traffic_lights(threshold_factor)
                        optimize_roundabout_flow("gjilaniData/gjilani.net.xml", threshold_factor)
                        optimize_routes(threshold_factor)
                        
                    # Track history
                    speed_history.append(current_avg_speed)
                    waiting_history.append(current_avg_waiting)
                    metrics.append(current_metrics)
                except Exception as e:
                    print(f"Error during optimization: {e}")
        
        if traci.simulation.getTime() % 60 == 0:
            print(f"Optimized simulation progress: {traci.simulation.getTime()//60} minutes")
    
    traci.close()
    print("\nBoth simulations completed successfully!")

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

def optimize_traffic_lights(threshold_factor):
    tl_ids = traci.trafficlight.getIDList()
    optimizations_made = 0
    
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

        # More aggressive thresholds (lower thresholds, higher extensions)
        if green_demand > 8 * threshold_factor and green_demand > red_demand * 1.1:
            extension = min(10, max(3, int(green_demand / 3)))
            traci.trafficlight.setPhaseDuration(tl_id, phase_duration + extension)
            optimizations_made += 1
            print(f"Extended phase at {tl_id} by {extension}s (green: {green_demand}, red: {red_demand})")
        elif red_demand > 10 * threshold_factor and red_demand > green_demand * 1.2:
            traci.trafficlight.setPhase(tl_id, (current_phase + 1) % num_phases)
            optimizations_made += 1
            print(f"Switched phase early at {tl_id} (green: {green_demand}, red: {red_demand})")
    
    if optimizations_made > 0:
        print(f"Made {optimizations_made} traffic light optimizations")
    return optimizations_made

def optimize_roundabout_flow(net_file, threshold_factor):
    try:
        roundabout_edges = get_roundabout_edges(net_file)
        for edge_id in roundabout_edges:
            if edge_id not in traci.edge.getIDList():
                continue
            num_lanes = traci.edge.getLaneNumber(edge_id)
            vehicle_threshold = num_lanes * 2 * threshold_factor
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

def optimize_routes(threshold_factor):
    vehicles = traci.vehicle.getIDList()
    reroute_counts = defaultdict(int)
    max_reroute_threshold = 0.2 / threshold_factor  # Higher value = more rerouting
    reroutes_made = 0

    for veh_id in vehicles:
        current_edge = traci.vehicle.getRoadID(veh_id)
        route = traci.vehicle.getRoute(veh_id)
        try:
            current_index = route.index(current_edge)
            next_edges = route[current_index+1:]
        except ValueError:
            continue

        # Lower waiting time threshold for rerouting
        if next_edges and is_congested(next_edges[0]) and traci.vehicle.getWaitingTime(veh_id) > 40 * threshold_factor:
            destination = route[-1]
            alternative_route = find_least_congested_route(current_edge, destination)
            if (alternative_route and alternative_route != route and 
                reroute_counts[tuple(alternative_route)] / len(traci.edge.getIDList()) < max_reroute_threshold):
                traci.vehicle.setRoute(veh_id, alternative_route)
                reroute_counts[tuple(alternative_route)] += 1
                reroutes_made += 1
                print(f"Rerouted {veh_id} to {alternative_route}")
    
    if reroutes_made > 0:
        print(f"Rerouted {reroutes_made} vehicles")
    return reroutes_made

def is_congested(edge_id):
    vehicle_count = traci.edge.getLastStepVehicleNumber(edge_id)
    mean_speed = traci.edge.getLastStepMeanSpeed(edge_id)
    waiting_time = traci.edge.getWaitingTime(edge_id)
    max_speed = 15
    
    # More sensitive congestion detection
    congestion_score = (vehicle_count * 0.6 + (waiting_time / 60 * 0.4) + ((max_speed - mean_speed) / max_speed * 0.3))
    return congestion_score > 5  # Lower threshold to detect congestion earlier

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
    run_separate_simulations()
