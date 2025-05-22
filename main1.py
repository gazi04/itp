import traci
import sumolib
from collections import defaultdict

def run_simulation(config_file, optimized=False):
    try:
        sumo_cmd = ["sumo", "-c", config_file]
        traci.start(sumo_cmd)
        
        # Initialize performance metrics
        metrics = []

        net_file = config_file.replace('.sumocfg', '.net.xml')
        
        # Main simulation loop (900 seconds as per sumocfg)
        while (traci.simulation.getMinExpectedNumber() > 0 and 
               traci.simulation.getTime() < 200):
            traci.simulationStep()
            
            if optimized:
                if traci.simulation.getTime() % 30 == 0:  # Run optimizations every 30 seconds
                    optimize_traffic_lights()
                    optimize_roundabout_flow(net_file)
                    optimize_routes()
                    prioritize_emergency_vehicles()
                    prioritize_public_transport()

            if traci.simulation.getTime() % 60 == 0:
                print(f"Simulation progress: {traci.simulation.getTime()//60} minutes")           

            # Collect data for analysis
            if traci.simulation.getTime() % 10 == 0:
                collect_traffic_data()
                metrics.append(calculate_performance_metrics())

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

"""
OPTIMIZATION STRATEGIES
"""
def optimize_traffic_lights():
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

        # Calculate demand for green and red directions
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

        if green_demand > 10 and green_demand > red_demand * 1.2:  # Lower threshold for extension
            extension = min(10, max(2, int(green_demand / 3)))
            traci.trafficlight.setPhaseDuration(tl_id, phase_duration + extension)
            print(f"Extended phase at {tl_id} by {extension}s (green: {green_demand}, red: {red_demand})")
        elif red_demand > 10 and red_demand > green_demand * 1.5:  # Lower threshold for switch
            traci.trafficlight.setPhase(tl_id, (current_phase + 1) % num_phases)
            print(f"Switched phase early at {tl_id} (green: {green_demand}, red: {red_demand})")

def optimize_roundabout_flow(net_file):
    try:
        roundabout_edges = get_roundabout_edges(net_file)
        for edge_id in roundabout_edges:
            if edge_id not in traci.edge.getIDList():
                continue
            num_lanes = traci.edge.getLaneNumber(edge_id)
            vehicle_threshold = num_lanes  # Lower threshold
            speed_threshold = 0.5 * 15  # Adjusted threshold

            for lane_index in range(num_lanes):
                lane_id = f"{edge_id}_{lane_index}"
                if lane_id not in traci.lane.getIDList():
                    continue
                vehicle_count = traci.lane.getLastStepVehicleNumber(lane_id)
                mean_speed = traci.lane.getLastStepMeanSpeed(lane_id)
                links = traci.lane.getLinks(lane_id)
                exit_edge = None
                if links:
                    successor_lane = links[0][0]
                    exit_edge = traci.lane.getEdgeID(successor_lane) if successor_lane else None

                if vehicle_count > vehicle_threshold and mean_speed < speed_threshold:
                    traci.lane.setParameter(lane_id, "stopOffset", "-5")
                    print(f"Adjusted stopOffset at {lane_id} to -5")
                else:
                    traci.lane.setParameter(lane_id, "stopOffset", "0")
    except Exception as e:
        if traci.simulation.getTime() % 10 == 0:
            print(f"Roundabout optimization warning: {e}")

def optimize_routes():
    vehicles = traci.vehicle.getIDList()
    reroute_counts = defaultdict(int)
    max_reroute_threshold = 0.15

    for veh_id in vehicles:
        current_edge = traci.vehicle.getRoadID(veh_id)
        route = traci.vehicle.getRoute(veh_id)
        try:
            current_index = route.index(current_edge)
            next_edges = route[current_index+1:]
        except ValueError:
            continue

        if next_edges and is_congested(next_edges[0]) and traci.vehicle.getWaitingTime(veh_id) > 60:
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
    return congestion_score > 5

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

def prioritize_emergency_vehicles():
    for veh_id in traci.vehicle.getIDList():
        if "emergency" in veh_id.lower():
            route = traci.vehicle.getRoute(veh_id)
            current_edge = traci.vehicle.getRoadID(veh_id)
            for tl_id in get_upcoming_traffic_lights(route, current_edge):
                traci.trafficlight.setPhase(tl_id, get_green_phase_for_direction(tl_id, veh_id))
                print(f"Prioritized emergency vehicle {veh_id} at {tl_id}")

def prioritize_public_transport():
    bus_stops = traci.busstop.getIDList()
    for stop in bus_stops:
        lane = traci.busstop.getLaneID(stop)
        edge = traci.lane.getEdgeID(lane)
        if traci.lane.getLastStepVehicleNumber(lane) > 0:
            adjust_traffic_lights_near(edge, priority=True)
            print(f"Prioritized public transport near {edge}")

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

def get_upcoming_traffic_lights(route, current_edge):
    tl_ids = []
    try:
        start_idx = route.index(current_edge)
        for edge in route[start_idx+1:]:
            if traci.edge.getTrafficLights(edge):
                tl_ids.extend(traci.trafficlight.getIDList())
    except ValueError:
        pass
    return list(set(tl_ids))

def get_green_phase_for_direction(tl_id, veh_id):
    current_phase = traci.trafficlight.getPhase(tl_id)
    return current_phase  # Placeholder

def adjust_traffic_lights_near(edge, priority):
    tl_ids = traci.trafficlight.getIDList()
    for tl_id in tl_ids:
        if any(edge in link[0][0] for link in traci.trafficlight.getControlledLinks(tl_id) if link):
            traci.trafficlight.setPhaseDuration(tl_id, 15 if priority else 5)

if __name__ == "__main__":
    performance_metrics = run_simulation("gjilaniData/gjilani.sumocfg")
    
    if performance_metrics:
        print("Simulation completed successfully. Performance metrics:")
        for i, metric in enumerate(performance_metrics):
            print(f"Time {i*10}s: {metric}")
    else:
        print("Simulation failed or was interrupted")
