from real_time_controller import RealTimeController
import multiprocessing as mp

if __name__ == "__main__":
    # Enable multiprocessing support
    mp.freeze_support()
    
    # Create and run the controller
    controller = RealTimeController(
        config_file="gjilaniData/gjilani.sumocfg",
        net_file="gjilaniData/gjilani.net.xml",
        route_file="gjilaniData/route-alt.rou.xml",
        output_dir="real_time_results"
    )
    
    # Run the simulation
    controller.run(simulation_steps=600)  # 1 hour simulation
    controller.analyze_results()
