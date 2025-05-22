from congestion_optimizer import CongestionOptimizer

optimizer = CongestionOptimizer(
    config_file="gjilaniData/gjilani.sumocfg",
    net_file="gjilaniData/gjilani.net.xml",
    route_file="gjilaniData/gjilani.rou.xml",
    output_dir="congestion_results_400s"
)

print("Starting congestion optimization")
best_solution = optimizer.optimize()

print("\nBest solution found:")
print(best_solution)
