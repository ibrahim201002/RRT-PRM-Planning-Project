import matplotlib.pyplot as plt

from path_planner_interface import Configuration
from RRT import RRTPlanner
from PRM import PRMPlanner
from test_planners import generate_random_obstacles, test_planner


def run_average_tests(num_runs=100,num_obstacles=100,best_step_size=1.0,best_k_neighbors=10):
    bounds = ((0.0, 10.0), (0.0, 10.0))
    start = Configuration(1.0, 1.0)
    goal = Configuration(9.0, 9.0)

    rrt_max_iterations_list = [1000, 5000, 10000]
    prm_num_samples_list = [1000, 5000, 10000]

    rrt_results = []
    prm_results = []
    for max_it in rrt_max_iterations_list:
        total_time = 0.0
        total_length = 0.0
        success_count = 0
        print("RRT: max_iterations =", max_it)

        for run_id in range(num_runs):
            obstacles = generate_random_obstacles(bounds, num_obstacles, start, goal)
            planner = RRTPlanner(start=start,goal=goal,bounds=bounds,step_size=best_step_size,max_iterations=max_it,)
            planner.set_obstacles(obstacles)
            res = test_planner(planner, start, goal, obstacles, visualize=False)

            if res["success"]:
                success_count += 1
                total_time += res["planning_time"]
                total_length += res["path_length"]

        if success_count > 0:
            avg_time = total_time / float(success_count)
            avg_length = total_length / float(success_count)
        else:
            avg_time = 0.0
            avg_length = 0.0

        success_rate = 100.0 * success_count / float(num_runs)
        rrt_results.append({"max_iterations": max_it,"avg_time": avg_time,"avg_length": avg_length,"success_rate": success_rate,})
        print("  success_count =", success_count, "/", num_runs,
              "(success rate = {:.1f}% )".format(success_rate))
        print("  avg_time   =", avg_time)
        print("  avg_length =", avg_length)
        print()

    for num_samples in prm_num_samples_list:
        total_time = 0.0
        total_length = 0.0
        success_count = 0

        print("PRM: num_samples =", num_samples)

        for run_id in range(num_runs):
            obstacles = generate_random_obstacles(bounds, num_obstacles, start, goal)

            planner = PRMPlanner(start=start,goal=goal,bounds=bounds,num_samples=num_samples,k_neighbors=best_k_neighbors,)
            planner.set_obstacles(obstacles)
            res = test_planner(planner, "PRM_avg_ns{}_run{}".format(num_samples, run_id),visualize_result=False)

            if res["success"]:
                success_count += 1
                total_time += res["planning_time"]
                total_length += res["path_length"]

        if success_count > 0:
            avg_time = total_time / float(success_count)
            avg_length = total_length / float(success_count)
        else:
            avg_time = 0.0
            avg_length = 0.0

        success_rate = 100.0 * success_count / float(num_runs)

        prm_results.append({"num_samples": num_samples,"avg_time": avg_time, "avg_length": avg_length, "success_rate": success_rate, })
        print("  success_count =", success_count, "/", num_runs,
              "(success rate = {:.1f}% )".format(success_rate))
        print("  avg_time   =", avg_time)
        print("  avg_length =", avg_length)
        print()

    return rrt_results, prm_results


def make_bar_plots(rrt_results, prm_results):

    labels = ["1000", "5000", "10000"]

    rrt_times = [res["avg_time"] for res in rrt_results]
    prm_times = [res["avg_time"] for res in prm_results]

    rrt_lengths = [res["avg_length"] for res in rrt_results]
    prm_lengths = [res["avg_length"] for res in prm_results]

    x = range(len(labels))
    width = 0.35

    plt.figure(figsize=(8, 5))
    x_rrt = [xi - width / 2.0 for xi in x]
    x_prm = [xi + width / 2.0 for xi in x]

    plt.bar(x_rrt, rrt_times, width, label="RRT")
    plt.bar(x_prm, prm_times, width, label="PRM")

    plt.xticks(x, labels)
    plt.xlabel("max_iterations / num_samples")
    plt.ylabel("Average planning time (s)")
    plt.title("Average Planning Time (100 random maps, 100 obstacles)")
    plt.legend()
    plt.grid(axis="y", alpha=0.3)
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(8, 5))
    plt.bar(x_rrt, rrt_lengths, width, label="RRT")
    plt.bar(x_prm, prm_lengths, width, label="PRM")

    plt.xticks(x, labels)
    plt.xlabel("max_iterations / num_samples")
    plt.ylabel("Average path length")
    plt.title("Average Path Length (100 random maps, 100 obstacles)")
    plt.legend()
    plt.grid(axis="y", alpha=0.3)
    plt.tight_layout()
    plt.show()
    
def main():
    best_step_size = 1.0     
    best_k_neighbors = 10   

    rrt_results, prm_results = run_average_tests( num_runs=100,num_obstacles=100,best_step_size=best_step_size,best_k_neighbors=best_k_neighbors,)
    make_bar_plots(rrt_results, prm_results)

if __name__ == "__main__":
    main()
