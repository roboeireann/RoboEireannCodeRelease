import matplotlib.pyplot as plt
from stable_baselines3.common import results_plotter
import sys
sys.path.append(sys.path[0] + "/..")

path = "/home/yoonchae/Abstract/dummy_defenders"

"""
if "--ext" in sys.argv:
    Monitor.EXT  = "monitor.csv" #sys.argv[sys.argv.index("--ext") + 1]
else:
    print("Please specify a log file extension")
"""
# Monitor.EXT = '/home/yoonchae/Abstract/Models/dummy_defenders/monitor.csv'


results_plotter.plot_results(
    ["./"], 10e8, results_plotter.X_TIMESTEPS, "grounded training convergence test"
)

plt.show()
