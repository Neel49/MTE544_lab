import matplotlib.pyplot as plt
import numpy as np
from utilities import FileReader




def plot_errors(filename, overlay=None):
    
    headers, values=FileReader(filename).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    
    
    fig, axes = plt.subplots(1,2, figsize=(14,6))


    axes[0].plot([lin[0] for lin in values], [lin[1] for lin in values])
    if headers[0].strip() == "x" and overlay is not None:
        # Use the same reference points as planner.py.
        if overlay == "parabola":
            x = np.linspace(0.0, 1.5, 61)
            y = x ** 2
        elif overlay == "sigmoid":
            x = np.linspace(0.0, 2.5, 101)
            y = 2.0 / (1 + np.exp(-2.0 * x)) - 1.0
        axes[0].lines[0].set_label("Recorded path")
        axes[0].plot(x, y, color="orange", linestyle="--", label=f"Planned {overlay}")
        axes[0].legend()
    axes[0].set_title("state space")
    axes[0].grid()

    
    axes[1].set_title("each individual state")
    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")

    axes[1].legend()
    axes[1].grid()

    plt.show()
    
    





import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    parser.add_argument('--overlay', choices=['parabola', 'sigmoid'],
                        help='Optional reference curve for the robot x-y plot')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        plot_errors(filename, args.overlay)



