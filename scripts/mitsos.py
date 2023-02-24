import matplotlib.pyplot as plt
import sys

if len(sys.argv) != 2:
    print("Usage: python mitsos.py <file>")
    exit()

algorithm_names = [
    "Local Search Incremental",
    "Local Search Convex Hull",
    "Local Step Incremental",
    "Local Step Convex Hull",
    "Global Step Incremental",
    "Global Step Convex Hull",
    "Subdivision",
]

algorithms_count = 7

with open(sys.argv[1], "r") as f:
    lines = f.readlines()[2:]

    # Remove the spaces from all the lines
    lines = [line.replace(" ", "") for line in lines]

    # Replace all the || and | with a space
    lines = [line.replace("||", " ") for line in lines]
    lines = [line.replace("|", " ") for line in lines]

    sizes = []
    min_scores = [[] for _ in range(algorithms_count)]
    max_scores = [[] for _ in range(algorithms_count)]
    min_bounds = [[] for _ in range(algorithms_count)]
    max_bounds = [[] for _ in range(algorithms_count)]
    for line in lines:
        # Tokenize the line
        tokens = line.split()

        # Get the size
        sizes.append(float(tokens[0]))

        for i in range(algorithms_count):
            min_scores[i].append(float(tokens[i * 4 + 1]))
            max_scores[i].append(float(tokens[i * 4 + 2]))
            min_bounds[i].append(float(tokens[i * 4 + 3]))
            max_bounds[i].append(float(tokens[i * 4 + 4]))

    for i in range(algorithms_count):
        plt.plot(sizes, min_scores[i], label=algorithm_names[i])
    plt.title("min_score")
    plt.legend()
    plt.show()

    for i in range(algorithms_count):
        plt.plot(sizes, max_scores[i], label=algorithm_names[i])
    plt.title("max_score")
    plt.legend()
    plt.show()

    for i in range(algorithms_count):
        plt.plot(sizes, min_bounds[i], label=algorithm_names[i])
    plt.title("min_bound")
    plt.legend()
    plt.show()

    for i in range(algorithms_count):
        plt.plot(sizes, max_bounds[i], label=algorithm_names[i])
    plt.title("max_bound")
    plt.legend()
    plt.show()
