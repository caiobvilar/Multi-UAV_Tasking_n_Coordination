import json
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import os

from argparse import ArgumentParser
np.random.seed(20201988)
dir_path = os.path.dirname(os.path.realpath(__file__))
def read_json(filename):
    with open(filename, 'r') as file:
        data = json.load(file)
    return data

def plot_data(data = None):
    poly = np.array(list(zip(data["roi"]["x"], data["roi"]["y"])))
    ax = plt.gca()
    plt.gcf().canvas.mpl_connect('key_release_event',
                                 lambda event: [exit(0) if event.key == 'escape' else None])
    ax.plot(poly[:,0].tolist()+[poly[0,0]], poly[:,1].tolist()+[poly[0,1]])

    for path in data["paths"].values():
        ax.plot(path["x"], path["y"])
    patches = []
    for k in data["areas"]:
        new_area = data["areas"][k]
        poly = np.array(list(zip(new_area["x"], new_area["y"])))
        polygon = Polygon(poly)
        # ax.add_patch(polygon)
        patches.append(polygon)
    colors = 100*np.random.rand(len(patches))
    p = PatchCollection(patches, alpha=0.35)
    p.set_array(np.array(colors))
    ax.add_collection(p)


    plt.show()

if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--file', required=True, type=str, help = "json file")
    args = parser.parse_args()

    data = read_json(os.path.join(dir_path, args.file))
    plot_data(data)
