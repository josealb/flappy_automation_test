import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

def animate(i):
    graph_data = open('/home/flyatest/map.txt','r').read()
    lines = graph_data.split('\n')
    xs = []
    ys = []
    for line in lines:
        if len(line) > 1:
            try:
                x, y = line.split(',')
                xs.append(float(x))
                ys.append(float(y))
            except:
                print('WARNING: problem parsing some values')
    ax1.clear()
    try:
        ax1.scatter(xs, ys)
    except:
        print('WARNING: problem parsing data. Probably file is being written')

    graph_data = open('/home/flyatest/freespace_map.txt','r').read()
    lines = graph_data.split('\n')
    xs = []
    ys = []
    for line in lines:
        if len(line) > 1:
            try:
                x, y = line.split(',')
                xs.append(float(x))
                ys.append(float(y))
            except:
                print('WARNING: problem parsing some values')
    try:
        ax1.scatter(xs, ys, color='green')
    except:
        print('WARNING: problem parsing data. Probably file is being written')


    graph_data = open('/home/flyatest/ego_position.txt','r').read()
    lines = graph_data.split('\n')
    xs = []
    ys = []
    for line in lines:
        if len(line) > 1:
            try:
                x, y = line.split(',')
                xs.append(float(x))
                ys.append(float(y))
            except:
                print('WARNING: problem parsing some values')
    try:
        ax1.scatter(xs, ys, color='red')
    except:
        print('WARNING: problem parsing data. Probably file is being written')

    try:
        graph_data = open('/home/flyatest/goal_position.txt','r').read()
        lines = graph_data.split('\n')
        xs = []
        ys = []
        for line in lines:
            if len(line) > 1:
                try:
                    x, y = line.split(',')
                    xs.append(float(x))
                    ys.append(float(y))
                except:
                    print('WARNING: problem parsing some values')
        try:
            ax1.scatter(xs, ys, color='orange')
        except:
            print('WARNING: problem parsing data. Probably file is being written')
    except:
        print('Unable to open file')


ani = animation.FuncAnimation(fig, animate, interval=1000)
plt.show()