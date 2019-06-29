import gym_warehouse

import gym
from heapq import *


def heuristic(a, b):
    return abs(b[0] - a[0]) + abs(b[1] - a[1])


def astar(array, start, goal, height, width):
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]

    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []
    heappush(oheap, (fscore[start], start))

    while oheap:

        current = heappop(oheap)[1]

        if current[0] == goal[0] and current[1] == goal[1]:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append((start[0], start[1]))
            return data

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < height:
                if 0 <= neighbor[1] < width:
                    if array[neighbor[0]][neighbor[1]] == '*':
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))

    return False


def run_agent():
    # create the Gym environment
    env = gym.make('singlerobot-warehouse-v0')
    s = env.look()

    x1 = 0
    for f1 in s:

        y1 = 0
        for f2 in f1:
            if f2 == "A":
                exitpoint = ['A', (x1, y1)]

            if f2 == "a":
                startpoint = ['a', (x1, y1)]

            y1 = y1 + 1
        x1 = x1 + 1

    path = astar(s, startpoint[1], exitpoint[1],len(s), len(s[0]))
    print(str("The optimal path:"+str(path)))

    done = False
    count = len(path) - 1

    while True:
        env.render()  # you can used this for printing the environment

        # sense

        x = path[count][0]
        y = path[count][1]
        targetx = path[count - 1][0]
        targety = path[count - 1][1]

        if x - targetx > 0:
            ob, rew, done = env.step(env.ACTION_UP)

        if x - targetx < 0:
            ob, rew, done = env.step(env.ACTION_DOWN)

        if y - targety < 0:
            ob, rew, done = env.step(env.ACTION_RIGHT)

        if y - targety > 0:
            ob, rew, done = env.step(env.ACTION_LEFT)
        if done:
            exit(0)
            break

        if done:
            break
        count = count - 1;

    env.close()


if __name__ == "__main__":
    run_agent()
