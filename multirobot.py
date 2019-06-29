from math import sqrt

import gym

import gym_warehouse
from heapq import *


def heuristic(a, b):
    return abs(b[0] - a[0]) + abs(b[1] - a[1])


def astarmodifed(array, start, goal, allpath, height, width):
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]

    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic((start[0], start[1]), goal)}
    oheap = []
    time = 0
    heappush(oheap, (fscore[start], start))
    waitdict = {}
    wait = 1

    while oheap:
        current = heappop(oheap)[1]
        if (current[0], current[1]) not in waitdict:
            waitdict[(current[0], current[1])] = wait
        if current[0] == goal[0] and current[1] == goal[1]:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start)
            return list(reversed(data))
        notfound = True
        count = 0
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j, current[2] + 1
            tentative_g_score = gscore[current] + heuristic((current[0], current[1]), (neighbor[0], neighbor[1]))
            if 0 <= neighbor[0] < height:
                if 0 <= neighbor[1] < width:
                    if array[neighbor[0]][neighbor[1]] == '*':
                        count = count + 1
                        continue
                    else:
                        if array[neighbor[0]][neighbor[1]] != '.':
                            if str(array[neighbor[0]][neighbor[1]]).islower():
                                count = count + 1

                                continue

                else:
                    # array bound y walls
                    count = count + 1

                    continue
            else:
                count = count + 1

                # array bound x walls

                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if (tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [
                (i[1][0], i[1][1], i[1][2]) for i in
                oheap]):
                if len(allpath) > 0:
                    if neighbor not in [i for i in allpath]:
                        if (current[0], current[1], (current[2] + 1)) in [i for i in allpath]:
                            if (neighbor[0], neighbor[1], (current[2])) not in [(i[0], i[1], i[2]) for i in allpath]:
                                notfound = False
                                came_from[(neighbor[0], neighbor[1], current[2] + 1)] = current
                                gscore[neighbor] = tentative_g_score
                                fscore[neighbor] = tentative_g_score + heuristic((neighbor[0], neighbor[1]), goal)
                                heappush(oheap, (fscore[neighbor], neighbor))


                        else:
                            notfound = False

                            came_from[(neighbor[0], neighbor[1], current[2] + 1)] = current
                            gscore[neighbor] = tentative_g_score
                            fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                            heappush(oheap, (fscore[neighbor], neighbor))

                if len(allpath) == 0:
                    notfound = False
                    if waitdict[current[0], current[1]] < 101:
                        waitdict[(current[0], current[1])] += wait
                        came_from[(neighbor[0], neighbor[1], current[2] + 1)] = current
                        gscore[neighbor] = tentative_g_score
                        fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                        heappush(oheap, (fscore[neighbor], neighbor))

        if notfound and count != 4:
            came_from[(current[0], current[1], current[2] + 1)] = current
            gscore[(current[0], current[1], current[2] + 1)] = (gscore[current])
            fscore[(current[0], current[1], current[2] + 1)] = (
                    gscore[current] + heuristic((current[0], current[1]), goal) + 200)
            heappush(oheap,
                     (fscore[(current[0], current[1], current[2] + 1)], (current[0], current[1], current[2] + 1)))

    return False


def run_agent():
    # create the Gym environment
    env = gym.make('multirobot-warehouse-v0')
    s = env.look()
    x1 = 0
    pointstart = {}
    pointExit = {}

    for f1 in s:  # adding treasure to list

        y1 = 0
        for f2 in f1:
            if f2 != "*" and f2 != ".":
                if str(f2).isupper():
                    pointExit[f2] = (x1, y1)

                else:
                    pointstart[f2] = (x1, y1)

            y1 = y1 + 1
        x1 = x1 + 1

    allpaths = []
    agentpaths = {}
    for ct in sorted(pointstart):
        path = astarmodifed(s, (pointstart[ct][0], pointstart[ct][1], 0), pointExit[str(ct).upper()], allpaths, len(s),
                            len(s[0]))
        if path == False:
            print(str("There is no valid path "))
            raise SystemExit(0)
        print(str("optimal path of ") + str(ct) + ":" + str(path))

        agentpaths[ct] = path
        for sd in path:
            allpaths.append(sd)

    done = False
    count = 0

    while True:
        env.render()  # you can used this for printing the environment

        # sense
        temp = []
        tempnext = []
        tempaction = []
        for fg in sorted(pointstart):
            if len(agentpaths[fg]) - 1 > count:
                temp.append(agentpaths[fg][count])
                tempnext.append((agentpaths[fg][count + 1]))
                x = agentpaths[fg][count][0]
                y = agentpaths[fg][count][1]
                targetx = agentpaths[fg][count + 1][0]
                targety = agentpaths[fg][count + 1][1]
                did = True
                if x - targetx < 0 and did:
                    did = False
                    tempaction.append(env.ACTION_DOWN)

                if x - targetx > 0 and did:
                    did = False
                    tempaction.append(env.ACTION_UP)

                if y - targety > 0 and did:
                    did = False
                    tempaction.append(env.ACTION_LEFT)

                if y - targety < 0 and did:
                    did = False
                    tempaction.append(env.ACTION_RIGHT)
                if x == targetx and y == targety and did:
                    did = False

                    tempaction.append(env.ACTION_WAIT)


            else:
                tempaction.append(env.ACTION_WAIT)

        print(tempaction)
        ob, rew, done = env.step(tempaction)

        if done:
            break
        count = count + 1;

    env.close()


if __name__ == "__main__":
    run_agent()
