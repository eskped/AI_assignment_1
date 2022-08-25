import random
from constants import *
from environment import *
from state import State
import math
import heapq
"""
solution.py

This file is a template you should use to implement your solution.

You should implement

COMP3702 2022 Assignment 1 Support Code

Last updated by njc 01/08/22
"""


class Solver:

    def __init__(self, environment, loop_counter):
        self.environment = environment
        self.loop_counter = loop_counter

        self.parents = {}
        self.parents[self.environment.get_init_state()] = [None, None, 0]
        self.frontier = []
        heapq.heapify(self.frontier)
        self.init_center_targets()
        # self.init_target_cells()

    def init_center_targets(self):
        env = self.environment
        target_list = env.target_list.copy()
        # print("Target list", target_list)
        possible_center_targets = []
        for cell in target_list:
            number = 0
            for ori in ROBOT_ORIENTATIONS:
                adj = get_adjacent_cell_coords(cell, ori)
                if adj in target_list:
                    number += 1
            if number >= 2:
                possible_center_targets.append(cell)
        # print("poss", possible_center_targets)

        center_targets = []
        for cell in possible_center_targets:
            other_targets = []
            non_targets = []
            for ori in ROBOT_ORIENTATIONS:
                adj = get_adjacent_cell_coords(cell, ori)
                if adj in target_list:
                    other_targets.append((adj, ori))
                else:
                    non_targets.append((adj, ori))
            # print("Cell", cell)
            # print(other_targets)
            # print(non_targets)
            if len(other_targets) == 4 and len(non_targets) == 2:
                if self.is_opposite(cell, non_targets[0][0], non_targets[0][1], non_targets[1][0], non_targets[1][1]):
                    center_targets.append((WIDGET5, cell))
                    continue
                    possible_center_targets = [
                        elem for elem in possible_center_targets if elem not in [i for i in other_targets[0]]]
            elif len(other_targets) == 3 and len(non_targets) == 3:
                center_targets.append((WIDGET4, cell))
                continue
                possible_center_targets = [
                    elem for elem in possible_center_targets if elem not in [i for i in other_targets[0]]]
            elif len(other_targets) == 2 and len(non_targets) == 4:
                # print("Nå ja")
                if self.is_opposite(cell, other_targets[0][0], other_targets[0][1], other_targets[1][0], other_targets[1][1]):
                    center_targets.append((WIDGET3, cell))
                    continue
                    possible_center_targets = [
                        elem for elem in possible_center_targets if elem not in [i for i in other_targets[0]]]
        count = {WIDGET3: 0, WIDGET4: 0, WIDGET5: 0}
        for cell in center_targets:
            if cell[0] == WIDGET3:
                count[WIDGET3] += 1
            elif cell[0] == WIDGET4:
                count[WIDGET4] += 1
            elif cell[0] == WIDGET5:
                count[WIDGET5] += 1

        for key in count.keys():
            if count[key] >= 2:
                # print("mhm")
                # print(center_targets)
                first = center_targets[0]
                last = center_targets[-1]
                center_targets[0] = last
                center_targets[-1] = first
                # print(center_targets)

        # this could be changed to for example find the average of all the target cells
        if len(center_targets) == 0:
            center_targets.append((WIDGET4, (0, 1)))
            center_targets.append((WIDGET3, (2, 0)))
            center_targets.append((WIDGET5, (2, 2)))
            count[WIDGET3] = 1
            count[WIDGET4] = 1
            count[WIDGET5] = 1

        self.center_targets = center_targets
        self.widget_count = count
        # print("Widget_count", self.widget_count)

    # def init_target_cells(self):
    #     self.target_cells = []
    #     for cell in self.center_targets:
    #         for ori in ROBOT_ORIENTATIONS:
    #             adjecent = get_adjacent_cell_coords(cell, ori)
    #     return

    def solve_ucs(self):
        """
        Find a path which solves the environment using Uniform Cost Search (UCS).
        :return: path (list of actions, where each action is an element of ROBOT_ACTIONS)
        """

        possible_actions = self.possible_actions(
            self.environment.get_init_state())
        for action in possible_actions:
            success, cost, new_state = self.environment.perform_action(
                self.environment.get_init_state(), action)
            heapq.heappush(self.frontier, (cost, [
                           new_state.robot_posit, new_state.robot_orient, new_state.widget_centres, new_state.widget_orients]))
            self.parents[new_state] = [
                action, self.environment.get_init_state(), cost]

        while len(self.frontier) > 0:
            self.loop_counter.inc()
            frontie_cost, frontie_state = heapq.heappop(self.frontier)
            current_state = State(
                self.environment, frontie_state[0], frontie_state[1], frontie_state[2], frontie_state[3])
            if self.environment.is_solved(current_state):
                print("UCS Self.frontier length: ", len(self.frontier))
                print("UCS Self.partens length: ", len(self.parents))
                return self.construct_path(current_state)
            for action in self.possible_actions(current_state):
                success, cost, new_state = self.environment.perform_action(
                    current_state, action)
                total_cost = frontie_cost + cost
                if new_state in self.parents.keys():
                    if total_cost < self.parents[new_state][2]:
                        heapq.heappush(self.frontier, (total_cost, [
                                       new_state.robot_posit, new_state.robot_orient, new_state.widget_centres, new_state.widget_orients]))
                        self.parents[new_state] = [
                            action, current_state, total_cost]
                else:
                    self.parents[new_state] = [
                        action, current_state, total_cost]
                    heapq.heappush(self.frontier, (total_cost, [
                                   new_state.robot_posit, new_state.robot_orient, new_state.widget_centres, new_state.widget_orients]))

    def solve_a_star(self):
        """
        Find a path which solves the environment using A* search.
        :return: path (list of actions, where each action is an element of ROBOT_ACTIONS)
        """

        possible_actions = self.possible_actions(
            self.environment.get_init_state())
        for action in possible_actions:
            success, cost, new_state = self.environment.perform_action(
                self.environment.get_init_state(), action)
            heapq.heappush(self.frontier, (cost, [
                           new_state.robot_posit, new_state.robot_orient, new_state.widget_centres, new_state.widget_orients]))
            self.parents[new_state] = [
                action, self.environment.get_init_state(), cost]

        while len(self.frontier) > 0:
            self.loop_counter.inc()
            frontie_cost, frontie_state = heapq.heappop(self.frontier)
            current_state = State(
                self.environment, frontie_state[0], frontie_state[1], frontie_state[2], frontie_state[3])
            if self.environment.is_solved(current_state):
                # print("A* Self.frontier length: ", len(self.frontier))
                # print("A* Self.partens length: ", len(self.parents))
                # self.heuristic14(current_state)
                return self.construct_path(current_state)
            for action in self.possible_actions(current_state):
                success, cost, new_state = self.environment.perform_action(
                    current_state, action)

                total_cost = frontie_cost + cost + \
                    self.heuristic1(new_state)

                if new_state in self.parents.keys():
                    if total_cost < self.parents[new_state][2]:
                        heapq.heappush(self.frontier, (total_cost, [
                                       new_state.robot_posit, new_state.robot_orient, new_state.widget_centres, new_state.widget_orients]))
                        self.parents[new_state] = [
                            action, current_state, total_cost]
                else:
                    self.parents[new_state] = [
                        action, current_state, total_cost]
                    heapq.heappush(self.frontier, (total_cost, [
                                   new_state.robot_posit, new_state.robot_orient, new_state.widget_centres, new_state.widget_orients]))

    def heuristic1(self, state):
        # how many target cell are filled with the widgets
        environment = self.environment
        widget_cells = [widget_get_occupied_cells(environment.widget_types[i], state.widget_centres[i],
                                                  state.widget_orients[i]) for i in range(environment.n_widgets)]
        # loop over each target
        tgt_unsolved = 0
        for tgt in environment.target_list:
            if tgt not in widget_cells:
                tgt_unsolved += 1
        return tgt_unsolved / 3

    def heuristic2(self, state):
        # total distance from robot to target cells that not is covered by widget
        environment = self.environment
        widget_cells = set([widget_get_occupied_cells(environment.widget_types[i], state.widget_centres[i],
                                                      state.widget_orients[i]) for i in range(environment.n_widgets)][0])
        tgt_cells_not_solved_cells = set()
        distance = 0
        for tgt in environment.target_list:  # loops the targets-cells
            if tgt not in widget_cells:
                # adds the target cell if no widget is in it
                tgt_cells_not_solved_cells.add(tgt)
        for cell in tgt_cells_not_solved_cells:  # loops target-cells not solved
            distance += self.compute_euclidean_distance(
                cell, state.robot_posit)  # adds euclidean distance between robot and that cell
        return distance

    def heuristic3(self, state):
        environment = self.environment
        distance = 0
        for center in state.widget_centres:  # loops every widget center
            closest = 9999  # defines a max distance
            for tgt in environment.target_list:  # loops evert target
                curr_dis = self.compute_manhattan_distance(center, tgt) * 1.5
                if curr_dis < closest:
                    closest = curr_dis  # updates cloest target
            distance += closest  # adds the closest target-path
        return distance

    def heuristic4(self, state):
        # distance from robot to each target cell that is not covered by a widget, and check adjecent target cells if they also are targets (skips these)
        environment = self.environment
        widget_cells = set([widget_get_occupied_cells(environment.widget_types[i], state.widget_centres[i],
                                                      state.widget_orients[i]) for i in range(environment.n_widgets)][0])
        tgt_cells_not_solved_cells = set()
        for tgt in environment.target_list:
            if tgt not in widget_cells:
                tgt_cells_not_solved_cells.add(tgt)

        distance = 0
        adjecents = set()
        for cell in tgt_cells_not_solved_cells:
            for ori in ROBOT_ORIENTATIONS:
                adjecent = get_adjacent_cell_coords(cell, ori)
                if adjecent in tgt_cells_not_solved_cells and adjecent not in adjecents:
                    distance += self.compute_manhattan_distance(
                        cell, state.robot_posit)*0.33
                    adjecents.add(adjecent)
                    adjecents.add(cell)

        return distance

    def heuristic5(self, state):
        # Computes the distance from each target to a random widget
        environment = self.environment
        widget_cells = [widget_get_occupied_cells(environment.widget_types[i], state.widget_centres[i],
                                                  state.widget_orients[i]) for i in range(environment.n_widgets)][0]
        distance = 0
        for tgt in environment.target_list:  # enumerate all targets
            if tgt not in widget_cells:  # see is target not equal to one of the widget cells
                distance += self.compute_manhattan_distance2(
                    tgt, random.choice(widget_cells))*1.5  # minimum cost per cell to move a widget
        return distance

    def heuristic11(self, state):
        environment = self.environment
        target_list = environment.target_list.copy()
        target_list_cells = []
        for i in self.environment.widget_types:
            ele1 = []
            for j in range(int(i)):
                if len(target_list) > 0:
                    ele1.append(target_list.pop(0))
            target_list_cells.append(ele1)
        target_list_cells = [x for x in target_list_cells if x != []]

        o = []
        for x in range(len(target_list_cells)):
            # dis = []
            for i in target_list_cells[x]:
                o.append(self.comupte_noe(
                    i, state.widget_centres[x]))
            # o.append(min(dis))
        return min(o)

    def heuristic13(self, state):
        env = self.environment
        summ = 0
        center_targets = self.center_targets.copy()

        for i in center_targets:
            index = env.widget_types.index(i[0])
            if center_targets.index(i) > 0:
                res = next(x for x in range(len(center_targets))
                           if x > 0)
                # index = env.widget_types.index(i, [1, -1])
                index = res

            summ += self.compute_euclidean_distance(
                i[1], state.widget_centres[index])
            # center_targets.remove(i)
        return summ

    def heuristic14(self, state):
        env = self.environment
        summ = 0
        indices = set()
        center_targets = self.center_targets.copy()

        for i in center_targets:
            print("Summ")
            # print(i[0])
            index = env.widget_types.index(i[0])
            if center_targets.index(i) > 0:
                res = next(x for x in range(len(center_targets))
                           if x > 0)
                # index = env.widget_types.index(i, [1, -1])
                index = res
            print(i)
            print(state.widget_centres[index])
            # print(env.widget_types)
            # print(self.center_targets)
            # print(index)
            # print(state.widget_centres)

            # center_targets.remove(i)
            print(center_targets)

    def possible_actions(self, state):
        possible_actions = []

        for action in ROBOT_ACTIONS:  # finds possible actions to take from current state
            if self.environment.perform_action(state, action)[0]:
                possible_actions.append(action)
        return possible_actions

    def equal_states(self, state1, state2):
        if (state1 == None) or (state2 == None):
            return False
        if state1.robot_posit != state2.robot_posit:
            return False
        if state1.robot_orient != state2.robot_orient:
            return False
        if state1.widget_centres != state2.widget_centres:
            return False
        if state1.widget_orients != state2.widget_orients:
            return False
        return True

    def construct_path(self, state):
        path = []
        while self.parents[state][1] != None:
            move = str(self.parents[state][0])
            if move == 0:
                move = 1
            elif move == 1:
                move = 0
            elif move == 2:
                move = 3
            elif move == 3:
                move = 2
            path.insert(0, int(move))

            state = self.parents[state][1]
        return path

    def compute_manhattan_distance(self, coor1, coor2):
        return abs(coor1[0] - coor2[0]) + abs(coor1[1] - coor2[1])

    def compute_euclidean_distance(self, coor1, coor2):
        return math.sqrt((abs(coor1[0] - coor2[0]))**2 + (abs(coor1[1] - coor2[1]))**2)

    def comupte_noe(self, coor1, coor2):
        return max(abs(coor1[0] - coor2[0]), abs(coor1[1] - coor2[1]))

    def compute_manhattan_distance2(self, coor1, coor2):
        return (abs(coor2[0]-coor1[0]) + abs(coor1[0] + coor1[1] - coor2[0] - coor2[1]) + abs(coor2[1] - coor1[1]))/2

    def is_opposite(self, center, cell1, ori1, cell2, ori2):
        if ori1 == ROBOT_UP:
            return ori2 == ROBOT_DOWN
        elif ori1 == ROBOT_UP:
            return ori2 == ROBOT_DOWN
        elif ori1 == ROBOT_DOWN:
            return ori2 == ROBOT_UP
        elif ori1 == ROBOT_UP_LEFT:
            return ori2 == ROBOT_DOWN_RIGHT
        elif ori1 == ROBOT_DOWN_RIGHT:
            return ori2 == ROBOT_UP_LEFT
        elif ori1 == ROBOT_UP_RIGHT:
            return ori2 == ROBOT_DOWN_LEFT
        elif ori1 == ROBOT_DOWN_LEFT:
            return ori2 == ROBOT_UP_RIGHT

        return False
