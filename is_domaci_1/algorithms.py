import random

from state import State
import heapq
import itertools
import config

class Algorithm:
    def get_path(self, state):
        pass


class ExampleAlgorithm(Algorithm):
    def get_path(self, state):
        path = []
        while not state.is_goal_state():
            possible_actions = state.get_legal_actions()
            action = possible_actions[random.randint(0, len(possible_actions) - 1)]
            path.append(action)
            state = state.generate_successor_state(action)
        return path


def action_priority_dfs(action):
                src, dst = action
                src_row, src_col = src
                dst_row, dst_col = dst

                # Sever
                if dst_row < src_row:
                    return 3
                # Istok
                elif dst_col > src_col:
                    return 4
                # Jug
                elif dst_row > src_row:
                    return 2
                # Zapad
                elif dst_col < src_col:
                    return 1
                else:
                    return 5  

class DFS_Algorithm(Algorithm):
    def get_path(self, state):
        path = []
        visited = set()

        while not state.is_goal_state():
            state_key = state.get_state_key()
            if state_key in visited:
                print("Stanje je već posećeno, nema mogućih akcija dalje.")
                return []  # Ili možete koristiti 'break' ako želite da prekidate petlju

            visited.add(state_key)

            possible_actions = state.get_legal_actions()
            if not possible_actions:
                print("Nema mogućih akcija iz trenutnog stanja.")
                return []  # Ili možete podići izuzetak

            # Definišemo prioritet akcija: sever, istok, jug, zapad
            def action_priority(action):
                src, dst = action
                src_row, src_col = src
                dst_row, dst_col = dst

                # Sever
                if dst_row < src_row:
                    return 1
                # Istok
                elif dst_col > src_col:
                    return 2
                # Jug
                elif dst_row > src_row:
                    return 3
                # Zapad
                elif dst_col < src_col:
                    return 4
                else:
                    return 5  # Ostalo

            # Sortiramo akcije po prioritetu
            possible_actions.sort(key=action_priority)

            action_taken = False
            for action in possible_actions:
                next_state = state.generate_successor_state(action)
                next_state_key = next_state.get_state_key()
                if next_state_key not in visited:
                    path.append(action)
                    state = next_state
                    action_taken = True
                    break  # Prelazimo na sledeće stanje

            if not action_taken:
                print("Sve akcije vode u već posećena stanja.")
                return []  # Ili možete podići izuzetak

        return path


class BFS_Algorithm(Algorithm):
    def get_path(self, state):
        from collections import deque

        visited = set()
        queue = deque()
        initial_state = state

        # Red će sadržati tuple (trenutno stanje, putanja do tog stanja)
        queue.append((initial_state, []))

        while queue:
            current_state, path = queue.popleft()

            # Proveravamo da li smo već posećivali ovo stanje
            state_key = current_state.get_state_key()
            if state_key in visited:
                continue
            visited.add(state_key)

            # Proveravamo da li smo dostigli cilj
            if current_state.is_goal_state():
                return path

            # Dobijamo moguće akcije iz trenutnog stanja
            possible_actions = current_state.get_legal_actions()

            # Definišemo prioritet akcija
            def action_priority(action):
                src, dst = action
                src_row, src_col = src
                dst_row, dst_col = dst
                if dst_row < src_row:
                    return 1  # Sever
                elif dst_col > src_col:
                    return 2  # Istok
                elif dst_row > src_row:
                    return 3  # Jug
                elif dst_col < src_col:
                    return 4  # Zapad
                else:
                    return 5  # Ostalo

            # Sortiramo akcije po prioritetu
            possible_actions.sort(key=action_priority)

            # Generišemo sukcesivna stanja
            for action in possible_actions:
                successor_state = current_state.generate_successor_state(action)
                successor_state_key = successor_state.get_state_key()

                if successor_state_key not in visited:
                    successor_path = path + [action]
                    queue.append((successor_state, successor_path))

        # Ako nije pronađen put, vraćamo praznu listu
        return []


class BB_Algorithm(Algorithm):
    def get_path(self, state):
        visited = {}
        heap = []
        initial_state = state
        initial_cost = 0
        initial_path = []
        counter = itertools.count()  # Inicijalizujemo counter

        # Heuristika je 0 jer je Branch and Bound ekvivalentan Uniform Cost Search-u bez heuristike
        heapq.heappush(heap, (initial_cost, next(counter), initial_state, initial_path))

        while heap:
            current_cost, _, current_state, path = heapq.heappop(heap)

            state_key = current_state.get_state_key()
            if state_key in visited and visited[state_key] <= current_cost:
                continue
            visited[state_key] = current_cost

            if current_state.is_goal_state():
                return path

            possible_actions = current_state.get_legal_actions()

            # Definišemo prioritet akcija
            def action_priority(action):
                src, dst = action
                src_row, src_col = src
                dst_row, dst_col = dst
                # Sever
                if dst_row < src_row:
                    return 1
                # Istok
                elif dst_col > src_col:
                    return 2
                # Jug
                elif dst_row > src_row:
                    return 3
                # Zapad
                elif dst_col < src_col:
                    return 4
                else:
                    return 5  # Ostalo

            # Sortiramo akcije po prioritetu
            possible_actions.sort(key=action_priority)

            for action in possible_actions:
                successor_state = current_state.generate_successor_state(action)
                successor_state_key = successor_state.get_state_key()

                action_cost = State.get_action_cost(action)
                total_cost = current_cost + action_cost

                if successor_state_key in visited and visited[successor_state_key] <= total_cost:
                    continue

                successor_path = path + [action]
                heapq.heappush(heap, (total_cost, next(counter), successor_state, successor_path))

        # Ako nije pronađen put, vraćamo praznu listu
        return []




class A_Star_Algorithm(Algorithm):
    def get_path(self, state):
        visited = {}
        heap = []
        initial_state = state
        initial_cost = 0
        initial_path = []
        counter = itertools.count()  # Inicijalizujemo tie-breaker

        h = self.heuristic(initial_state)
        f = initial_cost + h

        heapq.heappush(heap, (f, next(counter), initial_state, initial_cost, initial_path))

        while heap:
            _, _, current_state, current_cost, path = heapq.heappop(heap)

            state_key = current_state.get_state_key()
            if state_key in visited and visited[state_key] <= current_cost:
                continue
            visited[state_key] = current_cost

            if current_state.is_goal_state():
                return path

            possible_actions = current_state.get_legal_actions()

            # Definišemo prioritet akcija
            def action_priority(action):
                src, dst = action
                src_row, src_col = src
                dst_row, dst_col = dst
                # Sever
                if dst_row < src_row:
                    return 1
                # Istok
                elif dst_col > src_col:
                    return 2
                # Jug
                elif dst_row > src_row:
                    return 3
                # Zapad
                elif dst_col < src_col:
                    return 4
                else:
                    return 5  # Ostalo

            # Sortiramo akcije po prioritetu
            possible_actions.sort(key=action_priority)

            for action in possible_actions:
                successor_state = current_state.generate_successor_state(action)
                successor_state_key = successor_state.get_state_key()

                action_cost = State.get_action_cost(action)
                total_cost = current_cost + action_cost

                if successor_state_key in visited and visited[successor_state_key] <= total_cost:
                    continue

                h = self.heuristic(successor_state)
                f = total_cost + h

                successor_path = path + [action]
                heapq.heappush(heap, (f, next(counter), successor_state, total_cost, successor_path))

        # Ako nije pronađen put, vraćamo praznu listu
        return []

    def heuristic(self, state):
        # Heuristika je suma minimalnih Manhattan distanci od svakog svemirskog broda do bilo kog cilja
        spaceship_positions = self.get_positions(state.spaceships)
        goal_positions = self.get_positions(state.goals)

        total_distance = 0
        for sp in spaceship_positions:
            min_distance = min(abs(sp[0] - gp[0]) + abs(sp[1] - gp[1]) for gp in goal_positions)
            total_distance += min_distance

        return total_distance

    def get_positions(self, bitmask):
        positions = []
        for index in range(config.M * config.N):
            if bitmask & (1 << index):
                row = index // config.N
                col = index % config.N
                positions.append((row, col))
        return positions
