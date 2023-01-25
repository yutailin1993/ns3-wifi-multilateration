import random
import math
import numpy as np
import json
import sys

from collections import defaultdict

num_nodes = 12


class LocMap:
    def __init__(self, nodes_pos, K=3):
        self.nodes_pos = nodes_pos
        self.nodes_num = nodes_pos.shape[0]
        self.graph = np.zeros((self.nodes_num, self.nodes_num), dtype=np.uint8)
        self.K = K
        self.dist_matrix = self._construct_dist_matrix()
    
    def _euclidean_dist(self, u, v):
        return math.dist(self.nodes_pos[u], self.nodes_pos[v])
    
    def _construct_dist_matrix(self):
        dist_matrix = np.zeros((self.nodes_num, self.nodes_num), dtype=np.float32)
        for i in range(self.nodes_num):
            for j in range(self.nodes_num):
                dist_matrix[i][j] = self._euclidean_dist(i,j)
                
        return dist_matrix
                
    def _add_edge(self, u, v):
        self.graph[u][v] = 1
        self.graph[v][u] = 1
    
    def get_graph(self):
        return self.graph
    
    def get_dist_matrix(self):
        return self.dist_matrix
    
    def _plot(self):
        plt.figure(figsize=(8,6))

        # ax = plt.axes()

        # ax.set_rasterized(True)

        plt.xlabel('X axis (m)', fontsize=16)
        plt.ylabel('Y axis (m)', fontsize=16)
        # plt.title('max time vs number of device')

        for sta in self.nodes_pos:
            plt.plot(sta[0], sta[1], 'o', markersize='6', color='Blue')

        for node_v in range(self.nodes_num):
            for node_u in range(self.nodes_num):
                if (self.graph[node_v][node_u] == 1):
                    plt.plot([nodes_pos[node_v][0], nodes_pos[node_u][0]],
                             [nodes_pos[node_v][1], nodes_pos[node_u][1]],
                             'ro-', markersize='1', color='Green')


        # plt.xticks(x_axis[1:], fontsize=14)
        # plt.yticks([6,7,8,9,10,11,12,13,14], fontsize=14)
        plt.grid()
        plt.legend(fontsize=14)

        # plt.plot()
        plt.show()
    
    def make_bilateration_ordering(self, S, U):
        def _get_candidate(candidate_list, graph_mask):
            candidate = -1
            _max = -1
            for i in candidate_list:
                if _max < np.count_nonzero(graph_mask[i]):
                    _max = np.count_nonzero(graph_mask[i])
                    candidate = i
            return candidate

        print ("no bilateration ordering")
        add_list = []

        graph_mask = self.graph.copy()
        for i in S:
            graph_mask[:, i] = 0
            graph_mask[i, :] = 0

        for i in U:
            if np.intersect1d(np.nonzero(self.graph[i])[0], S).shape[0] >= 1:
                add_list.append(i)

        candidate = _get_candidate(add_list, graph_mask)
        add_cand = -1
        _min = 1000
        for i in S:
            if (self.dist_matrix[candidate, i] < _min and self.graph[candidate, i] == 0):
                add_cand = i
                _min = self.dist_matrix[candidate, i]
        
        print ("candidate: {}, add_candidate: {}".format(candidate, add_cand))
        # print (self.graph)
        self._add_edge(candidate, add_cand)
        # print (self.graph)

    def bilateration_ordering_check(self):
        S = []
        U = [i for i in range(self.nodes_num)]

        while (len(U) > 0):
            candidate = pick_next_node(S, U, self.graph)
            if candidate == -1:
                self.make_bilateration_ordering(S, U)
                continue
                
            S.append(candidate)
            U.remove(candidate)
        
        return S

    
    def BFS(self, s):
        visited = [False] * (self.nodes_num)
        queue = []
        
        queue.append(s)
        
        flag = 0
        
        while not all(node is True for node in visited):
            if not queue:
                for idx in range(self.nodes_num):
                    if not visited[idx]:
                        queue.append(idx)
                        break
                continue

            s = queue.pop(0)
            visited[s] = True
            k = self.K - np.count_nonzero(self.graph[s])
            dist_rank = np.argsort(self.dist_matrix[s])
            
            
            for i, idx in enumerate(dist_rank):
                if k == 0:
                    break

                if all(node is True for node in visited):
                    if idx != s and self.graph[s][idx] != 1:
                        self._add_edge(s, idx)
                        k -= 1
                else:
                    # if i != self.nodes_num-1 and np.count_nonzero(self.graph[idx]) >= 3:
                    #     continue
                
                    if idx != s and self.graph[s][idx] != 1:
                        self._add_edge(s, idx)
                        k -= 1
                        if not visited[idx] and idx not in queue:
                            queue.append(idx)


class ConflictGraph:
    def __init__(self, node_num, nodes_pos, dist_matrix, p2p_graph, link_edges):
        self.nodes_pos = nodes_pos
        self.node_num = node_num
        self.dist_matrix = dist_matrix
        self.p2p_graph = p2p_graph
        self.num_edges = int(np.count_nonzero(p2p_graph == 1) / 2)
        self.conflict_map = np.zeros((self.num_edges, self.num_edges), dtype=np.int8)
        self.link_edges = link_edges
        self.independent_sets = defaultdict(list)
    
    def construct_conflict_graph(self):
        def get_nodes_in_range(node_u, node_v, dist):
            conflict_nodes = set()
            
            for node in range(self.node_num):
                if node == node_u or node == node_v:
                    continue
                
                if self.dist_matrix[node][node_u] <= dist or self.dist_matrix[node][node_v] <= dist:
                    conflict_nodes.add(node)
            return conflict_nodes
        
        def get_edges_by_node(node_u):
            edges = set()
            
            for edge_no in self.link_edges:
                if (node_u in list(self.link_edges[edge_no].values())[:-1]):
                    edges.add(edge_no)

            return edges
        
        for edge_no in range(self.num_edges):
            node_u = self.link_edges[edge_no]['u']
            node_v = self.link_edges[edge_no]['v']
            dist = self.link_edges[edge_no]['dist']
            
            conflict_edges = set()

            
            conflict_nodes = get_nodes_in_range(node_u, node_v, dist)
            conflict_nodes.add(node_u)
            conflict_nodes.add(node_v)
            
            for node in conflict_nodes:
                conflict_edges.update(get_edges_by_node(node))

            conflict_edges.remove(edge_no)
            
            # print ("node u: {}, node v: {}, \n conflict_nodes: {}, \n conflict_edges: {}".format(
            #         node_u, node_v, conflict_nodes, conflict_edges))
            
            for edge_idx in conflict_edges:
                self.conflict_map[edge_no][edge_idx] = 1
                self.conflict_map[edge_idx][edge_no] = 1

    def construct_independent_sets_by_edges(self):
        conflict_map = np.copy(self.conflict_map)
        max_vector = np.full(self.num_edges, self.num_edges+1, dtype=np.int8)

        for edge_idx in range(self.num_edges):
            edges = [x for x in range(self.num_edges)]
            conf_map = np.copy(conflict_map)
            independent_set = []
            
            candidate_set = edges.copy()
            independent_set.append(edge_idx)
            self.remove_node_adjacent_nodes(edge_idx, conf_map, candidate_set)

            while candidate_set:
                independent_set.append(self.get_select_node_and_remove_adjacency(conf_map, candidate_set))
            
            self.independent_sets[edge_idx] = independent_set

    def remove_node_adjacent_nodes(self, node_u, conf_map, candidate_set):
        max_vector = np.full(self.num_edges, self.num_edges+1, dtype=np.int8)

        remove_set = []
        for node_v in candidate_set:
            if conf_map[node_u][node_v] == 1:
                remove_set.append(node_v)

        candidate_set.remove(node_u)
        for i in remove_set:
            candidate_set.remove(i)
            conf_map[i] = max_vector

            for j in candidate_set:
                conf_map[j][i] = 0

        conf_map[node_u] = max_vector

    def get_select_node_and_remove_adjacency(self, conf_map, candidate_set):
        conflict_counts = np.count_nonzero(conf_map, axis=1)
        
        for i in range(self.num_edges):
            if i not in candidate_set:
                conflict_counts[i] += self.num_edges
        min_edge_no = conflict_counts.argmin()
        
        # print (candidate_set, min_edge_no)

        self.remove_node_adjacent_nodes(min_edge_no, conf_map, candidate_set)

        return min_edge_no

    def construct_independent_sets(self):
        edges = [x for x in range(self.num_edges)]
        conflict_map = np.copy(self.conflict_map)
        max_vector = np.full(self.num_edges, self.num_edges+1, dtype=np.int8)
        set_idx = 0
        
        while edges:
            conf_map = np.copy(conflict_map)
            candidate_set = edges.copy()
            
            independent_set = []
            
            while candidate_set:
                independent_set.append(self.get_select_node_and_remove_adjacency(conf_map, candidate_set))
                
            for node in independent_set:
                edges.remove(node)
                conflict_map[node] = max_vector
                for i in edges:
                    conflict_map[i][node] = 0
            
            self.independent_sets[set_idx] = independent_set
            set_idx += 1
            
    def get_independent_sets(self):
        return self.independent_sets

    def get_independent_peer_links(self):
        independent_peer_links = []
        for row in range(self.num_edges):
            links = []
            for edge_no in self.independent_sets[row]:
                links.append(self.link_edges[edge_no]['u'])
                links.append(self.link_edges[edge_no]['v'])
            
            independent_peer_links.append(links)
        
        return independent_peer_links
    
    def get_conflict_graph(self):
        return self.conflict_map


def generate_nodes_pos(node_num, xy_min, xy_max):
    nodes_pos = np.random.uniform(low=xy_min, high=xy_max, size=(node_num, 2))

    return nodes_pos

def generate_complete_dist_graph(nodes_pos):
    node_num = nodes_pos.shape[0]
    graph = np.zeros((node_num, node_num))

    for i in range(node_num):
        for j in range(node_num):
            if i == j:
                continue

            graph[i][j] = euclidean_dist(nodes_pos[i], nodes_pos[j])

    return graph


class DistMatrixReconstruction:
    def __init__(self, D_u, nodes_num):
        self.D_u = D_u
        self.nodes_num = nodes_num
        self.nodes_loc = np.zeros((nodes_num, 2))

    def sequential_multilateration(self):
        # U = S.copy()
        # K = []

        # print (U)

        # for candidate in U:
        #     if len(K) == 0:
        #         self.nodes_loc[candidate][0] = 0
        #         self.nodes_loc[candidate][1] = 0
        #     elif len(K) == 1:
        #         assert(self.D_u[K[0], candidate] > 0)
        #         self.nodes_loc[candidate][0] = self.D_u[K[0], candidate]
        #         self.nodes_loc[candidate][1] = 0
        #     elif len(K) == 2:
        #         assert(self.D_u[K[0], candidate] > 0 and self.D_u[K[1], candidate] > 0)
        #         self.nodes_loc[candidate][0] = self.dual_lateration(K, candidate)
        #     else: # greater or equal to 3 nodes
        #         adjList = np.intersect1d(K, np.nonzero(self.D_u[candidate])[0])
        #         assert(adjList.shape[0] >= 3)
        #         self.nodes_loc[candidate] = self.tri_lateration(adjList[:3], candidate)

        S = []
        U = [i for i in range(self.nodes_num)]
        
        for _ in range(self.nodes_num):
            candidate = pick_next_node(S, U, self.D_u)
            if len(S) == 0:
                self.nodes_loc[candidate][0] = 0
                self.nodes_loc[candidate][1] = 0
            elif len(S) == 1:
                assert(self.D_u[S[0], candidate] > 0)
                self.nodes_loc[candidate][0] = self.D_u[S[0], candidate]
                self.nodes_loc[candidate][1] = 0
            elif len(S) == 2:
                assert(self.D_u[S[0], candidate] > 0 and self.D_u[S[1], candidate] > 0)
                self.nodes_loc[candidate] = self.dual_lateration(S, candidate)
            else: # greater or equal to 3 nodes
                adjList = np.intersect1d(S, np.nonzero(self.D_u[candidate])[0])
                assert(adjList.shape[0] >= 2)
                if adjList.shape[0] == 2: # bilateration
                    self.nodes_loc[candidate] = self.dual_lateration(S, candidate)
                else: # trilateration
                    self.nodes_loc[candidate] = self.tri_lateration(adjList[:3], candidate)
            
            S.append(candidate)
            U.remove(candidate)
    
    def compute_Dcur(self):
        D = np.zeros((self.nodes_num, self.nodes_num))
        for i in range(self.nodes_num):
            for j in range(self.nodes_num):
                D[i, j] = math.sqrt(pow((self.nodes_loc[i][0]-self.nodes_loc[j][0]), 2) + pow((self.nodes_loc[i][1]-self.nodes_loc[j][1]), 2))
        assert(np.allclose(D, D.transpose(), rtol=1e-05, atol=1e-08))
        assert(np.trace(D) == 0)
        return D

    def generate_distance_matrices(self):
        D = np.zeros((self.nodes_num, self.nodes_num))
        D_cur = self.compute_Dcur()
        for i in range(self.nodes_num):
            for j in range(self.nodes_num):
                D[i, j] = self.D_u[i, j] if self.D_u[i, j] != 0 and i != j else D_cur[i, j]
        
        return D, D_cur

    def update_location(self, i, j, d, gradient_ratio):
        if self.nodes_loc[j][0] == self.nodes_loc[i][0]:
            sign = np.sign(self.nodes_loc[j][1] - self.nodes_loc[i][1])
            self.nodes_loc[j][1] += sign*gradient_ratio*d
        else:
            slope_r = (self.nodes_loc[j][1] - self.nodes_loc[i][1]) / (self.nodes_loc[j][0] - self.nodes_loc[i][0])
            sign = np.sign(self.nodes_loc[j][0] - self.nodes_loc[i][0])
            self.nodes_loc[j][0] += sign*gradient_ratio*d/math.sqrt(1+pow(slope_r, 2))
            self.nodes_loc[j][1] += sign*gradient_ratio*d*slope_r/math.sqrt(1+pow(slope_r, 2))

    def rmse(self, A, B):
        rmse = np.sqrt(np.mean((A-B)**2))
        return rmse

    def EDM_Completion(self, stop_condition, gradient_ratio):
        self.sequential_multilateration()
        D, D_cur = self.generate_distance_matrices()

        cnt = 0

        while (self.rmse(D, D_cur) >= stop_condition and cnt <= 500):
            print ("cnt: {}, Curr rmse: {}".format(cnt, self.rmse(D, D_cur)))
            delta_D = D - D_cur
            for i in range(self.nodes_num):
                for j in range(i+1, self.nodes_num):
                    self.update_location(i, j, delta_D[i, j], gradient_ratio)
            
            D, D_cur = self.generate_distance_matrices()
            cnt += 1

        return D

    def dual_lateration(self, adjNodes, candidate):
        def _get_intersections(node_loc_0, r0, node_loc_1, r1):
            # circle 1: (x0, y0), radius r0
            # circle 2: (x1, y1), radius r1
            x0 = node_loc_0[0]; y0 = node_loc_0[1]
            x1 = node_loc_1[0]; y1 = node_loc_1[1]
            d=math.sqrt((x1-x0)**2 + (y1-y0)**2)
    
            # non intersecting
            if d > r0 + r1 :
                ValueError("non intersecting")
            # One circle within other
            if d < abs(r0-r1):
                ValueError(("one circle within other"))
            # coincident circles
            if d == 0 and r0 == r1:
                ValueError(("coincident circles"))

            a=(r0**2-r1**2+d**2)/(2*d)
            h=math.sqrt(abs(r0**2-a**2))
            x2=x0+a*(x1-x0)/d   
            y2=y0+a*(y1-y0)/d   
            x3=x2+h*(y1-y0)/d     
            y3=y2-h*(x1-x0)/d 

            return np.array([x3, y3]) # we only need one result
        
        return _get_intersections(self.nodes_loc[adjNodes[0]], self.D_u[adjNodes[0], candidate],
                                  self.nodes_loc[adjNodes[1]], self.D_u[adjNodes[1], candidate])
    
    def tri_lateration(self, adjNodes, candidate):
        loc0 = self.nodes_loc[adjNodes[0]]
        loc1 = self.nodes_loc[adjNodes[1]]
        loc2 = self.nodes_loc[adjNodes[2]]

        A = np.array([[2*(loc0[0]-loc1[0]), 2*(loc0[1]-loc1[1])],
                      [2*(loc0[0]-loc2[0]), 2*(loc0[1]-loc2[1])]])
        
        L = np.array([pow(self.D_u[adjNodes[1], candidate], 2)-pow(self.D_u[adjNodes[0], candidate], 2)-(pow(loc1[0], 2)-pow(loc0[0], 2))-(pow(loc1[1], 2)-pow(loc0[1], 2)),
                      pow(self.D_u[adjNodes[2], candidate], 2)-pow(self.D_u[adjNodes[0], candidate], 2)-(pow(loc2[0], 2)-pow(loc0[0], 2))-(pow(loc2[1], 2)-pow(loc0[1], 2))])

        tempResult = np.matmul(np.linalg.inv(np.matmul(A.transpose(), A)), A.transpose())
        result = np.matmul(tempResult, L)

        return result.transpose()

def pick_next_node(S, U, D_u):
    def _get_candidate(candidate_list, D_mask):
        candidate = -1
        _max = -1
        for i in candidate_list:
            if _max < np.count_nonzero(D_mask[i]):
                _max = np.count_nonzero(D_mask[i])
                candidate = i
        return candidate

    D_mask = D_u.copy()
    for i in S:
        D_mask[:, i] = 0
        D_mask[i, :] = 0
    
    if len(S) == 0:
        # choose node with maximum edges
        candidate = np.argmax(np.count_nonzero(D_mask, axis=1))
        return candidate
    
    elif len(S) == 1:
        candidate_list = np.nonzero(D_u[S[0]])[0]
        candidate = _get_candidate(candidate_list, D_mask)
        
        return candidate
    
    elif len(S) >= 2:
        candidate_list = []
        for i in U:
            if np.intersect1d(np.nonzero(D_u[i])[0], S).shape[0] >= 2:
                candidate_list.append(i)
        
        candidate = _get_candidate(candidate_list, D_mask)
        # candidate_list = np.intersect1d(np.nonzero(D_u[S[0]])[0], np.nonzero(D_u[S[1]])[0])
        # candidate_list = np.intersect1d(candidate_list, U)
        # candidate = _get_candidate(candidate_list, D_mask)

        return candidate

    # else: # len(S) greater than 3
    #     candidate_list = []
    #     for i in U:
    #         if np.intersect1d(np.nonzero(D_u[i])[0], S).shape[0] >= 3:
    #             candidate_list.append(i)
    #     
    #     candidate = _get_candidate(candidate_list, D_mask)
    #     return candidate

def euclidean_dist(pos_a, pos_b):
    return math.dist(pos_a, pos_b)


def matrix_mask(target_matrix, mask_matrix):
    assert (target_matrix.shape == mask_matrix.shape)

    ret_matrix = target_matrix * mask_matrix

    return ret_matrix
    

def mds_relative_to_absolute_scale(estimated_coordinates, indices_of_anchors, anchors_true_coordinates):
    A = np.transpose(estimated_coordinates[indices_of_anchors,:])
    B = np.transpose(anchors_true_coordinates)
    estimated_coordinates = np.transpose(estimated_coordinates)

    assert (A.shape == B.shape)

    all_estimate_coordinates = estimated_coordinates
    num_of_points = estimated_coordinates.shape[1]
    
    def rigid_transform_3D(A,B):
        dimension, num_anchors = A.shape
        
        # find scale
        anchor_dist_A = np.zeros((num_anchors, num_anchors))
        anchor_dist_B = np.zeros((num_anchors, num_anchors))

        for i in range(num_anchors):
            for j in range(num_anchors):
                anchor_dist_A[i][j] = euclidean_dist(A[:, i], A[:, j])
                anchor_dist_B[i][j] = euclidean_dist(B[:, i], B[:, j])
        
        scale_matrix = np.divide(anchor_dist_B, anchor_dist_A, out=np.zeros_like(anchor_dist_A), where=anchor_dist_B!=0)
        total_n = 0
        cnt = 0
        for i in range(0, scale_matrix.shape[1]):
            for j in range(i+1, scale_matrix.shape[1]):
                total_n += scale_matrix[i, j]
                cnt += 1
        scale = total_n / cnt
        A = scale*A

        # find rotation
        centroid_A = np.expand_dims(np.mean(A, axis=1), axis=1)
        centroid_B = np.expand_dims(np.mean(B, axis=1), axis=1)
        
        diff_A = A - np.tile(centroid_A, (1, num_anchors))
        diff_B = B - np.tile(centroid_B, (1, num_anchors))
        

        H = np.matmul(diff_A, np.transpose(diff_B))

        u, _, v = np.linalg.svd(H)

        rotation = np.matmul(v, np.transpose(u))

        # find translation
        translation = np.matmul(-rotation, centroid_A) + centroid_B

        return rotation, translation, scale

    rotation, translation, scale = rigid_transform_3D(A, B)
    
    estimated_coord_mds = np.matmul(rotation, (scale*all_estimate_coordinates)) + np.tile(translation, (1, num_of_points))

    return np.transpose(estimated_coord_mds)


if __name__ == '__main__':
        nodes_pos = np.array(json.loads(sys.argv[1]))
        node_num = nodes_pos.shape[1]
        loc_pos = LocMap(nodes_pos)
        loc_pos.BFS(random.randint(0, node_num))

        p2p_graph = loc_pos.get_graph()
        dist_matrix = loc_pos.get_dist_matrix()

