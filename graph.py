import random
import math
import numpy as np
import json
import sys

from collections import defaultdict

from sklearn.manifold import MDS


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
    
    def make_trilateration_ordering(self, S, U):
        def _get_candidate(candidate_list, graph_mask):
            candidate = -1
            _max = -1
            for i in candidate_list:
                if _max < np.count_nonzero(graph_mask[i]):
                    _max = np.count_nonzero(graph_mask[i])
                    candidate = i
            return candidate

        # print ("no trilateration ordering", file=sys.stderr)
        add_list = []

        graph_mask = self.graph.copy()
        for i in S:
            graph_mask[:, i] = 0
            graph_mask[i, :] = 0

        add_edge_cnt = 0
        for i in U:
            if np.intersect1d(np.nonzero(self.graph[i])[0], S).shape[0] >= 2:
                add_list.append(i)
        
        add_edge_cnt += 1
        if len(add_list) == 0:
            if len(S) > 2:
                add_edge_cnt += 1
            for i in U:
                if np.intersect1d(np.nonzero(self.graph[i])[0], S).shape[0] >= 1:
                    add_list.append(i)

        assert(len(add_list) > 0)

        candidate = _get_candidate(add_list, graph_mask)
        S_cpy = S.copy()
        for _ in range(add_edge_cnt):
            add_cand = -1
            _min = sys.float_info.max
            for i in S_cpy:
                if (self.dist_matrix[candidate, i] < _min and self.graph[candidate, i] == 0):
                    add_cand = i
                    _min = self.dist_matrix[candidate, i]

            S_cpy.remove(add_cand)
            # print ("candidate: {}, add_candidate: {}".format(candidate, add_cand), file=sys.stderr)
            self._add_edge(candidate, add_cand)

    def trilateration_ordering_check(self):
        S = []
        U = [i for i in range(self.nodes_num)]

        while (len(U) > 0):
            candidate = pick_next_node(S, U, self.graph)
            if candidate == -1:
                self.make_trilateration_ordering(S, U)
                U = [i for i in range(self.nodes_num)]
                S = []
                continue
                
            S.append(candidate)
            U.remove(candidate)
        
        return S

    def HennenburgConstruction(self, s):
        U = [x for x in range(self.nodes_num)]
        S = []
        S.append(s)
        U.remove(s)

        n = np.argsort(self.dist_matrix[s])[1]
        S.append(n)
        U.remove(n)

        self._add_edge(S[0], S[1])

        while (U):
            min_dist = sys.float_info.max
            u1 = -1; s1 = -1; s2 = -1
            for i in U:
                i_list = np.argsort(self.dist_matrix[i])
                K = []
                for j in i_list:
                    if j in S:
                        K.append(j)
                    if (len(K) == 2):
                        break

                if self.dist_matrix[i][K[0]] + self.dist_matrix[i][K[1]] < min_dist:
                    u1 = i; s1 = K[0]; s2 = K[1]

            assert(u1 >= 0 and s1 >= 0 and s2 >= 0)
            self._add_edge(u1, s1)
            self._add_edge(u1, s2)
            U.remove(u1)
            S.append(u1)

    def ConstructEdgesK(self):
        for i in range(self.nodes_num):
            if np.count_nonzero(self.graph) < self.K:
                diff = self.K - np.count_nonzero(self.graph)
                assert (diff > 0)
                dist_list = np.argsort(self.dist_matrix[i])
                for j in dist_list:
                    if self.graph[i][j] == 0 and i != j:
                        self._add_edge(i, j)
                        diff -= 1
                    if diff == 0:
                        break
    
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
    def __init__(self, node_num, nodes_pos, dist_matrix, p2p_graph, link_edges, channel_bandwidth):
        self.nodes_pos = nodes_pos
        self.node_num = node_num
        self.channel_bandwidth = channel_bandwidth
        self.dist_matrix = dist_matrix
        self.p2p_graph = p2p_graph
        self.num_edges = int(np.count_nonzero(p2p_graph == 1) / 2)
        self.conflict_map = np.zeros((self.num_edges, self.num_edges), dtype=np.int8)
        self.link_edges = link_edges
        self.independent_sets = defaultdict(list)

        self.txPower_const = 5.43*pow(10,-6)
        if self.channel_bandwidth == 20:
            self.rssi_threshold = -70
        elif self.channel_bandwidth == 40:
            self.rssi_threshold = -67
        elif self.channel_bandwidth == 80:
            self.rssi_threshold = -64
        else:
            raise ValueError("Bandwidth not implemented: {}".format(self.channel_bandwidth))
    
    def construct_conflict_graph(self):
        def get_nodes_in_range(node_u, node_v, dist):
            conflict_nodes = set()
            conflict_dist = math.sqrt(self.txPower_const/pow(10,(self.rssi_threshold-30)/10))
            
            for node in range(self.node_num):
                if node == node_u or node == node_v:
                    continue
                
                if self.dist_matrix[node][node_u] <= conflict_dist or self.dist_matrix[node][node_v] <= conflict_dist:
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
        nodes_loc = np.zeros((self.nodes_num, 2))
        U = [i for i in range(self.nodes_num)]
        S = []

        for _ in range(self.nodes_num):
            candidate = pick_next_node(S, U, self.D_u)
            if len(S) == 0:
                nodes_loc[candidate][0] = 0
                nodes_loc[candidate][1] = 0
            elif len(S) == 1:
                assert(self.D_u[S[0], candidate] > 0)
                nodes_loc[candidate][0] = self.D_u[S[0], candidate]
                nodes_loc[candidate][1] = 0
            elif len(S) == 2:
                assert(self.D_u[S[0], candidate] > 0 and self.D_u[S[1], candidate] > 0)
                nodes_loc[candidate] = self.bilateration(S, candidate, nodes_loc)
            else: # greater or equal to 3 nodes
                adjList = np.intersect1d(S, np.nonzero(self.D_u[candidate])[0])
                assert(adjList.shape[0] >= 3)
                temp_nodes_loc = np.zeros(2)
                min_offset = sys.float_info.max
                for _ in range(3, len(adjList)+1):
                    temp_nodes_loc += self.tri_lateration(np.random.choice(adjList, 3, replace=False), candidate, nodes_loc)
                temp_nodes_loc /= (len(adjList)-3+1)
                nodes_loc[candidate] = temp_nodes_loc

            U.remove(candidate)
            S.append(candidate)
        
        return nodes_loc

    

    def generate_distance_matrices(self, nodes_loc):
        def _compute_Dcur(nodes_num, nodes_loc):
            D = np.zeros((nodes_num, nodes_num))
            for i in range(nodes_num):
                for j in range(i+1, nodes_num):
                    dist = math.sqrt(pow((nodes_loc[i][0]-nodes_loc[j][0]), 2) + pow((nodes_loc[i][1]-nodes_loc[j][1]), 2))
                    D[i, j] = dist
                    D[j, i] = dist

            assert(np.allclose(D, D.transpose(), rtol=1e-05, atol=1e-08))
            assert(np.trace(D) == 0)
            return D

        D = np.zeros((self.nodes_num, self.nodes_num))
        D_cur = _compute_Dcur(self.nodes_num, nodes_loc)
        for i in range(self.nodes_num):
            for j in range(self.nodes_num):
                D[i, j] = self.D_u[i, j] if self.D_u[i, j] != 0 and i != j else D_cur[i, j]
        
        return D, D_cur

    def update_location(self, i, j, d, gradient_ratio, nodes_loc):
        if nodes_loc[j][0] == nodes_loc[i][0]:
            sign = np.sign(nodes_loc[j][1] - nodes_loc[i][1])
            nodes_loc[j][1] += sign*gradient_ratio*d
        else:
            slope_r = (nodes_loc[j][1] - nodes_loc[i][1]) / (nodes_loc[j][0] - nodes_loc[i][0])
            sign = np.sign(nodes_loc[j][0] - nodes_loc[i][0])
            nodes_loc[j][0] += sign*gradient_ratio*d/math.sqrt(1+pow(slope_r, 2))
            nodes_loc[j][1] += sign*gradient_ratio*d*slope_r/math.sqrt(1+pow(slope_r, 2))

    def EDM_Completion(self, stop_condition, gradient_ratio, stop_itrr):
        candidate_nodes_loc = np.zeros((self.nodes_num, 2))
        candidate_rmse_D = sys.float_info.max
        for _ in range(30):
            temp_nodes_loc = self.sequential_multilateration()
            temp_D, temp_D_cur = self.generate_distance_matrices(temp_nodes_loc)

            cnt = 0

            while (rmse(temp_D, temp_D_cur) >= stop_condition and cnt <=stop_itrr):
                temp_delta_D = temp_D - temp_D_cur
                for i in range(self.nodes_num):
                    for j in range(i+1, self.nodes_num):
                        self.update_location(i, j, temp_delta_D[i, j], gradient_ratio, temp_nodes_loc)
                
                temp_D, temp_D_cur = self.generate_distance_matrices(temp_nodes_loc)
                cnt += 1

            # print ("candidate_rmse_D: {}, Curr rmse: {}".format(candidate_rmse_D, rmse(temp_D, temp_D_cur)), file=sys.stderr)
            if candidate_rmse_D >= rmse(temp_D, temp_D_cur):
                candidate_nodes_loc = temp_nodes_loc
                candidate_rmse_D = rmse(temp_D, temp_D_cur)
        
        self.nodes_loc = candidate_nodes_loc
        D, D_cur = self.generate_distance_matrices(self.nodes_loc)
        return D

    def Get_nodes_loc(self):
        return self.nodes_loc

    def bilateration(self, adjNodes, candidate, nodes_loc):
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
        
        return _get_intersections(nodes_loc[adjNodes[0]], self.D_u[adjNodes[0], candidate],
                                  nodes_loc[adjNodes[1]], self.D_u[adjNodes[1], candidate])
    
    def tri_lateration(self, adjNodes, candidate, nodes_loc):
        # def matrixInverse(inputM):
        #     e0 = inputM[0, 0]
        #     e1 = inputM[0, 1]
        #     e2 = inputM[1, 0]
        #     e3 = inputM[1, 1]
        #     m = 1.0 / (e0*e3 - e1*e2)
        #     matrix = np.array([[m*e3, m*(-e1)], [m*(-e2), m*e0]])
        #     return matrix

        loc0 = nodes_loc[adjNodes[0]]
        loc1 = nodes_loc[adjNodes[1]]
        loc2 = nodes_loc[adjNodes[2]]

        A = np.array([[2*(loc0[0]-loc1[0]), 2*(loc0[1]-loc1[1])],
                      [2*(loc0[0]-loc2[0]), 2*(loc0[1]-loc2[1])]])
        
        L = np.array([pow(self.D_u[adjNodes[1], candidate], 2)-pow(self.D_u[adjNodes[0], candidate], 2)-(pow(loc1[0], 2)-pow(loc0[0], 2))-(pow(loc1[1], 2)-pow(loc0[1], 2)),
                      pow(self.D_u[adjNodes[2], candidate], 2)-pow(self.D_u[adjNodes[0], candidate], 2)-(pow(loc2[0], 2)-pow(loc0[0], 2))-(pow(loc2[1], 2)-pow(loc0[1], 2))])


        tempResult = np.matmul(np.linalg.inv(np.matmul(A.transpose(), A)), A.transpose())
        result = np.matmul(tempResult, L)

        return result

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
    
    elif len(S) == 2:
        candidate_list = []
        for i in U:
            if np.intersect1d(np.nonzero(D_u[i])[0], S).shape[0] >= 2:
                candidate_list.append(i)
        
        candidate = _get_candidate(candidate_list, D_mask)

        return candidate

    else: # len(S) greater than 3
        candidate_list = []
        for i in U:
            if np.intersect1d(np.nonzero(D_u[i])[0], S).shape[0] >= 3:
                candidate_list.append(i)

        # if (len(candidate_list) == 0): # fall back to bilateration ordering
        #     for i in U:
        #         if np.intersect1d(np.nonzero(D_u[i])[0], S).shape[0] >= 2:
	    #          candidate_list.append(i)
        candidate = _get_candidate(candidate_list, D_mask)
        return candidate

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

        H = diff_A @ np.transpose(diff_B)

        u, _, v = np.linalg.svd(H)

        rotation = v.T @ u.T

        # find translation
        translation = np.matmul(-rotation, centroid_A) + centroid_B

        return rotation, translation, scale

    rotation, translation, scale = rigid_transform_3D(A, B)
    
    # print ("Rotation: \n{},\nTranslation: \n{},\nScale: {}".format(rotation, translation, scale))

    estimated_coord_mds = np.matmul(rotation, (scale*all_estimate_coordinates)) + np.tile(translation, (1, num_of_points))

    return np.transpose(estimated_coord_mds)

def construct_edges(node_num, p2p_graph, dist_matrix, num_edges):
    link_edges = defaultdict(dict)
    edge_no = 0
    
    labels = {}
    comm_edges_list = []

    for node_u in range(node_num):
        for node_v in range(node_u+1, node_num):
            if p2p_graph[node_u][node_v] == 1:
                link_edges[edge_no] = {'u':node_u, 'v':node_v, 'dist':dist_matrix[node_u][node_v]}
                edge_no += 1               

    assert (edge_no == num_edges)
    
    for edge_no in range(int(num_edges)):
        node_u = link_edges[edge_no]['u']
        node_v = link_edges[edge_no]['v']
        dist = link_edges[edge_no]['dist']
    
        comm_edges_list.append([node_u, node_v])
        labels[tuple([node_u, node_v])] = edge_no
    
    return link_edges, comm_edges_list, labels

def applyMDS(completeEDM):
    mds = MDS(n_components=2,
              max_iter=3000,
              eps=1e-9,
              dissimilarity="precomputed",
              normalized_stress='auto')

    node_locations = mds.fit_transform(completeEDM)

    return node_locations

def rmse(A, B):
    rmse = np.sqrt(np.mean((A-B)**2))
    return rmse

if __name__ == '__main__':
    if (sys.argv[1] == '--idSets'):
        nodes_pos = np.array(json.loads(sys.argv[2]))

        # np.savetxt('node_pos.csv', nodes_pos, delimiter=',')
        channel_bandwidth = int(sys.argv[3])
        node_num = nodes_pos.shape[0]
        loc_pos = LocMap(nodes_pos, K=4)
        loc_pos.HennenburgConstruction(random.randint(0, node_num-1))
        loc_pos.ConstructEdgesK()
        loc_pos.trilateration_ordering_check()

        p2p_graph = loc_pos.get_graph()
        dist_matrix = loc_pos.get_dist_matrix()

        # np.savetxt('dist_matrix.csv', dist_matrix, delimiter=',')

        link_edges, comm_edges_list, labels = construct_edges(node_num, p2p_graph, dist_matrix, np.count_nonzero(p2p_graph == 1)/2)

        conf_graph_obj = ConflictGraph(node_num, nodes_pos, dist_matrix, p2p_graph, link_edges, channel_bandwidth)
        conf_graph_obj.construct_conflict_graph()
        conf_graph_obj.construct_independent_sets_by_edges()
        
        independent_sets_peer_links = conf_graph_obj.get_independent_peer_links()
        
        for row in independent_sets_peer_links:
            print (" ".join(str(x) for x in row))

    elif (sys.argv[1] == '--getLocs'):
        measured_dist_matrix = np.array(json.loads(sys.argv[2]))
        anchor_nodes_pos = np.array(json.loads(sys.argv[3]))
        anchor_nodes_idx = np.array(json.loads(sys.argv[4]))
        # active_EDM = np.array(json.loads(sys.argv[5]))
        # passive_EDM = np.array(json.loads(sys.argv[6]))

        # assert (np.allclose(active_EDM, active_EDM.T, rtol=10e-5, atol=10e-5))
        # assert (np.allclose(passive_EDM, passive_EDM.T, rtol=10e-5, atol=10e-5))

        # np.savetxt('measured_dist_matrix.csv', measured_dist_matrix, delimiter=',')
        # np.savetxt('active_EDM.csv', active_EDM, delimiter=',')
        # np.savetxt('passive_EDM.csv', passive_EDM, delimiter=',')

        assert (measured_dist_matrix.shape[0] == measured_dist_matrix.shape[1])

        # print (measured_dist_matrix, file=sys.stderr)

        recon = DistMatrixReconstruction(measured_dist_matrix, measured_dist_matrix.shape[0])

        D = recon.EDM_Completion(1, 0.05, 500)

        mds = applyMDS(D)
        
        estimated_pos = mds_relative_to_absolute_scale(mds, anchor_nodes_idx, anchor_nodes_pos)
        # print ("estimate_pos: \n{}".format(estimated_pos), file=sys.stderr)

        for row in estimated_pos:
            # print (" ".join("{:.2f}".format(x) for x in row), file=sys.stderr)
            print (" ".join(str(x) for x in row))

    else:
        raise("Function not impelemented!")

