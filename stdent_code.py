import math

def first(begin, cord, cord_gr):

    node = Create_Node(begin, cord , 0)
    node.Node_confige(cord_gr)
    return(begin, node)

def node_another_new(node, vr, cord_vr, cord_gr):

    distance_vr = node.dict_begin + h_cord(cord_vr, node.cord)
    new_one = Create_Node(vr, cord_vr, distance_vr, node.gr_vertex)
    new_one.Node_confige(cord_gr)
    return new_one

#Class to create Node
class Create_Node:

    def __init__(self, vertex, cord, dist = 0, prev_vertex = None):

        self.gr_vertex = vertex
        self.cord = cord
        self.dict_begin = dist
        self.heur_s = 0
        self.f_value = 0
        self.vertex_back = prev_vertex

    def set_f_value(self, cord_gr):
        #self.heur_s = h_cord(self.cord, g_cord)
        self.f_value= f_value(self.dict_begin, self.cord, cord_gr)

    def set_heur_s(self, cord_gr):
        self.heur_s = h_cord(self.cord, cord_gr)

    def Node_confige(self, cord_gr):
        self.set_heur_s(cord_gr)
        self.set_f_value(cord_gr)

    def __repr__(self):
        return('\ngr_vertex: ' + str(self.gr_vertex) + ', distance_g: ' + str(self.dict_begin) + ', heur_s: ' + str(self.heur_s) + ', F_value: ' + str(self.f_value))


def f_value(total_weight, cord_vr, cord_gr):
    heur_s = h_cord(cord_vr, cord_gr)
    return(total_weight + heur_s)


def h_cord(one, two):
    return math.sqrt( (two[1] - one[1]) ** 2 + (two[0] - one[0]) ** 2  )

def close_build(goal, closed):
    temp_way = [goal]
    back = closed[goal].vertex_back
    while back:
        temp_way.append(back)
        back = closed[back].vertex_back
    return list(reversed(temp_way))



# Implementation of A* Algorithm
# Sortest Path
def shortest_path(M, begin, target):
    vr_open = {}
    cord_gr = M.intersections[target]
    vr_close = {}
    curr, node = first(begin, M.intersections[begin], cord_gr)

    while curr is not target:
        for vertex in M.roads[curr]:
            if vr_close.get(vertex) is not None:
                continue

            v_open_node = vr_open.get(vertex, None)
            new_node = node_another_new(node, vertex, M.intersections[vertex], cord_gr)

            if v_open_node is None or new_node.f_value < v_open_node.f_value:
                vr_open[vertex] = new_node

        curr, node = change_previous(node, vr_open, vr_close, curr)
    vr_close[curr] = node

    return(close_build(target, vr_close))

# Implementation of algorithm Ends


def change_previous(node, open_, close_, curr):
    close_[curr] = node
    new_one = min(open_, key = lambda i: open_[i].f_value)
    node_n = open_[new_one]
    del open_[new_one]
    return(new_one,node_n)

#------------------------------------------------------------------------------------------------------------------------------------------
