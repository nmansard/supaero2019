class Graph:
    '''
    Simple Graph structure, with nodes stored as a flat list (you can put 
    any data you want to represent a node), and connectivities represented 
    as a dictionary of childrens.
    '''
    def __init__(self):
          self.q        = []  # Node configuration
          self.children = {}  # Connectivity of the graph
                              # Each node corresponds to one entry of the dict.
                              # The corresponding dict data is the list of children, i.e.
                              # nodes reachable from the parent node.
    def addNode(self, q=None):
          '''
          Create the memory to store a new edge. Initialize all components to None.
          Create an empty list of children.
          '''
          idx = len(self.children)
          self.children[idx] = []
          self.q.append(q)
          return idx

    def addEdge(self,first,second,orientation = 0):
          '''
          Add edge from first to second. Also add edge from second to first if orientation
          is null.
          '''
          assert( first in self.children and second in self.children )
          if orientation>=0: self.children[first].append(second)
          if orientation<=0: self.children[second].append(first)



def astar(graph, start, goal, gdistance = None, hdistance = None):
     '''
     Compute A* path for the input graph connecting start to goal.
     Edges might be oriented.
     gdistance is the distance function between two nodes of the graph.
     hdistance is the heuristic distance function typically called between 
     any arbitrary nodes of the graph and the goal.
     '''
     frontier         = [ (start, 0) ]    # frontier should be a sorted heap 
                                          # (use push and pop methods.
     cameFrom         = { start: None }   # represent a tree.
     costToHere       = { start: 0    }   # cost from start to current node.

     # Graph distance
     if gdistance is None:
          gdistance = lambda i1,i2: norm(graph.q[i1]-graph.q[i2])
     # Heuristic distance
     if hdistance is None:
          hdistance = lambda i1,i2: norm(graph.q[i1]-graph.q[i2])

     # Compute the path from leave to root in a tree
     pathFromTree = lambda tree,path,start,goal: \
         [start,]+path if goal==start \
         else pathFromTree(tree,[goal,]+path,start,tree[goal])

     # Push and pop in a sorted heap
     pop  = lambda heap: heapq.heappop(heap)[1]
     push = lambda heap,item,cost: heapq.heappush(heap,(cost,item))

     # A* make groth a set initially containing only the start while
     # maintaining a list of the nodes a the frontier of this set.
     # The set is iterativelly extended by heuristcally choosing in 
     # the frontier list, until the frontier reaches the goal.
     while len(frontier)>0:
          cur = pop(frontier)                   # Pick the (heuristic) max of the frontier
          if cur == goal:                       # If it is the goal: stop
               return pathFromTree(cameFrom,[], # Return the trajectory from tree <camFrom>
                                   start,goal)  # root to goal.

          for nex in graph.children[cur]:       # Expand the frontier to cur node childrens
               curCost = costToHere[cur] + gdistance(cur, nex)          # Exact cost to nex
               if nex not in costToHere or curCost < costToHere[nex]:
                    # If nex is not yet explored or if a shortest path to nex has been found
                    costToHere[nex] = curCost                           # Set cost-to-here.
                    push(frontier,nex, curCost + hdistance(goal, nex))  # Add nex to the sorted frontier
                    cameFrom[nex] = cur                                 # Add nex to tree.
                    
     # If arriving here: start and goal are not in the same connex component
     # of the graph. Return empty path.
     return []
     
