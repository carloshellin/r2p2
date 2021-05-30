
import path_planning as pp

def children(point,grid):
    """
        Calculates the children of a given node over a grid.
        Inputs:
            - point: node for which to calculate children.
            - grid: grid over which to calculate children.
        Outputs:
            - list of children for the given node.
    """
    x,y = point.grid_point
    if x > 0 and x < len(grid) - 1:
        if y > 0 and y < len(grid[0]) - 1:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x,y + 1),(x+1,y),\
                      (x-1, y-1), (x-1, y+1), (x+1, y-1),\
                      (x+1, y+1)]]
        elif y > 0:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x+1,y),\
                      (x-1, y-1), (x+1, y-1)]]
        else:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y + 1),(x+1,y),\
                      (x-1, y+1), (x+1, y+1)]]
    elif x > 0:
        if y > 0 and y < len(grid[0]) - 1:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x,y + 1),\
                      (x-1, y-1), (x-1, y+1)]]
        elif y > 0:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x-1, y-1)]]
        else:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y), (x,y + 1), (x-1, y+1)]]
    else:
        if y > 0 and y < len(grid[0]) - 1:
            links = [grid[d[0]][d[1]] for d in\
                     [(x+1, y),(x,y - 1),(x,y + 1),\
                      (x+1, y-1), (x+1, y+1)]]
        elif y > 0:
            links = [grid[d[0]][d[1]] for d in\
                     [(x+1, y),(x,y - 1),(x+1, y-1)]]
        else:
            links = [grid[d[0]][d[1]] for d in\
                     [(x+1, y), (x,y + 1), (x+1, y+1)]]
    return [link for link in links if link.value != 9]

def CheckObst(nodo, grid):   
	x = nodo[0]   
	y = nodo[1]
	
	return (grid[int(x)][int(y)].value >= 5)
 
def lineofsight(parent, node, grid):
	x0 = parent.grid_point[0]
	y0 = parent.grid_point[1]
	x1 = node.grid_point[0]
	y1 = node.grid_point[1]
	dy = y1 - y0
	dx = x1 - x0
	f = 0
	
	if dy < 0:
		dy = -dy
		sy = -1
	else:
		sy = 1
		
	if dx < 0:
		dx = -dx
		sx = -1
	else:
		sx = 1
		
	if dx >= dy:
		while x0 != x1:
			f += dy
			if f >= dx:
				if CheckObst((x0 + ((sx - 1) / 2), y0 + ((sy - 1) / 2)), grid):
					return False
				y0 += sy
				f -= dx
			if f != 0 and CheckObst((x0 + ((sx - 1) / 2), y0 + ((sy - 1) / 2)), grid):
				return False
			if dy == 0 and CheckObst((x0 + ((sx - 1) / 2), y0), grid) and CheckObst((x0 + ((sx - 1) / 2), y0 - 1), grid):
				return False
			x0 += sx
	else:
		while y0 != y1:
			f += dx
			if f >= dy:
				if CheckObst((x0 + ((sx - 1) / 2), y0 + ((sy - 1) / 2)), grid):
					return False
				x0 += sx
				f -= dy
			if f != 0 and CheckObst((x0 + ((sx - 1) / 2), y0 + ((sy - 1) / 2)), grid):
				return False
			if dx == 0 and CheckObst((x0, y0 + ((sy - 1) / 2)), grid) and CheckObst((x0 - 1, y0 + ((sy - 1) / 2)), grid):
				return False
			y0 += sy
	
	return True
	
	
def search_theta(start, goal, grid, heur='naive'):
	#The open and closed sets
	openset = set()
	closedset = set()
	#Current point is the starting point
	current = start
	#Add the starting point to the open set
	openset.add(current)
    #While the open set is not empty
	while openset:
		#Find the item in the open set with the lowest G + H score
		current = min(openset, key=lambda o:o.G + o.H)
		pp.expanded_nodes += 1
		#If it is the item we want, retrace the path and return it
		if current == goal:
			path = []
			while current.parent:
				path.append(current)
				current = current.parent
			path.append(current)
			return path[::-1]
		#Remove the item from the open set
		openset.remove(current)
		#Add it to the closed set
		closedset.add(current)
		#Loop through the node's children/siblings
		for node in children(current,grid):
			#If it is already in the closed set, skip it
			if node in closedset:
				continue
			#Otherwise if it is already in the open set
			if node in openset:
				if current.parent != None and lineofsight(current.parent, node, grid):
					# Path 2
					new_g = current.parent.G + current.parent.move_cost(node)
					if node.G > new_g:
						node.G = new_g
						node.parent = current.parent
				else:
					# Path 1
					#Check if we beat the G score 
					new_g = current.G + current.move_cost(node)
					if node.G > new_g:
						#If so, update the node to have a new parent
						node.G = new_g
						node.parent = current
			else:
				#If it isn't in the open set, calculate the G and H score for the node
				node.G = current.G + current.move_cost(node)
				node.H = pp.heuristic[heur](node, goal)
				#Set the parent to our current item
				node.parent = current
				#Add it to the set
				openset.add(node)
	#Throw an exception if there is no path
	raise ValueError('No Path Found')

pp.register_search_method('Theta*', search_theta)
	