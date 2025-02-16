from heapq import heappush, heappop

def a_star_3d(start, goal, grid, resolution):
    neighbors = [(1,0,0), (-1,0,0), (0,1,0), (0,-1,0), (0,0,1), (0,0,-1)]
    
    open_heap = []
    heappush(open_heap, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    while open_heap:
        current = heappop(open_heap)[1]
        if current == goal:
            return reconstruct_path(came_from, current)
        
        for dx, dy, dz in neighbors:
            neighbor = (current[0]+dx, current[1]+dy, current[2]+dz)
            
            # 边界检查
            if not (0 <= neighbor[0] < grid.shape[0] and
                    0 <= neighbor[1] < grid.shape[1] and
                    0 <= neighbor[2] < grid.shape[2]):
                continue
                
            if grid[neighbor]:
                continue
                
            tentative_g = g_score[current] + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal)
                heappush(open_heap, (f, neighbor))
    
    return []

def heuristic(a, b):
    return ((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)**0.5

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]