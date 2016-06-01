import heapq

class PriorityQueue:
    def __init__(self):
        self.elements = []
        
    # insert a new item with a given priority
    def insert(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
        
    # change the priority of a item
    def changePriority(self, item, priority):
        return heapq.heapreplace(self.elements, (priority, item))
    
    # check if queue is empty
    def empty(self):
        return len(self.elements) == 0
        
    # Get and remove the item with the minimum priority
    def delMin(self):
        return heapq.heappop(self.elements)[1]