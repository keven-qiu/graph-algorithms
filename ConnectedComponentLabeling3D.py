import numpy as np
"""
Algorithm for image processing"
"""

class Node:
  def __init__(self, value = None, nextPtr = None) -> None:
    self.value = value
    self.nextPtr = nextPtr

class Queue:
  def __init__(self) -> None:
    self.q = None

  def push(self, value):
    newNode = Node(value, self.q)
    self.q = newNode

  def pop(self):
    self.q = self.q.nextPtr

  def front(self):
    return self.q.value

  def empty(self):
    return self.q is None

def findConnectedComponents(image: np.array, label: int):
  size_x, size_y, size_z = image.shape

  def get6ConnectedNeighbours(x, y, z):
    neighbours = []
    if (x-1 >= 0):
      neighbours.append((x-1,y,z))
    if (x+1 <= size_x-1):
      neighbours.append((x+1,y,z))
    if (y-1 >= 0):
      neighbours.append((x,y-1,z))
    if (y+1 <= size_y-1):
      neighbours.append((x,y+1,z))
    if (z-1 >= 0):
      neighbours.append((x,y,z-1))
    if (z+1 <= size_z-1):
      neighbours.append((x,y,z+1))
    return neighbours

  q = Queue()
  visited = np.zeros_like(image, dtype=bool)

  currLabel = 1
  labelToIndices = {}

  for i in range(size_x):
    for j in range(size_y):
      for k in range(size_z):
        if (not visited[i,j,k]):
          if (image[i,j,k] == label):
            if (currLabel not in labelToIndices):
              labelToIndices[currLabel] = []

            labelToIndices[currLabel].append((i,j,k))
            visited[i,j,k] = True
            q.push((i,j,k))

            while (not q.empty()):
              index = q.front()
              q.pop()

              neighbours = get6ConnectedNeighbours(index[0], index[1], index[2])

              for n in neighbours:
                if (image[n[0],n[1],n[2]] == label and not visited[n[0],n[1],n[2]]):
                  labelToIndices[currLabel].append(n)
                  visited[n[0],n[1],n[2]] = True
                  q.push(n)

            currLabel += 1

  return labelToIndices
