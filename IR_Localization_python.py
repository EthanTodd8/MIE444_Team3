import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import convolve2d

m_r = np.array([1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1])
m_m = np.array(list('wwwdawdwwwwwwwwwwwwdwwwwa'))

#World Initialization
dim1 = 32
dim2 = 16
locationindex = np.arange(1, dim1 * dim2 + 1).reshape(dim2, dim1)
n = locationindex.size

np.random.seed(5489)
bw = np.random.randint(0, 2, (dim2, dim1))  # 0 = black, 1 = white

#Make blocks (walls)
M = np.zeros_like(bw)
Blocks = np.array([
    [2, 3],
    [3, 2],
    [4, 3],
    [5, 1],
    [5, 3],
    [7, 1],
    [7, 3],
    [7, 4]
])

for xx in range(Blocks.shape[0]):
    x, y = Blocks[xx]
    M[1 + (y - 1) * 4:(y - 1) * 4 + 4,
      1 + (x - 1) * 4:(x - 1) * 4 + 4] = 1
    
# Add border walls
M = np.pad(M, pad_width=1, mode='constant', constant_values=1)

# Create mask for blocks (invert walls)
M = np.abs(M - 1)
M = M[1:-1, 1:-1]
plt.figure()
plt.imshow((bw + 1) * M, cmap='gray')
plt.title('Initial Map')
plt.show(block=False)


heading = 0  # Initial heading

#Sense Function 
def sense_r(world, mask, p, SenVal):
    """
    Simulate sensor update:
    bw: black/white world
    M: mask (walls)
    p: current probability map
    measurement: 1 or 0 (sensor reading)
    """
    pHit = 0.6
    pMiss = 0.2

    # Default multiplier (pMiss everywhere)
    mult = np.full(world.shape, pMiss)
    mult[world == SenVal] = pHit # Where world matches measurement, set pHit
    pnew = p * mult # Multiply probabilities elementwise
    pnew *= mask # Apply mask (zero out walls)

    # Normalize
    total = np.sum(pnew)
    if total > 0:
        pnew /= total

    return pnew

#Move Function 
def move(p, mask, heading, Move):
    """
    Simulate motion update:
    p: current probability map
    M: mask (walls)
    heading: direction (0, 90, 180, 270)
    move_cmd: 'w', 'a', 'd', etc.
    """
    # Movement uncertainty kernel filter 
    K = np.array([[0.1, 0.1, 0.1], [0.1, 0.8, 0.1], [0.1, 0.1, 0.1]])
    
    #New belief after convolution
    pnew = convolve2d(p, K, mode='same', boundary='fill', fillvalue=0) 

    print(f"heading: {heading}")

    # Direction update
    if move == 'a':  # turn left
        heading = (heading + 90) % 360
    elif move == 'd':  # turn right
        heading = (heading - 90) % 360
    elif move == 's':  # reverse
        heading = (heading + 180) % 360

    # Determine movement direction (x = columns, y = rows)
    col_move = np.cos(np.deg2rad(heading))
    row_move = np.sin(np.deg2rad(heading))

    # Horizontal shift
    if col_move > 0:
        # move right: shift columns right, fill left with zeros
        pnew = np.hstack((np.zeros((pnew.shape[0], 1)), pnew[:, :-1]))
    elif col_move < 0:
        # move left: shift columns left, fill right with zeros
        pnew = np.hstack((pnew[:, 1:], np.zeros((pnew.shape[0], 1))))

    # Vertical shift
    if row_move < 0:
        # move up
        pnew = np.vstack((np.zeros((1, pnew.shape[1])), pnew[:-1, :]))
    elif row_move > 0:
        # move down
        pnew = np.vstack((pnew[1:, :], np.zeros((1, pnew.shape[1]))))
        
    pnew *= mask # Apply mask

    # Normalize
    total = np.sum(pnew)
    if total > 0:
        pnew /= total

    print(f"move: {Move}, heading: {heading}")
    return pnew, heading


# Initialize probability map
p = np.ones((dim2, dim1)) * (1 / n)
plt.figure()
plt.title('Localization Process')

# Localization loop
for k in range(len(m_m)):
    # Sensor update (placeholder)
    p = sense_r(bw, M, p, m_r[k])

    plt.clf()
    plt.imshow(p, cmap='hot', interpolation='nearest')
    plt.title(f'Step: {k + 1}')
    plt.pause(0.5)

    # Movement update (placeholder)
    p, heading = move(p, M, heading, m_m[k])

plt.show()