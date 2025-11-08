import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import convolve2d

def sense_u(world, mask, p, SenVal):
    """
    Models sensor reading update for localization.
    
    Parameters:
        world : np.ndarray
            Ultrasonic map or sensor reading world.
        mask : np.ndarray
            Binary mask of valid regions (1 = free, 0 = blocked).
        p : np.ndarray
            Current probability distribution.
        SenVal : int or float
            Current sensor measurement value.
    Returns:
        pnew : np.ndarray
            Updated and normalized probability map.
    """
    # Probabilities for sensor accuracy
    pHit = 0.6
    pMiss = 0.2

    # Create multiplier matrix
    mult = np.full_like(world, pMiss, dtype=float)
    mult[world == SenVal] = pHit

    # Apply sensor update
    pnew = p * mult

    # Apply mask
    pnew = pnew * mask

    # Normalize (avoid divide-by-zero)
    total = np.sum(pnew)
    if total > 0:
        pnew /= total
    else:
        pnew = np.ones_like(pnew) / np.size(pnew)

    return pnew



def move(p, mask, heading, Move):
    """
    Models probabilistic movement and heading update.

    Parameters:
        p : np.ndarray
            Current probability map.
        mask : np.ndarray
            Binary mask for valid regions (1 = free, 0 = blocked).
        heading : float
            Current heading angle in degrees.
        Move : str
            Movement command: 'w', 'a', 's', 'd'.

    Returns:
        pnew : np.ndarray
            Updated probability map.
        heading : float
            Updated heading after rotation.
    """

    # Movement error kernel (same as MATLAB's K)
    K = np.array([
        [0.1, 0.1, 0.1],
        [0.1, 0.8, 0.1],
        [0.1, 0.1, 0.1]
    ])

    # 2D convolution with same size output
    pnew = convolve2d(p, K, mode='same', boundary='fill', fillvalue=0)

    print(f"heading: {heading}")

    # --- DIRECTION HANDLING ---
    if Move == 'a':
        heading = (heading + 90) % 360
    elif Move == 'd':
        heading = (heading - 90) % 360
    elif Move == 's':
        heading = (heading + 180) % 360
    # 'w' means forward → no change to heading

    # Determine movement direction
    col_move = np.cos(np.deg2rad(heading))
    row_move = np.sin(np.deg2rad(heading))

    # --- COLUMN (X) MOVEMENT ---
    if col_move > 0:
        # Move right
        pnew = np.hstack([np.zeros((pnew.shape[0], 1)), pnew[:, :-1]])
    elif col_move < 0:
        # Move left
        pnew = np.hstack([pnew[:, 1:], np.zeros((pnew.shape[0], 1))])

    # --- ROW (Y) MOVEMENT ---
    if row_move < 0:
        # Move up
        pnew = np.vstack([np.zeros((1, pnew.shape[1])), pnew[:-1, :]])
    elif row_move > 0:
        # Move down
        pnew = np.vstack([pnew[1:, :], np.zeros((1, pnew.shape[1]))])

    # Apply mask
    pnew = pnew * mask

    # Normalize
    total = np.sum(pnew)
    if total > 0:
        pnew /= total
    else:
        pnew = np.ones_like(pnew) / np.size(pnew)

    print(f"move: {Move} heading: {heading}")

    return pnew, heading

# --- INITIAL SETUP ---

# Heading (degrees) //get from gyroscope
heading = 270  # 270° implies downward

# Provide measurements and movements 
m_u = [3, 3, 3, 1, 1, 1, 1, 5, 5, 5, 5, 0, 0, 0, 0, 5, 5, 5, 5, 2, 2, 2, 2, 2, 2] #Ultrasonic measurements from Sensor 0
m_m = ['w', 'w', 'w', 'd', 'a', 'd', 'w', 'w', 'w', 'w', 'w', 'w', 'w', 'w', 'w','w', 'w', 'w', 'w', 'w', 'd', 'w', 'w', 'w', 'a'] #Movements

# Initialization of the world
dim1, dim2 = 32, 16
locationindex = np.arange(1, dim1 * dim2 + 1).reshape(dim2, dim1)
n = locationindex.size

np.random.seed(5489)
bw = np.random.randint(0, 2, size=(dim2, dim1))  # 0 = black, 1 = white

# Make blocks
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
    M[(y - 1) * 4: (y - 1) * 4 + 4, (x - 1) * 4: (x - 1) * 4 + 4] = 1

# Add walls around M
M = np.pad(M, pad_width=1, mode='constant', constant_values=1)

# --- GENERATE ULTRASONIC WORLD ---

ultra = np.zeros_like(bw)
for sec_row in range(0, dim2, 4):
    for sec_col in range(0, dim1, 4):
        segRow = M[sec_row + 2, sec_col:sec_col + 6]
        segCol = M[sec_row:sec_row + 6, sec_col + 2]
        val = np.sum(segRow) + np.sum(segCol)
        if val == 2 and np.sum(segRow) != 1:
            val = 5
        ultra[sec_row:sec_row + 4, sec_col:sec_col + 4] = val

# --- CREATE MASK FOR BLOCKS ---
M = np.abs(M - 1)
M = M[1:-1, 1:-1]

plt.figure()
plt.imshow((bw + 1) * M, cmap='gray')
plt.title("Initial World")
plt.show(block=False)

# --- INITIALIZE PROBABILITY ---
p = np.ones((dim2, dim1)) * (1 / n)

# --- LOCALIZATION LOOP ---

for k in range(len(m_m)):
    # Sensor update
    p = sense_u(ultra, M, p, m_u[k])  # You'll define this later

    plt.clf()
    plt.imshow(p, cmap='hot', interpolation='nearest')
    plt.title(f"Step: {k+1}")
    plt.pause(0.5)

    # Movement update
    p, heading = move(p, M, heading, m_m[k])  # You'll define this later

plt.show()

