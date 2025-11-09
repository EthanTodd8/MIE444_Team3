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
    pHit = 0.7
    pMiss = 0.2

    # Create multiplier matrix
    mult = np.full_like(world, pMiss, dtype=float) #full like creates a new array with the same shape and type as a given array, filled with a specified value
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
            Movement command: 'F', 'L', 'B', 'R'.

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
    if Move == 'L':
        heading = (heading + 90) % 360
    elif Move == 'R':
        heading = (heading - 90) % 360
    elif Move == 'B':
        heading = (heading + 180) % 360
    # 'F' means forward → no change to heading

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
heading = 0 # 270° implies downward

# Provide measurements and movements 
m_u = [3, 3, 3, 1, 1, 1, 1, 5, 5, 5, 5, 0, 0, 0, 0, 5, 5, 5, 5, 2, 2, 2, 2, 2, 2] #Ultrasonic measurements from Sensor 0,1,2,3,4. 
m_m = ['F', 'F', 'F', 'R', 'L'] #Movements
#m_m = ['F', 'F', 'F', 'R', 'L', 'R', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F','F', 'F', 'F', 'F', 'F', 'R', 'F', 'F', 'F', 'L'] #Movements

# Initialization of the world
dim1, dim2 = 32, 16
locationindex = np.arange(1, dim1 * dim2 + 1).reshape(dim2, dim1)
n = locationindex.size

#np.random.seed(5489)
#bw = np.random.randint(0, 2, size=(dim2, dim1))  # 0 = black, 1 = white #bw map

# Make blocks
M = np.zeros((dim2, dim1), dtype=int)
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

for xx in range(Blocks.shape[0]): #dimensions for the blocks in the array
    x, y = Blocks[xx]
    M[(y - 1) * 4: (y - 1) * 4 + 4, (x - 1) * 4: (x - 1) * 4 + 4] = 1 

# Add walls around M
M = np.pad(M, pad_width=1, mode='constant', constant_values=1)

# --- GENERATE ULTRASONIC WORLD ---
# Prepare sensor data into shape (num_zones, 5):
m_u_arr = np.array(m_u)
usable_length = (m_u_arr.size // 5) * 5
m_u_arr = m_u_arr[:usable_length]

# number of 4x4 zones
num_zones = m_u_arr.size//5
zones_x = dim1 // 4
zones_y = dim2 // 4

#Sensor data for each block
sensor_data = m_u_arr.reshape(num_zones, 5)

# Threshold for classifying ultrasonic readings (adjust as needed)
threshold = 5

# Correct ultra array shape (rows=dim2, cols=dim1)
ultra = np.zeros((dim2, dim1), dtype=int)  # new array with only zeros

for i in range(num_zones):
    u0, u1, u2, u3, u4 = sensor_data[i]

    # Classify into zones
    if all(u > threshold for u in [u0, u1, u2, u3]):
        val = 0
    elif any(u < threshold for u in [u0, u1, u2, u3]):
        val = 1
    elif ((u0 < threshold and u1 < threshold) or
          (u1 < threshold and u2 < threshold) or
          (u2 < threshold and u3 < threshold) or
          (u3 < threshold and u0 < threshold)):
        val = 2
    elif sum(1 for u in [u0, u1, u2, u3] if u > threshold) == 1:
        val = 3
    elif ((u0 < threshold and u2 < threshold) or
          (u1 < threshold and u3 < threshold)):
        val = 5
    else:
        val = 0  # fallback (should rarely happen)

    # --- Fill the 4x4 region with this value ---
    zone_row = (i // zones_x) * 4
    zone_col = (i % zones_x) * 4
    ultra[zone_row:zone_row+4, zone_col:zone_col+4] = val

# --- CREATE MASK FOR BLOCKS ---
M = np.abs(M - 1)
M = M[1:-1, 1:-1]

#Visualize Ultrasonic Map with Zones 
plt.imshow(ultra, cmap='plasma', origin='upper')
for i in range(ultra.shape[0]):
    for j in range(ultra.shape[1]):
        plt.text(j, i, f"{int(ultra[i, j])}", ha='center', va='center', color='white', fontsize=6)
plt.title("Ultrasonic Map with Zone Labels")
plt.show()

# plt.figure()
# plt.imshow((bw + 1) * M, cmap='gray')
# plt.title("Initial World")
# plt.show(block=False)

# --- INITIALIZE PROBABILITY ---
p = np.ones((dim2, dim1)) * (1 / n)

# --- LOCALIZATION LOOP ---
for k in range(len(m_m)):
    # Sensor update
    p = sense_u(ultra, M, p, m_u[k])  

    # HeatMap Visualization
    plt.clf()
    plt.imshow(p, cmap='hot', interpolation='nearest')
    plt.title(f"Step: {k+1}")
    plt.pause(0.5)

    # Movement update
    p, heading = move(p, M, heading, m_m[k])  

plt.show()

