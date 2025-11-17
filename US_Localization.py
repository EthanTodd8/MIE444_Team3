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
    step_fraction = 0.4 #rover moves 5 inches = 0.4 of a cell (12.5 inches)
    
    if col_move > 0:
        # Move right
        shifted = np.hstack([np.zeros((pnew.shape[0], 1)), pnew[:, :-1]])
        pnew = (1 - step_fraction) * pnew + step_fraction * shifted
    elif col_move < 0:
        # Move left
        shifted = np.hstack([pnew[:, 1:], np.zeros((pnew.shape[0], 1))])
        pnew = (1 - step_fraction) * pnew + step_fraction * shifted

    # --- ROW (Y) MOVEMENT ---
    if row_move < 0:
        # Move up
        shifted = np.vstack([np.zeros((1, pnew.shape[1])), pnew[:-1, :]])
        pnew = (1 - step_fraction) * pnew + step_fraction * shifted
        
    elif row_move > 0:
        # Move down
        shifted = np.vstack([pnew[1:, :], np.zeros((1, pnew.shape[1]))])
        pnew = (1 - step_fraction) * pnew + step_fraction * shifted

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
heading = 90 # 270° implies downward

# Provide measurements and movements 
m_u = [84.33054, 8.63346, 9.6647, 55.2323, 85.58276000000001, 85.78596, 8.67664, 9.0551, 54.00802, 85.21446, 77.75194, 28.4607, 42.40276, 60.93206, 82.16392, 70.28434, 23.703280000000003, 65.56756, 67.47256, 67.35826, 63.46444, 22.89048, 72.28078000000001, 77.6732, 60.060840000000006, 58.05424000000001, 22.5171, 8.811259999999999, 82.26044, 56.88838, 52.58308, 9.2583, 8.7757, 91.25204000000001, 48.20412, 45.61586, 9.16686, 8.68934, 97.46996, 36.56584, 36.31184, 9.03478, 8.80618, 105.90276000000001, 27.25674, 25.19172, 9.32942, 8.32104, 112.4204, 23.34514, 22.872700000000002, 26.53284, 8.658859999999999, 116.74348, 14.7066, 11.69416, 37.4904, 9.05764, 138.29538000000002, 8.03148, 7.41934, 24.290020000000002, 11.440159999999999, 12.7635, 45.32376000000001, 36.449, 122.98172, 9.07034, 8.59282, 36.078160000000004, 36.235640000000004, 140.25626, 9.133840000000001, 15.87246, 30.29204, 27.10434, 20.025360000000003, 9.16432, 24.45258, 22.95652, 20.65528, 8.90016, 102.12832, 29.35732, 14.74724, 12.125960000000001, 8.93572, 101.44506, 39.71544, 7.71398, 7.84606, 9.39292, 17.277079999999998, 11.98372, 21.59, 104.5591, 8.71982, 37.44722, 9.21004, 98.80092, 95.29318, 8.6233, 39.116, 14.02334, 95.36684, 89.87536, 8.31088, 8.99668, 24.23668, 87.92464, 85.21954000000001, 9.30148, 8.636, 28.99156, 76.83754, 77.81544, 8.68934, 8.45312, 37.2491, 70.98537999999999, 70.19036, 9.08558, 8.60806, 47.142399999999995, 59.232800000000005, 58.882279999999994, 9.14654, 8.49122, 52.89042, 51.98872, 50.368199999999995, 8.83158, 36.6522, 62.933580000000006, 40.66794, 45.2247, 8.90778, 38.770559999999996, 69.49694000000001, 37.9476, 37.77488, 8.826500000000001, 39.36492, 72.42048, 30.73908, 26.89352, 8.54202, 38.488620000000004, 85.471, 22.009099999999997, 20.5232, 8.57758, 40.34536, 92.12072, 14.757399999999999, 11.940539999999999, 8.49376, 99.16159999999999, 98.13036, 7.47014, 7.881620000000001, 10.10158, 46.30166, 12.29868, 21.95068, 97.56648, 8.44042, 107.28198, 8.82142, 101.88194, 97.1931, 8.74268, 96.28632, 15.48638, 89.56801999999999, 85.4964, 8.85444, 39.878, 24.48306, 82.8167, 81.9785, 8.75538, 39.81196, 29.21762, 76.05776, 70.72122, 8.966199999999999, 41.275, 42.55008, 70.41134, 63.45682, 8.963659999999999, 36.64204, 46.21022000000001, 54.378859999999996, 60.51296000000001, 9.0932, 8.42264, 52.58562, 52.46116, 51.252120000000005, 8.864600000000001, 8.93318, 55.71744, 46.482, 41.376599999999996, 8.7376, 9.09066, 70.1675, 36.89604, 37.81298, 8.91286, 9.425939999999999, 76.90358, 31.02102, 26.339799999999997, 8.8773, 8.65124, 86.26602, 24.09952, 23.10384, 9.08812, 69.73062, 87.78240000000001, 14.693900000000001, 11.661140000000001, 9.02716, 131.73456, 104.24922, 8.064499999999999, 8.02132, 10.08126, 15.6591, 13.81252, 21.1455, 101.03358, 9.02208, 105.86211999999999, 9.2964, 139.05738, 144.76222, 9.30148, 100.20808, 15.05204, 144.11705999999998, 140.02258, 8.93318, 8.945879999999999, 24.00808, 144.7927, 123.55575999999999, 8.89254, 9.151620000000001, 29.67736, 137.38098000000002, 133.2484, 8.85444, 8.81634, 37.85362, 134.48538000000002, 127.50038, 8.91286, 9.573260000000001, 49.80178, 126.34468, 116.03482, 8.844280000000001, 8.47852, 52.204620000000006, 119.66194, 111.21898, 9.01192, 38.651180000000004, 61.90996, 100.21824000000001, 96.86544, 8.01878, 41.18356, 69.31406, 97.38868, 98.38436, 8.78332, 40.98290000000001, 76.073, 98.81362, 92.05467999999999, 8.25754, 8.636, 86.40826, 86.1441, 89.59342000000001, 8.5979, 8.66648, 94.26448, 74.7141, 72.75322, 8.64362, 9.23544, 93.34754, 72.77608000000001, 64.60236, 8.93826, 9.33196, 105.7275, 57.721500000000006, 57.22112, 9.0932, 8.48106, 118.8212, 55.102759999999996, 52.849779999999996, 9.288780000000001, 8.83666, 120.87352, 47.58182, 44.3992, 8.803640000000001, 8.7757, 137.2108, 38.61054, 36.64204, 8.76046, 8.93318, 141.68120000000002, 31.013400000000004, 27.787599999999998, 9.45388, 8.387080000000001, 130.57632, 21.8186, 20.957539999999998, 8.90016, 71.8185, 132.08254, 15.48892, 12.09294, 8.9281, 85.60054000000001, 134.5946, 7.404100000000001, 7.145020000000001, 9.418320000000001, 17.02816, 12.17168, 20.462239999999998, 83.43138, 8.74014, 135.36676, 9.0424, 87.07374, 76.98486, 8.90778, 97.50806000000001, 14.810740000000001, 70.31228, 74.05878, 9.20242, 9.42848, 25.326340000000002, 66.34226, 73.9775, 9.24306, 8.790939999999999, 31.371540000000003, 60.7695] #Ultrasonic measurements from Sensor 0,1,2,3,4. 
m_m = ['F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'R', 'R', 'F', 'F', 'F', 'F', 'L', 'L', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'L', 'L', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'L', 'L', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'L', 'L', 'F', 'F', 'F' ] #Movements
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

#plt.imshow(M, cmap='plasma', origin='upper')

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
threshold = 12.57  # in cm

# Correct ultra array shape (rows=dim2, cols=dim1)
ultra = np.zeros((dim2, dim1), dtype=int)  # new array with only zeros

for i in range(num_zones):
    u2, u3, u0, u1, u4 = sensor_data[i] #front sensor is u2, right is u3, back is u0, left is u1, front_bottom is u4

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

# Visualize Ultrasonic Map with Zones 
# plt.imshow(ultra, cmap='plasma', origin='upper')
# for i in range(ultra.shape[0]):
#     for j in range(ultra.shape[1]):
#         plt.text(j, i, f"{int(ultra[i, j])}", ha='center', va='center', color='white', fontsize=6)
# plt.title("Ultrasonic Map with Zone Labels")
# plt.show()

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

