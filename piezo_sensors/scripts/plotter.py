import matplotlib.pyplot as plt
import numpy as np

# Load data
start_dir= '/home/lar03/ros/FedericoMucci/Tutorial/'                         #!Change this with the folder the workspace is in
copied_dir= 'Federico_ws/Dataset/Dataset_fatto/Dataset_T_5530_8000_7300_2'   #!Change with the path of the folder you want to plot
directory = start_dir + copied_dir +'/'
data_T = np.array([])
data_I = np.array([])
data_R = np.array([])
data_Set_T = np.array([])
data_Set_I = np.array([])
data_Set_R = np.array([])
data_Pos_T = np.array([])
data_Pos_I = np.array([])
data_Pos_R = np.array([])
data_velocity = np.array([])
case= [0,0,0]
for c in copied_dir:
    if c == 'T':
        case[0] = 1
    if c == 'I':
        case[1] = 1
    if c == 'R':
        case[2] = 1

subplots=1
if case[0]:
    data_T = np.load( directory + 'Thumb.npy')
    data_Set_T = np.load(directory + 'Set_T.npy')
    data_Pos_T = np.load(directory + 'Pos_T.npy')
    subplots += 2
if case[1]:
    data_I = np.load(directory + 'Index.npy')
    data_Set_I = np.load(directory + 'Set_I.npy')
    data_Pos_I = np.load(directory + 'Pos_I.npy')
    subplots += 2
if case[2]:
    data_R = np.load(directory + 'Ring.npy')
    data_Set_R = np.load(directory + 'Set_R.npy')
    data_Pos_R = np.load(directory + 'Pos_R.npy')
    subplots += 2
data_velocity = np.load(directory + 'Velocity.npy')

# Plot data

fig, axs = plt.subplots(subplots, 1, sharex=True, sharey=False)
fig.suptitle('Tactile sensors')
axs_i=0
if case[0]:
    axs[axs_i].plot(data_T)
    axs[axs_i].set_title('Thumb')
    axs[axs_i+1].plot(data_Set_T)
    axs[axs_i+1].set_title('Set_T/Pos_T')
    axs[axs_i+1].plot(data_Pos_T)
    axs_i += 2
if case[1]:
    axs[axs_i].plot(data_I)
    axs[axs_i].set_title('Index')
    axs[axs_i+1].plot(data_Set_I)
    axs[axs_i+1].set_title('Set_I/Pos_I')
    axs[axs_i+1].plot(data_Pos_I)
    axs_i += 2
if case[2]:
    axs[axs_i].plot(data_R)
    axs[axs_i].set_title('Ring')
    axs[axs_i+1].plot(data_Set_R)
    axs[axs_i+1].set_title('Set_R/Pos_R')
    axs[axs_i+1].plot(data_Pos_R)
    axs_i += 2
axs[axs_i].plot(data_velocity)
axs[axs_i].set_title('Velocity')

plt.show()

