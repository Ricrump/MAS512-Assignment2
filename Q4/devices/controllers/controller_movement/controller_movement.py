from controller import Supervisor, Motor, GPS, Lidar, Camera, Accelerometer, Gyro
import math
import csv

robot = Supervisor()

timestep = int(robot.getBasicTimeStep())

#default name. Could see in webots documentation: "https://www.cyberbotics.com/doc/guide/pioneer-3at?version=R2019b-rev1"
f_left_motor = robot.getDevice('front left wheel') 
f_right_motor = robot.getDevice('front right wheel')
b_left_motor = robot.getDevice('back left wheel')
b_right_motor = robot.getDevice('back right wheel')


f_left_motor.setPosition(float('inf'))
f_right_motor.setPosition(float('inf'))
b_left_motor.setPosition(float('inf'))
b_right_motor.setPosition(float('inf'))

f_left_motor.setVelocity(0.0)
f_right_motor.setVelocity(0.0)
b_left_motor.setVelocity(0.0)
b_right_motor.setVelocity(0.0)


#Initialization and enabling of required sensors
gps = robot.getDevice('gps')
if gps:
    gps.enable(timestep)
    
lidar = robot.getDevice('lidar')
if lidar:
    lidar.enable(timestep)
    lidar.enablePointCloud()

camera = robot.getDevice('camera')
if camera:
    camera.enable(timestep)

accelerometer = robot.getDevice('accelerometer')
if accelerometer:
    accelerometer.enable(timestep)

gyro = robot.getDevice('gyro')
if gyro:
    gyro.enable(timestep)

#Obtain pos of the predefined objective
target_node = robot.getFromDef("Safe")
if target_node:
    target_pos = target_node.getPosition()
    target_x = target_pos[0]
    target_z = target_pos[2]
    
r = 0.11 #Official documentation in webots sais that
prev_x, prev_z = 0.0, 0.0
if gps:
    init_pos = gps.getValues()
    prev_x = init_pos[0]
    prev_z = init_pos[2]
    
target_reached = 0.5 

time_history = []
slip_history = []
v_enc_history = []
v_gt_history = []

time_counter = 0.0
csv_filename = 'data_Friction.csv'

while robot.step(timestep) != -1:
    vx, vz = 0.0, 0.0
    x, z = 0.0, 0.0
    
    if gps:
        current_pos = gps.getValues()
        x = current_pos[0]
        z = current_pos[2]
        
        dt = timestep / 1000.0
        vx = (x - prev_x) / dt
        vz = (z - prev_z) / dt
        prev_x = x
        prev_z = z
    
    #Check if the objective was achieved
    distance_to_target = math.sqrt((target_x - x) ** 2 + (target_z - z) ** 2)
    if distance_to_target < target_reached:
        print("¡Object reached!")
        f_left_motor.setVelocity(0.0)
        f_right_motor.setVelocity(0.0)
        b_left_motor.setVelocity(0.0)
        b_right_motor.setVelocity(0.0)
        break
    
    #Collision avoidance
    avoid = False
    if lidar:
        range_image = lidar.getRangeImage()
        width = lidar.getHorizontalResolution()
        
        center_start = int(width * 0.3)
        center_end = int(width * 0.7)
        
        for i in range(center_start, center_end):
            distance = range_image[i]
            if distance < 1.2:
                avoid = True
                break
    if avoid:
        f_left_motor.setVelocity(-1.5)
        f_right_motor.setVelocity(1.5)
        b_left_motor.setVelocity(-1.5)
        b_right_motor.setVelocity(1.5)
    else:
        f_left_motor.setVelocity(6.4)
        f_right_motor.setVelocity(6.4)
        b_left_motor.setVelocity(6.4)
        b_right_motor.setVelocity(6.4)
    
    #Velocity obtained from encoder
    w_f = f_left_motor.getVelocity()
    w_b = b_left_motor.getVelocity()
    w = (w_f + w_b) / 2.0
    
    v_enc = w * r
    
    #Compute the longitudinal slip
    s_long = 0.0
    vtotal = math.sqrt(vx**2 + vz**2)
    if abs(vx) > 0.001:  #Avoid dividing by 0
        s_long = (v_enc - vtotal) / vtotal
    
    time_history.append(time_counter)
    slip_history.append(s_long)
    v_enc_history.append(v_enc)
    v_gt_history.append(vtotal)
        
    print(f"v_enc: {v_enc:.3f} | vx (Ground Truth): {vx:.3f} | s_long: {s_long:.4f}")

#save csv file
with open(csv_filename, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['Time', 's_long', 'v_enc', 'v_gt'])
    for t, s, ve, vg in zip(time_history, slip_history, v_enc_history, v_gt_history):
        writer.writerow([t, s, ve, vg])

print(f"Exported data correctly!")