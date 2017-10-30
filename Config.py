

#######Angles for the differents sectors: Right[0-1],Up-Right[1-2],Up[2-3],Up-left[3-4],Down-Right[0-5],Down[5-6],Down-Left[6-7], Left[Else]
Angles_values=[-20.0, 30.0,60.0,110.0,150.0,-70.0,-120.0,-160.0]

#######Frame_id for movement
#Frame_id_mov = 'odom'
Frame_id_mov = 'world'

#######Frame_id for the marker
#Frame_id = "vicon" #For Vicon Room
Frame_id_world = "world"  #For Stereo Odometry and simulation

#######Topic for the command (PoseStamped type)
#Topic_command="/pegasus/command/pose" #Pegasus Drone
Topic_command="/firefly/command/pose" #Firefly(Simulation) Drone

#######Topic for the Odometry (Odometry msgs)
#Topic_Odometry="/pegasus/vrpn_client/estimated_odometry" #Vicon Room
#Topic_Odometry="/stereo_odometer/odometry" #Stereo Odometry
Topic_Odometry="/firefly/odometry_sensor1/odometry" #Simulation

#######Limits for the movement of the drone 
x_min = -99.9 # 0.5 for the Vicon room
y_min = -99.9 # -2.0 for the Vicon room
z_min = 0.0 
x_max = 99.9 # 2.5 for the Vicon room
y_max = 99.9 # 0.0 for the Vicon room
z_max = 99.9

#######Number of frames skipped:
Skip_f = 3

#######Scales used for the skeleton
scale=[0.7]

#######Scale for resizing before
Res_scale = 0.8

#######Topic for the frontal camera (compressed)
#Topic_camera = "/camera/color/image_raw/compressed" #RealSense
Topic_camera = "/firefly/camera/camera_frontal/image_raw/compressed" #Simulation

#######Number of images to process
N_Images= 2

#######Threshold to accept the position
x_thres = 0.1
y_thres = 0.1
z_thres = 0.1

####Orientation Vector Transformation for Odometry
#orientation_list2 = [0,0,1.0,0] # Pegasus
orientation_list2 = [0,0,0,1.0] # Simulation
#######Rviz file
#Rviz_File = "Visual_Real.rviz"
Rviz_File = "Visual_Sim.rviz"

#######Function for transforming the output of the Visual Recogn to the Drone
def Transform_Dir(x,y,z):
	y=y*-1  #Simulation
	return z,y,x




