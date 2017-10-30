# Drone_Rviz_Visualization_Control
Proyect that launch a Rviz windows that shows the Drone parameters and allows to send directions from the visual information

To launch this you need to have a Dron running (Tested with Rotors simulation). In the Rviz file the topics that feed the visualizator are stored. It can be open with Rviz to change the topics. In the Config.py there is the different topics needed and parameters used.

For the Full system to work you need to launch (in different terminals):

	-Python script Viz_Dron.py is to launch the a window that works under Rviz that gives information of the Dron, the possible movements and the ability of decide if the movement is correct or not (as a fail-safe).

	-Python script Mov_Ev.py is to process the camera output and obtain the direction the User is pointing and feed the Drone with it.

	-Python script Moves.py is to create the different visuals helps for the first script.

You need to download the Caffe model for the CNN of the Skeleton[1]. We use the python version of their code with some minor changes to obtain the skeleton. You can download the model with the script in the model folder.

[1]Realtime Multi-Person Pose Estimation. By Zhe Cao, Tomas Simon, Shih-En Wei, Yaser Sheikh. https://github.com/ZheC/Realtime_Multi-Person_Pose_Estimation
