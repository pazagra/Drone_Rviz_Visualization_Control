# Drone_Rviz_Visualization_Control
Proyect that launch a Rviz windows that shows the Drone parameters and allows to send directions from the visual information

To launch this you need to have a Dron running (Tested with Rotors simulation). In the Rviz file the topics that feed the visualizator are stored. It can be open with Rviz to change the topics. It uses one VI_sensor in the back of the Drone for odometry and one RGB camera in the front to obtain the visual information.

Python script Mov_Base.py is to process the camera output and obtain the direction the User is pointing and feed the Drone with it.

Python script Viz_Main.py is to launch the a window that works under Rviz that gives information on the Dron, the possible movements and the ability of decide if the movement is correct or not.

You need to download the Caffe model for the CNN of the Skeleton[1]. We use the python version of their code with some minor changes to obtain the skeleton. The command is:

wget -nc --directory-prefix=Model/ http://posefs1.perception.cs.cmu.edu/Users/ZheCao/pose_iter_440000.caffemodel

[1]Realtime Multi-Person Pose Estimation. By Zhe Cao, Tomas Simon, Shih-En Wei, Yaser Sheikh. https://github.com/ZheC/Realtime_Multi-Person_Pose_Estimation
