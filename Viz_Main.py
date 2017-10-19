#!/usr/bin/env python

import roslib; roslib.load_manifest('rviz_python_tutorial')
from std_msgs.msg import String

import sys
import rospy

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

import rviz


class RosViz_Dron( QWidget ):

    def __init__(self):
        QWidget.__init__(self)
	rospy.init_node("Vis")
	self.pub = rospy.Publisher("/Output",String,queue_size = 5)
        self.frame = rviz.VisualizationFrame()

        self.frame.setSplashPath( "" )

        self.frame.initialize()

        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile( config, "Visualization_points.rviz" )
        self.frame.load( config )

        self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )

        self.frame.setMenuBar( None )
        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( False )

        self.manager = self.frame.getManager()

        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )      

        layout = QVBoxLayout()
        layout.addWidget( self.frame )
	h_layout = QHBoxLayout()
        top_button = QPushButton( "Acept Movement? (Green Arrow)" )
        top_button.clicked.connect( self.onTopButtonClick )
        h_layout.addWidget( top_button )

	No_button = QPushButton( "Reject Movement?" )
        No_button.clicked.connect( self.NoButtonClick )
        h_layout.addWidget( No_button )
        layout.addLayout(h_layout)
        self.setLayout( layout )


    def NoButtonClick( self ):
        print "Peticion Rechazada"
	self.pub.publish(String("No"))	

    def onTopButtonClick( self ):
        print "Peticion Aceptada"
	self.pub.publish(String("Ok"))
  

if __name__ == '__main__':
    app = QApplication( sys.argv )

    RosVix = RosViz_Dron()

    RosVix.show()

    app.exec_()
