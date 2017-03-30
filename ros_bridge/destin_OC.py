#
# destin_OC.py - Integration of DeSTIN with OpenCog.
# Copyright (C) 2016  Hanson Robotics
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
# 02110-1301  USA

import rospy
from atomic_msgs import AtomicMsgs
import ast
import time
from tensorflow_node.msg import TFNodeState, TFStatus

class DestinOC:
	


	def destin_cb(self,data, topic):
		#print "received " + data.id + " from " + topic			
		self.atomo.destin_status(data.state,topic)
			
			
			
	def __init__(self):
		self.atomo = AtomicMsgs()
		
	
		rospy.init_node('subscriber', anonymous=True)
		#print "hello"
		root_topic = rospy.get_param("/tensorflow_node/publishing/topic")
		node_topics = rospy.get_published_topics(root_topic)
		
		for topic,_ in node_topics:
			#print topic
			rospy.Subscriber(topic, TFNodeState, callback=self.destin_cb, callback_args=topic)
			
				
		rospy.spin()	
		
		
		
if __name__ == '__main__':
    c = DestinOC()
    #c.subscriber


