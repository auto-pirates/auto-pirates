#!/usr/bin/env python

# Import the required packages
import sys # Command Line Arguments
import cv2  # OpenCV bindings
import rospy # ROS Python bindings
from scipy.stats import binom
from sensor_msgs.msg import Image # Image Sensor msg 
from cv_bridge import CvBridge, CvBridgeError # cv_bridge ros_image <-> opencv_image
from std_msgs.msg import Float64, Float32
import math
from std_msgs.msg import UInt8MultiArray
import numpy as np
from scipy.interpolate import interp1d
from scipy.ndimage import map_coordinates
from span_pose.msg import span_pose

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

'''
`render_map` is class to render radar data from the radar
on the map.
'''

def conv_angle(f):
    if f < 0:
        return f + (2*np.pi)
    else:
        return f

conv_angle = np.vectorize(conv_angle)

class render_map:
	def __init__(self):
		self.image_pub = rospy.Publisher("radar_2_map_im", Image, queue_size=1)
		self.image_sub = rospy.Subscriber("raw_radar_image", UInt8MultiArray, self.callback, queue_size=1)
		self.pose_sub = rospy.Subscriber("recboat/span_pose", span_pose, self.callback_pose, queue_size=1)
		self.im_map = cv2.imread("map_bdg.bmp",0)
		self.im_map_nobdg = cv2.imread("map_nobdg.bmp",0)
		self.lat = 0
		self.lon = 0
		self.yaw = 0
		self.north = 0
		self.east = 0

		self.radardata = np.array([0])

		##### VISUALIZATION #####
		self.viz_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)

		marker = Marker()
		marker.header.frame_id = "/map"
		marker.header.stamp = rospy.Time.now()
		marker.type = marker.POINTS
		marker.action = marker.ADD
		marker.scale.x = 1.0
		marker.scale.y = 1.0
		marker.scale.z = 0.0

		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.g = 0.0
		marker.color.b = 0.0

		marker.pose.orientation.w = 1.0
		marker.pose.position.x = 0.0
		marker.pose.position.y = 0.0
		marker.pose.position.z = 0.0
		marker.lifetime = rospy.Duration()

		self.marker = marker
		##############################

		## Marker 2
		self.viz_pub2 = rospy.Publisher('visualization_marker2', Marker, queue_size=1)
		
		marker2 = Marker()
		marker2.header.frame_id = "/map"
		marker2.header.stamp = rospy.Time.now()
		marker2.type = marker.POINTS
		marker2.action = marker.ADD
		marker2.scale.x = 1.0
		marker2.scale.y = 1.0
		marker2.scale.z = 0.0

		marker2.color.a = 1.0
		marker2.color.r = 0.0
		marker2.color.g = 0.0
		marker2.color.b = 1.0

		marker2.pose.orientation.w = 1.0
		marker2.pose.position.x = 0.0
		marker2.pose.position.y = 0.0
		marker2.pose.position.z = 0.0
		marker2.lifetime = rospy.Duration()

		self.marker2 = marker2

		###############################

		## Marker 3
		self.viz_pub3 = rospy.Publisher('visualization_marker3', Marker, queue_size=1)
		
		marker3 = Marker()
		marker3.header.frame_id = "/map"
		marker3.header.stamp = rospy.Time.now()
		marker3.type = marker.POINTS
		marker3.action = marker.ADD
		marker3.scale.x = 1.0
		marker3.scale.y = 1.0
		marker3.scale.z = 0.0

		marker3.color.a = 1.0
		marker3.color.r = 0.0
		marker3.color.g = 1.0
		marker3.color.b = 0.0

		marker3.pose.orientation.w = 1.0
		marker3.pose.position.x = 0.0
		marker3.pose.position.y = 0.0
		marker3.pose.position.z = 0.0
		marker3.lifetime = rospy.Duration()

		self.marker3 = marker3

		###############################

		#Bounding Boxes
		
		self.bbox_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)

		marker4 = Marker()
		marker4.header.frame_id = "/map"
		marker4.header.stamp = rospy.Time.now()
		marker4.type = marker.LINE_STRIP
		marker4.action = marker.ADD

		marker4.scale.x = 1.0
		

		marker4.color.a = 1.0
		marker4.color.r = 1.0
		marker4.color.g = 0.0
		marker4.color.b = 0.0

		marker.pose.orientation.w = 1.0

		marker.lifetime = rospy.Duration()

		self.marker4 = marker4
		
		###############################

	def callback_pose(self, data):
		self.lat = data.lat
		self.lon = data.lon
		self.yaw = data.yaw
		self.north = data.north
		self.east = data.east

	def callback(self, data):

		new_data = map(ord, data.data)

		if len(new_data) != 1048576:
		    return

		z = np.array(new_data, dtype=float).reshape(2048, 512)

		deg_per_spoke = float(360) / 2048
		#adjust for heading

		yaw_deg = self.yaw * 180 / np.pi

		num_roll = int(round( (yaw_deg / deg_per_spoke) ))
		z = np.roll(z, num_roll, axis=0)

		#set last column to 0
		z[:, -1] = 0

		self.radardata = z

		
def main(args):

	'''
	new_radardata = ic.radardata.deepcopy()

	spoke_n = 16
	occupied_p = 0.9
	P_thresh  = 0.5
	raw_thresh = 0.1

	for i in range(2048):
		for j in range(512):
			for l in [-spoke_n/2, spoke_n/2]:
				if ic.radardata[i, j+l] > raw_thresh:
					m = m + 1
			
				P = binom.cdf(m, spoke_n, occupied_p)

			if P > P_thresh:
				new_radardata(i,j) = 255
			else: 
				new_radardata(i,j) = 0


	'''

	br = CvBridge()
	# Initialize the node
	rospy.init_node('radar_2_map', anonymous="True")

	ic = render_map()


	radar_range = 125.

	#define polar grid
	nr = 512    
	nt = 2048

	r = np.linspace(0., radar_range, nr)
	t = np.linspace(0., 2*np.pi, nt)


	### TODO: figure out correct scale
	# Define new cartesian grid
	nx = 180
	ny = 180
	# Setting resolution of radar iamge

	#1 meter resolution
	x = np.linspace(-125, 125, nx)
	y = np.linspace(-125, 125, ny)


	#map our cartesian grid to r, theta values
	X, Y = np.meshgrid(x, y)

	cart_r = np.sqrt(X*X+Y*Y)
	cart_t = np.arctan2(Y, X)

	cart_t = conv_angle(cart_t)  #convert angles to [0, 2pi]



	#interpolate our converted cartesian grid into indices of the radar data matrix
	ir = interp1d(r, np.arange(len(r)), bounds_error=False)
	it = interp1d(t, np.arange(len(t)))

	new_ir = ir(cart_r.ravel())
	new_it = it(cart_t.ravel())
	coords = np.array([new_it, new_ir])

	while not rospy.is_shutdown():
		if ic.radardata.shape == (2048,512):

			img = map_coordinates(ic.radardata, coords, order=0).reshape(cart_r.shape)

			img_new = img.copy()
			



			#world file/map constants
			ori_x = 582573.204801
			ori_y = 4479935.416819
			grid_size = 4.992175
			map_pix_height = 881


			boat_x = (ic.east - ori_x)/grid_size
			boat_y = map_pix_height + (ic.north - ori_y)/grid_size

			#im_map = ic.im_map_const.copy()

			#print im_map.shape
			#print img.shape
			
			'''
			for i in range(-100, 100):
				for j in range(-100, 100):
					if img[100+j, 100+i] > 0:
						im_map[p_j+j, p_i+i, 1:3] = 0
						im_map[p_j+j, p_i+i, 0] = 255

			'''
			
			ic.marker.points = []
			ic.marker2.points = []
			ic.marker3.points = []
			ic.marker4.points = []

			
			#remove radar data beyond the river bank
			
			for i in range(-nx/2, nx/2):
				for j in range(-nx/2, nx/2):
					#print ic.im_map[boat_x+j, boat_y+i, 1]
					#overlap
					if (img_new[nx/2+j, nx/2+i] > 0) and (ic.im_map_nobdg[boat_x+j, boat_y+i] != 0):
					#if (ic.im_map[boat_x+j, boat_y+i] == 0):
						img_new[nx/2+j, nx/2+i] = 0;
						p3 = Point() 
						p3.x = boat_x + j
						p3.y = boat_y + i
						p3.z = 0.0
						ic.marker3.points.append(p3)
			
			

			'''
						
						p1 = Point() 
						p1.x = boat_x + j
						p1.y = boat_y + i
						p1.z = 0.0
						ic.marker.points.append(p1)
						
					#no overlap
					
					
					elif (img_new[nx/2+j, nx/2+i] > 0) and (ic.im_map[boat_x+j, boat_y+i] != 0) :
						p2 = Point() 
						p2.x = boat_x + j
						p2.y = boat_y + i
						p2.z = 0.0
						ic.marker2.points.append(p2)
					
					
					
					if img[nx/2+j, nx/2+i] > 0:
						p2 = Point() 
						p2.x = boat_x + j
						p2.y = boat_y + i
						p2.z = 0.0
						ic.marker2.points.append(p2)

					
			'''
			# change new radar image to uint8		
			img_new = img_new.astype(np.uint8)
			center = (90,90)
			M = cv2.getRotationMatrix2D(center, -math.degrees(-90), 1.0)
			im_new = cv2.warpAffine(img_new, M, (180, 180))


			# get the map for only the bridges
			im_bridges = ic.im_map - ic.im_map_nobdg 
			im_b = im_bridges.copy()
			#cv2.imshow("Image", im_bridges)

			# find contours: contours for new radar image; ctr_bridges for bridge map
			contours, hierarchy = cv2.findContours(img_new.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
			ctr_bridges , _ = cv2.findContours(im_b,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)


			#print ctr_bridges
			#print ctr_bridges[0][0]
			
			#cv2.imshow("Image no brigde", ic.im_map_nobdg)
			#cv2.imshow("Image brigde", ic.im_map)
			#cv2.imshow("Image", im_bridges)
			#cv2.waitKey(0)

			# filter bridge contours and find the centroids: ctr_bridges-->new_ctrs
			# Threshold = 50
			centroids = []
			new_ctrs = []
			areas = []
			for cntr in ctr_bridges:
				#print cntr
				M = cv2.moments(cntr)
				if cv2.contourArea(cntr) > 20:
					new_ctrs.append(cntr)
					centroids.append([int(M['m10']/M['m00']), int(M['m01']/M['m00'])])
			
			# Get the radar contours
			# Threshold = 50
			centroids_rd = []
			new_ctrs_rd = []
			areas_rd = []
			for cntr_rd in contours:
				M = cv2.moments(cntr_rd)
				if cv2.contourArea(cntr_rd) > 20:
					areas_rd.append(cv2.contourArea(cntr_rd))
					centroids_rd.append([int(M['m10']/M['m00']), int(M['m01']/M['m00'])])
					new_ctrs_rd.append(cntr_rd)
			

			to_del = []
			cunt = []
			for index, cen in enumerate(centroids_rd):
				cent = [cen[0] - 90 + boat_x, cen[1] - 90 + boat_y]
				for cen_2 in centroids:
					#print cent, cen_2, np.linalg.norm(np.array(cen)-np.array(cen_2))
					if np.linalg.norm(np.array(cent)-np.array(cen_2)) < 800:
						to_del.append(index)
						break
						#cunt.append()

			print to_del

			cunt_cont = []

			for i in range(len(centroids_rd)):
				if i in to_del:
					cunt.append(centroids_rd[i])
					cunt_cont.append(new_ctrs_rd[i])

			#print cunt

			'''
			for index in range(len(new_ctrs_rd)):
				for index_2 in range(len(new_ctrs_rd)):
					print new_ctrs_rd[index][index_2]
					new_ctrs_rd[index][index_2] = [[new_ctrs_rd[index][index_2][0][0]+boat_x, new_ctrs_rd[index][index_2][0][1]+boat_y]]
			'''
			'''
			cv2.drawContours(img_new, new_ctrs_rd, -1, (127,0,0), 2)
			cv2.imshow("Image", img_new)
			for dick in new_ctrs_rd:
				cv2.fillPoly(img_new, pts=dick, color=(127,0,0))
			cv2.imshow("Image 2", img_new)
			
			cv2.waitKey(20)
			'''

			for i in range(-nx/2, nx/2):
				for j in range(-nx/2, nx/2):
					flag = 2;
					for bikz in new_ctrs_rd:
						dist = cv2.pointPolygonTest(bikz, (nx/2+i, nx/2+j), False)	
						if  dist == 1 or dist == 0:
							flag = 1;

					if flag == 1:
					# bridges
						p1 = Point() 
						p1.x = boat_x + j
						p1.y = boat_y + i 
						p1.z = 0.0
						ic.marker.points.append(p1)
					
					if flag == 2 and img_new[nx/2+j,nx/2+i]>0:
					# obstacles
						p2 = Point() 
						p2.x = boat_x + j
						p2.y = boat_y + i
						p2.z = 0.0
						ic.marker2.points.append(p2)
						
						
						


			ic.viz_pub.publish(ic.marker)

			ic.viz_pub2.publish(ic.marker2)
			
			ic.viz_pub3.publish(ic.marker3)

			#cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
			#cv2.imshow("image", img)
	        	#cv2.waitKey(1)
	        

			#ic.image_pub.publish(br.cv2_to_imgmsg(im_map, "bgr8"))
			### Morphological 
			


if __name__ == "__main__":

	main(sys.argv)
