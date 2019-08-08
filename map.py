# -*- coding: utf-8 -*-
"""
Created on Tue Oct 16 19:42:46 2018

@author: Shelton
"""

import numpy as np 
import cv2

from visual_odometry import PinholeCamera, VisualOdometry


cam = PinholeCamera(1241.0, 376.0, 718.8560, 718.8560, 607.1928, 185.2157)
#cam=PinholeCamera(640,480,538.8636705068433,538.8445314918096,318.83256039016896,242.66774610227353,k1=0.24478296725809076, k2=-0.5104803048920522, p1=-0.006448619715457021, p2=-0.002142112640096728, k3=0.0)

index=input("input data number")##input
print("data number:", index)##print
index=str(index)
#vo = VisualOdometry(cam, '/home/turtlebot/Downloads/dataset/sequences/00/02.txt')
vo = VisualOdometry(cam, '/home/turtlebot/Downloads/dataset/sequences/00/0'+index+'.txt')

traj = np.zeros((1000,1000,3), dtype=np.uint8)

for img_id in xrange(4541):
	#img = cv2.imread('/home/turtlebot/Downloads/dataset/sequences/02/image_0/'+str(img_id).zfill(6)+'.png', 0)
	img = cv2.imread('/home/turtlebot/Downloads/dataset/sequences/0'+index+'/image_0/'+str(img_id).zfill(6)+'.png', 0)
	#img=cv2.imread('/home/turtlebot/data/'+str(img_id)+'.png')
	#img=img[240:480,:]
	vo.update(img, img_id)

	cur_t = vo.cur_t
	if(img_id > 2):
		x, y, z = cur_t[0], cur_t[1], cur_t[2]
	else:
		x, y, z = 0., 0., 0.
	draw_x, draw_y = int(x)+500, int(z)+500
	true_x, true_y = int(vo.trueX)+500, int(vo.trueZ)+500

	#cv2.circle(traj, (draw_x,draw_y), 1, (img_id*255/4540,255-img_id*255/4540,0), 1)
	cv2.circle(traj, (draw_x,draw_y), 1, (0,255,0), 1)
	cv2.circle(traj, (true_x,true_y), 1, (0,0,255), 2)
	cv2.rectangle(traj, (10, 20), (500, 500), (0,0,0), -1)
	text = "Coordinates: x=%2fm y=%2fm z=%2fm"%(x,y,z)
	cv2.putText(traj, text, (20,40), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 1, 8)

	cv2.imshow('image', img)
	cv2.imshow('Trajectory', traj)
	cv2.waitKey(1)

cv2.imwrite('map.png', traj)
