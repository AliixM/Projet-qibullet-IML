#! /usr/bin/env python
# coding: utf-8

import cv2
import time
from qibullet import SimulationManager
from qibullet import PepperVirtual
import pybullet as p

def main() :
	
	# create simulator

	simulation_manager = SimulationManager()
	client = simulation_manager.launchSimulation(gui=True)

	# env
	
	p.connect(p.DIRECT)
	cube_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.125, 0.125, 0.125])
	cube_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.25, 0.25, 0.25])
	cube_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex = cube_collision, baseVisualShapeIndex = cube_visual, basePosition = [2, 1, 0.725])

	p.loadURDF("./chair/chair.urdf", basePosition = [4, 1, 0], globalScaling = 1)	

	# robot

	pepper = simulation_manager.spawnPepper(client, spawn_ground_plane = True)
	pepper.goToPosture("Crouch", 0.6)
	time.sleep(3)
	pepper.goToPosture("Stand", 0.6)
	time.sleep(3)
	pepper.goToPosture("StandZero", 0.6)
	time.sleep(5)
	pepper.moveTo(-3.0, 1.0, 0.0, frame = PepperVirtual.FRAME_WORLD, _async = True)
	time.sleep(5)

	# camera

	handle = pepper.subscribeCamera(PepperVirtual.ID_CAMERA_BOTTOM)
	handle2 = pepper.subscribeCamera(PepperVirtual.ID_CAMERA_TOP)
	#handle3 = pepper.subscribeCamera(PepperVirtual.ID_CAMERA_DEPTH)
	
	try :
		while True :
			
			img = pepper.getCameraFrame(handle)
			cv2.imshow("bottom camera", img)
	
			img2 = pepper.getCameraFrame(handle2)
			cv2.imshow("top camera", img2)

			#img3 = pepper.getCameraFrame(handle3)
			#cv2.imshow("depth camera", img3)

			#filename ="ImageDepth.png"
			#cv2.imwrite(filename, img3)
			#im_gray = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
			#im_color = cv2.applyColorMap(im_gray, cv2.COLORMAP_HSV)
			#cv2.imshow("colorBar depth camera", im_color)

			cv2.waitKey(1)

	except KeyboardInterrupt :
		simulation_manager.stopSimulation(client)


	# laser

	pepper.showLaser(True)
	pepper.subscribeLaser()

	while True:
		laser_list = pepper.getRightLaserValue()
		laser_list.extend(pepper.getFrontLaserValue())
		laser_list.extend(pepper.getLeftLaserValue())

		if all(laser == 5.6 for laser in laser_list):
			print ("Nothing detected")
		else :
			print("Detected")
			pass


if __name__ == "__main__":
	main()


