from collections import deque

import imutils
import numpy as np
import cv2
import math

from time import time

# Min: 1
# X max: 598
# Y max: 335
# Smallest radius: 2.8285

# Initialize variables
timePrev = 0
got_init = False
found_target = False
a_z = 0.0143589
v_xy = 0.0
v_z = 0.0
h0 = 0.0
s = math.inf
x_start = 0
y_start = 0
target = (0,0)

# Stepper motors
X_max = 1450
Y_max = 1600
min = 0

# Target zone boundaries
rect_tl = (276, 24)
rect_br = (343, 88)

# HSV boundaries
orangeLower = (0, 132, 96)
orangeUpper = (15, 255, 255)

# Define lists for data
buffer = 50
coords = deque(maxlen=buffer)
radii = deque(maxlen=10)
target_coords = deque(maxlen=rect_br[0])

# Kalman Filter
kf = cv2.KalmanFilter(4, 2)
kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], 
								[0, 0, 1, 0], [0, 0, 0, 1]], np.float32)

stream = cv2.VideoCapture(1)
while True:
	active, frame = stream.read()
	if active:
		# Frame config and HSV conversion
		frame = imutils.resize(frame, width=600)
		frame = cv2.flip(frame, 0)
		# frame = cv2.flip(frame, 1)
		blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
		hsv[...,1] = hsv[...,1]*10

		# Mask ball
		mask = cv2.inRange(hsv, orangeLower, orangeUpper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)

		# Get contour
		contour = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		contour = imutils.grab_contours(contour)
		
		# cv2.rectangle(frame, rect_tl, rect_br, (0, 255, 0), 2)

		# Identify largest contour and draw circle with center
		x, y = 0, 0
		predicted = (x, y)
		if len(contour) > 0:
			c = max(contour, key=cv2.contourArea)
			((X_min, Y_min), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)

			x, y = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
			center = (x, y)

			# Sample coordinates every 2.5 milliseconds
			timeNow = int(time() * 1000)
			if timeNow - timePrev >= 2.5:
				coords.append(center)
				radii.append(radius)

				timePrev = timeNow
			
			# print(radii)
			radii_sorted = sorted(radii)
			radius = radii_sorted[int(len(radii) / 2)]
			# cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
			# cv2.circle(frame, center, 5, (0, 0, 255), -1)
		
			# Prediction
			for coord in coords:
				kf.correct(np.array([[np.float32(coord[0])], [np.float32(coord[1])]]))

			for i in range(5):
				kf.correct(np.array([[np.float32(predicted[0])], [np.float32(predicted[1])]]))
				predicted = kf.predict()

				if (predicted[0] >= 1 and predicted[0] <= 598) and (predicted[1] >= 1 and predicted[1] <= 335):
					coords.append(predicted)
					cv2.circle(frame, predicted, 5, (255, 0, 0), -1)
				else:
					break
		
			try:
				if not got_init:
					# Get initial height
					h0 = radii[0]

					# Calculate xy velocity
					ds = math.sqrt((coords[9][0] - coords[0][0])**2 + (coords[9][1] - coords[0][1])**2)
					dt = 2.5 * 10
					v_xy = ds / dt

					# Calculate z velocity
					dz = float(radii[1] - radii[0])
					dt = 2.5 * 1
					# v_z = dz / dt
					v_z = 0.15

					# Calculate distance until target
					if not (v_z == 0 or v_xy == 0 or h0 == 0):
						s = (-(v_z/v_xy) - math.sqrt(((v_z/v_xy)**2 - 4*(-a_z/(2*(v_xy**2)))*(h0-2.83))))/(2*(-a_z/(2*(v_xy**2))))

					x_start = coords[0][0]
					y_start = coords[0][1]
					got_init = True
			except:
				pass
		
		# Best fit line through coordinates
		if len(coords) > 1:
			pts = np.array(coords, dtype=np.float32)
			
			[vx, vy, x0, y0] = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)

			lefty = int((-x0 * vy / vx) + y0)
			righty = int(((frame.shape[1] - x0) * vy / vx) + y0)

			slope = float((righty - lefty) / (frame.shape[1] - 1))
			y_line = float(lefty)

			# If predicted line coordinates are in zone, add to possibilities
			for x_line in range(0, (rect_br[0]+1)):
				y_line += slope

				if x_line >= rect_tl[0] and y_line >= rect_tl[1] and y_line <= rect_br[1]:
					# Map prediction to stepper motor coordinates
					x_mapped = round(np.interp(x_line - rect_tl[0], [0, rect_br[0] - rect_tl[0]], [min, X_max]).item())
					y_mapped = round(np.interp(rect_br[1] - round(y_line), [0, rect_br[1] - rect_tl[1]], [min, Y_max]).item())

					# target_coords.append((x_mapped, y_mapped))
					target_coords.append((x_line, round(y_line)))
			
			x_lim = 0
			step = 1
			if slope > 0:
				x_lim = rect_tl[0]-1
				step = -1
			else:
				x_lim = rect_br[0]+1

			y_line = y_start
			if not found_target:
				for x_line in range(x_start, x_lim, step):
					if not found_target:
						y_line -= abs(slope)

						# Calculate distance
						diff = math.sqrt((x_line - x_start)**2 + (y_line - y_start)**2)
						print(diff)
						if diff >= s:
							target = (x_line, round(y_line))
							found_target = True
							# if target[1] <= 335:
								# cv2.circle(frame, target, 1, (0, 0, 255), -3)

			# Draw possible target coords
			# try:
			# 	# cv2.line(frame, (frame.shape[1] - 1, righty), (0, lefty), (255, 0, 0), 2)
				
			# 	for coord in target_coords:
			# 		cv2.circle(frame, (coord[0], coord[1]), 1, (0, 255, 255), -1)
			# except:
			# 	pass
		
		# Make trail
		# for i in range(1, len(coords)):
		# 	if coords[i - 1] is None or coords[i] is None:
		# 		continue
			
		# 	cv2.line(frame, coords[i - 1], coords[i], (0, 0, 255), 5)

		# Show frame
		cv2.imshow("Frame", frame)
		key = cv2.waitKey(1) & 0xFF
		if key == ord("q"):
			break

stream.release()
cv2.destroyAllWindows()

print(f"Initial height: {h0}")
print(f"XY velocity: {v_xy}")
print(f"Z velocity: {v_z}")
print(f"Displacement: {s}")
print()
print(f"Target: {target}")