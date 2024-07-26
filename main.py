from collections import deque

import imutils
import numpy as np
import cv2

from time import time

timePrev = 0

# Target zone boundaries
rect_tl = (288, 30)
rect_br = (360, 94)

# HSV boundaries
orangeLower = (7, 0, 219)
orangeUpper = (14, 255, 255)

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
def track():
	global timePrev
	global rect_tl
	global rect_br
	global orangeLower
	global orangeUpper
	global coords
	global radii
	global target_coords
	global kf

	while True:
		active, frame = stream.read()
		if active:
			# Frame config and HSV conversion
			frame = imutils.resize(frame, width=600)
			frame = cv2.flip(frame, 0)
			frame = cv2.flip(frame, 1)
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
					coords.appendleft(center)
					radii.appendleft(radius)

					timePrev = timeNow

				radii_sorted = sorted(radii)
				radius = radii_sorted[int(len(radii) / 2)]
			
				# Prediction
				for coord in coords:
					kf.correct(np.array([[np.float32(coord[0])], [np.float32(coord[1])]]))

				for i in range(5):
					kf.correct(np.array([[np.float32(predicted[0])], [np.float32(predicted[1])]]))
					predicted = kf.predict()

					if (predicted[0] >= 1 and predicted[0] <= 598) and (predicted[1] >= 1 and predicted[1] <= 335):
						coords.appendleft(predicted)
					else:
						break
			
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
						target_coords.appendleft((x_line, round(y_line)))
