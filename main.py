import RPi.GPIO as GPIO
import imutils
import cv2
import numpy as np

from collections import deque
from time import time

from movement import to_pos, min, X_max, Y_max

# Start in center
to_pos(int(X_max / 2), int(Y_max / 2), (.0005) * 2)

# Initialize variables
timePrev = 0
found_x_target = False
found_y_target = False
x_target = 0
y_target = 0

# Stepper motors
X_max = 1450
Y_max = 1600
min = 0

# Target zone boundaries
rect_tl = (276, 24)
rect_br = (343, 88)

# HSV boundaries
orangeLower = (7, 0, 219)
orangeUpper = (14, 255, 255)

# Define lists for data
buffer = 50
coords = deque(maxlen=buffer)
radii = deque(maxlen=5)
target_coords = deque(maxlen=rect_br[0])

stream = cv2.VideoCapture(0)
try:
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
			
			# Determine target
			if not found_y_target:
				if len(coords) >= 5 and len(radii) >= 5:
					# Predict y coordinate with radius
					y_data = np.array([coords[0][1], coords[1][1], coords[2][1], coords[3][1], coords[4][1]])
					radii_data = np.array([radii[0], radii[1], radii[2], radii[3], radii[4]])

					coefficients = np.polyfit(y_data, radii_data, 2)

					y_predict = np.linspace(max(y_data), 0, 100)
					radii_predict = np.polyval(coefficients, y_predict)

					for i in range(len(radii_predict)):
						if radii_predict[i] <= 0:
							y_target = round(y_predict[i])

							# Limit to zone bounds
							if y_target > rect_br[1]:
								y_target = rect_br[1]
							elif y_target < rect_tl[1]:
								y_target = rect_tl[1]
							
							found_y_target = True
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

					if x_line >= rect_tl[0]:
						if not found_x_target and found_y_target:
							if slope > 0:
								if y_line >= y_target:
									x_target = x_line

									# Map prediction to stepper motor coordinates
									x_mapped = round(np.interp(x_target - rect_tl[0], [0, rect_br[0] - rect_tl[0]], [min, X_max]).item())
									y_mapped = round(np.interp(rect_br[1] - round(y_target), [0, rect_br[1] - rect_tl[1]], [min, Y_max]).item())
									to_pos(X_max - x_mapped, Y_max - y_mapped)

									found_x_target = True
							elif slope < 0:
								if y_line <= y_target:
									x_target = x_line

									# Map prediction to stepper motor coordinates
									x_mapped = round(np.interp(x_target - rect_tl[0], [0, rect_br[0] - rect_tl[0]], [min, X_max]).item())
									y_mapped = round(np.interp(rect_br[1] - round(y_target), [0, rect_br[1] - rect_tl[1]], [min, Y_max]).item())
									to_pos(X_max - x_mapped, Y_max - y_mapped)

									found_x_target = True
except KeyboardInterrupt:
	print(f"Target: {x_target, y_target}")
	stream.release()
	GPIO.cleanup()