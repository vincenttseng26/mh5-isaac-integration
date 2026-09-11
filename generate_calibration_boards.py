import cv2
import cv2.aruco as aruco

# 1. Generate ChArUco Board (Highly recommended for precision)
# A4 size is roughly 210x297mm. We'll use a grid of 5x7 squares.
# Square length: 30mm (0.03m), Marker length: 22.5mm (0.0225m)
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
board = aruco.CharucoBoard_create(5, 7, 0.03, 0.0225, aruco_dict)

# Generate image (high resolution for printing)
# 5 squares * 30mm = 150mm wide
# 7 squares * 30mm = 210mm high
# 100 pixels per 10mm -> 1500 x 2100 + margins
img = board.draw((1500, 2100), marginSize=100, borderBits=1)
cv2.imwrite("ChArUco_Board_5x7_30mm.png", img)

# 2. Generate standard Checkerboard (Alternative)
import numpy as np
# 6x8 squares (internal corners will be 5x7)
squares_x = 6
squares_y = 8
square_size_px = 300

checkerboard = np.zeros((squares_y * square_size_px, squares_x * square_size_px), dtype=np.uint8)
for i in range(squares_y):
    for j in range(squares_x):
        if (i + j) % 2 == 1:
            checkerboard[i*square_size_px:(i+1)*square_size_px, j*square_size_px:(j+1)*square_size_px] = 255

# Add white margin
margin = 150
checkerboard_with_margin = cv2.copyMakeBorder(checkerboard, margin, margin, margin, margin, cv2.BORDER_CONSTANT, value=[255, 255, 255])
cv2.imwrite("Checkerboard_6x8.png", checkerboard_with_margin)

print("Generated ChArUco_Board_5x7_30mm.png and Checkerboard_6x8.png")
