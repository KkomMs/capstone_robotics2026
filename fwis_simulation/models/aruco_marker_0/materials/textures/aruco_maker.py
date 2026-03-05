import cv2
import cv2.aruco as aruco

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
marker_img = aruco.generateImageMarker(dictionary, 0, 200)  # ID=0, 200px
cv2.imwrite("aruco_marker_0.png", marker_img)
print("Completed.")