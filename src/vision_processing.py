# vision_processing.py
# Object Detection with OpenCV
import cv2

def detect_objects(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    return edges
