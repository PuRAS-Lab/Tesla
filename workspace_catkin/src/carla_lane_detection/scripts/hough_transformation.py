import cv2
import numpy as np

def hough_transformation(image, hough_rho, hough_theta, hough_trshld, hough_min_line_len, hough_max_line_gap):
    lines = cv2.HoughLinesP(
        image,
        rho = hough_rho,
        theta = hough_theta,
        threshold = hough_trshld,
        lines=np.array([]),
        minLineLength = hough_min_line_len,
        maxLineGap = hough_max_line_gap
    )

    return lines