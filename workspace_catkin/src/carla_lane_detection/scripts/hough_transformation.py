import cv2
import numpy as np
import math


def draw_lines(img, lines):
    # Nacrtati dvije linije na slici
    for i in range(2):
        x1 = lines[i][0][0]
        y1 = lines[i][0][1]    
        x2 = lines[i][0][2]
        y2 = lines[i][0][3]
        cv2.line(img,(x1,y1),(x2,y2),(255,0,0),4)

    return img

def extrapolate_two_lines(lines):
    lines_info = []
    two_lines = []
    
    # Ako nema linija ne radi nista, vrati praznu listu
    if lines is None:
        pass
    elif lines.shape[0] == 1:
        two_lines.append(lines[0])
        two_lines.append(lines[0])
    else:
        for i in range(lines.shape[0]):
            x1 = lines[i][0][0]
            y1 = lines[i][0][1]    
            x2 = lines[i][0][2]
            y2 = lines[i][0][3]
            # Racunanje duzine linije
            l = math.sqrt(math.pow((x2 - x1), 2) + math.pow((y2 - y1), 2))
            # izbacivanje horizontalnih linija
            if abs(y2 - y1) > 50:
                lines_info.append([x1, y1, x2, y2, l, i])
        # sortiranje po duzini
        temp = sorted(lines_info, key = lambda x: int(x[4]))
        # Uzimanje najduze linije kao jedne od dvije potrebna
        two_lines.append(lines[temp[0][5]])

        for j in temp[1:]:
            # Odredjivanje druge linije prolaskom kroz ostatak sortirane liste
            # I uzimanje linije koja nije blizu prve koja je uzeta
            if abs(j[2] - temp[0][2]) > 100 and abs(j[3] - temp[0][3]) < 20:
                break
        # Ako nije pronadjena druga linija, uzeti drugu po duzini iz liste
        if len(two_lines) != 2:
            two_lines.append(lines[temp[2][5]])

    return two_lines
    

def hough_transformation(image, original_image, hough_rho, hough_theta, hough_trshld, hough_min_line_len, hough_max_line_gap):
    # Izvrsiti Houghovu transformaciju nad ulaznom slikom
    lines = cv2.HoughLinesP(
        image,
        rho = hough_rho,
        theta = hough_theta,
        threshold = hough_trshld,
        lines=np.array([]),
        minLineLength = hough_min_line_len,
        maxLineGap = hough_max_line_gap
    )
    
    # Izdvojiti dvije linije
    lines = extrapolate_two_lines(lines)
    
    if lines is not None:
        # Ako postoje linije onda ih nacrtati na originalnoj slici
        line_image = draw_lines(original_image, lines)
    else:
        # Ako ne postoje linije onda vratiti originalnu sliku bez izmjena
        line_image = original_image

    return line_image
