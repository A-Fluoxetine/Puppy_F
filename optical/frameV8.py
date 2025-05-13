import cv2
import numpy as np

# 锟斤拷始锟斤拷锟斤拷锟斤拷头
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 锟斤拷态锟斤拷锟斤拷锟斤拷始锟斤拷
poly_epsilon = 0.02  # 锟斤拷锟斤拷谓锟斤拷凭锟斤拷锟较碉拷锟?min_area = 2000      # 锟斤拷小锟斤拷锟斤拷锟斤拷锟斤拷锟街?
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 预锟斤拷锟斤拷锟斤拷强锟斤拷锟斤拷源锟斤拷锟斤拷锟斤拷呕锟斤拷锟?    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.bilateralFilter(gray, 11, 75, 75)  # 锟斤拷锟斤拷锟剿诧拷
    thresh = cv2.adaptiveThreshold(
        blurred, 255,
        cv2.ADAPTIVE_THRESH_MEAN_C,
        cv2.THRESH_BINARY_INV, 201, 8
    )
    
    # 强锟斤拷锟斤拷缘锟斤拷锟斤拷
    kernel = np.ones((7,7), np.uint8)
    processed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    processed = cv2.dilate(processed, None, iterations=2)

    # 锟斤拷锟斤拷锟斤拷锟?    contours, _ = cv2.findContours(processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue

        # 锟斤拷锟斤拷谓锟斤拷疲锟斤拷锟斤拷呔锟斤拷龋锟?        perimeter = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, poly_epsilon * perimeter, True)
        vertices = len(approx)

        # ==== 锟斤拷锟斤拷/锟斤拷锟斤拷双锟截硷拷锟?====
        is_rect = False
        rect = cv2.minAreaRect(cnt)
        (_, _), (w, h), angle = rect
        box = cv2.boxPoints(rect)
        box = box.astype(int)  # 使锟斤拷astype直锟斤拷转锟斤拷锟斤拷锟斤拷
        
        
        # 锟斤拷锟斤拷1锟斤拷锟斤拷转锟斤拷锟轿匡拷锟竭憋拷
        aspect_ratio = max(w, h) / (min(w, h) + 1e-5)
        cond1 = (0.9 < aspect_ratio < 1.1) if (abs(angle) < 5 or abs(angle) > 85) else (aspect_ratio < 1.2)

        # 锟斤拷锟斤拷2锟斤拷锟斤拷锟斤拷味锟斤拷锟斤拷锟街?        cond2 = (4 <= vertices <= 6) and cv2.isContourConvex(approx)

        if cond1 and cond2:
            # 锟斤拷锟斤拷实锟绞边筹拷锟斤拷锟斤拷
            side_ratios = []
            for i in range(4):
                p1 = box[i]
                p2 = box[(i+1)%4]
                side_ratios.append(np.linalg.norm(p1-p2))
            max_side = max(side_ratios)
            ratio_var = np.std(side_ratios) / max_side
            
            if ratio_var < 0.15:  # 锟竭筹拷锟斤拷锟斤拷小锟斤拷15%
                is_rect = True
                shape = "SQUARE" if (aspect_ratio < 1.1 and abs(angle) < 5) else "RECT"
                
                # 锟斤拷锟斤拷锟斤拷转锟斤拷锟斤拷
                cv2.drawContours(frame, [box], 0, (0,255,0), 2)

        # ==== 圆锟轿硷拷锟?====
        if not is_rect:
            # 锟侥斤拷圆锟轿度硷拷锟斤拷
            circularity = (4 * np.pi * area) / (perimeter**2 + 1e-5)
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            
            if circularity > 0.85 and abs(1 - (area / (np.pi*(radius**2)))) < 0.2:
                cv2.circle(frame, (int(x),int(y)), int(radius), (255,0,0), 2)
                shape = "CIRCLE"
            else:
                continue

        # 锟斤拷注锟斤拷息
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.putText(frame, f"{shape} {area}", (cX-50, cY), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

    # 锟斤拷锟斤拷锟斤拷示
    cv2.imshow('Processed', processed)
    cv2.imshow('Detection', frame)
    
    # 锟斤拷锟教匡拷锟斤拷
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
    elif key == ord('+'):
        poly_epsilon = min(0.1, poly_epsilon + 0.005)
    elif key == ord('-'):
        poly_epsilon = max(0.01, poly_epsilon - 0.005)

cap.release()
cv2.destroyAllWindows()
