"""
视觉识别基本实现原理：
1. 摄像头采集：通过OpenCV捕获实时视频流
2. 图像预处理：
   - 灰度化 + 双边滤波（保边去噪）
   - 自适应阈值处理（增强对比度）
   - 形态学操作（闭运算+膨胀，强化轮廓）
3. 形状检测：
   - 矩形/正方形：通过最小外接矩形+边长比例+角度判断
   - 圆形：通过圆形度+面积吻合度判断
4. 动态参数调节：
   - 可通过键盘'+'/'-'实时调整轮廓近似精度
   - 最小面积阈值过滤噪声
"""

import cv2
import numpy as np

# 初始化摄像头（默认摄像头，分辨率640x480）
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 算法参数初始化
poly_epsilon = 0.02  # 轮廓近似精度系数（值越小越精确）
min_area = 2000      # 有效形状的最小面积阈值（过滤噪声）

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # ----------------- 图像预处理 -----------------
    # 灰度化 + 双边滤波（保留边缘的同时降噪）
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.bilateralFilter(gray, 11, 75, 75)
    
    # 自适应阈值处理（局部二值化，增强对比度）
    thresh = cv2.adaptiveThreshold(
        blurred, 255,
        cv2.ADAPTIVE_THRESH_MEAN_C,
        cv2.THRESH_BINARY_INV, 201, 8
    )
    
    # 形态学操作：闭运算（填充空洞） + 膨胀（强化轮廓）
    kernel = np.ones((7,7), np.uint8)
    processed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    processed = cv2.dilate(processed, None, iterations=2)

    # ----------------- 轮廓检测 -----------------
    contours, _ = cv2.findContours(processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:  # 过滤小面积噪声
            continue

        # 计算轮廓周长和多边形近似
        perimeter = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, poly_epsilon * perimeter, True)
        vertices = len(approx)  # 获取顶点数量

        # ===== 矩形/正方形检测 =====
        is_rect = False
        rect = cv2.minAreaRect(cnt)  # 最小外接矩形
        (_, _), (w, h), angle = rect
        box = cv2.boxPoints(rect)
        box = box.astype(int)  # 坐标转换为整数
        
        # 条件1：宽高比判断（根据旋转角度动态调整）
        aspect_ratio = max(w, h) / (min(w, h) + 1e-5)  # 避免除零
        cond1 = (0.9 < aspect_ratio < 1.1) if (abs(angle) < 5 or abs(angle) > 85) else (aspect_ratio < 1.2)

        # 条件2：顶点数和凸性检测
        cond2 = (4 <= vertices <= 6) and cv2.isContourConvex(approx)

        if cond1 and cond2:
            # 额外验证：边长比例标准差（排除不规则四边形）
            side_lengths = [np.linalg.norm(box[i]-box[(i+1)%4]) for i in range(4)]
            max_side = max(side_lengths)
            ratio_var = np.std(side_lengths) / max_side
            
            if ratio_var < 0.15:  # 边长波动小于15%
                is_rect = True
                shape = "SQUARE" if (aspect_ratio < 1.1 and abs(angle) < 5) else "RECT"
                cv2.drawContours(frame, [box], 0, (0,255,0), 2)  # 绿色绘制矩形

        # ===== 圆形检测 =====
        if not is_rect:
            circularity = (4 * np.pi * area) / (perimeter**2 + 1e-5)  # 圆形度计算
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            
            # 圆形条件：圆形度>0.85 且 实际面积与外接圆面积比>80%
            if circularity > 0.85 and abs(1 - (area / (np.pi*(radius**2)))) < 0.2:
                cv2.circle(frame, (int(x),int(y)), int(radius), (255,0,0), 2)  # 蓝色绘制圆形
                shape = "CIRCLE"
            else:
                continue

        # 在形状中心标注类型和面积
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.putText(frame, f"{shape} {area}", (cX-50, cY), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)  # 红色文字

    # 显示处理结果
    cv2.imshow('Processed', processed)  # 二值化图像
    cv2.imshow('Detection', frame)      # 检测结果
    
    # 键盘交互控制
    key = cv2.waitKey(1)
    if key == ord('q'):  # 退出程序
        break
    elif key == ord('+'):  # 增加轮廓近似精度
        poly_epsilon = min(0.1, poly_epsilon + 0.005)
    elif key == ord('-'):  # 降低轮廓近似精度
        poly_epsilon = max(0.01, poly_epsilon - 0.005)

# 释放资源
cap.release()
cv2.destroyAllWindows()
