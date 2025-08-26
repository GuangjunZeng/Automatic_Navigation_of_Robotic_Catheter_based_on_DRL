import zmq
import cv2
import numpy as np
import numpy as np
import json

def color_point_detect(color_array1, color_array2, left1, top1, left2, top2, delta1, delta2):
    #H (Hue，色调): 表示颜色的基本类型（如红色、绿色、蓝色等）。红色：0° 到 12° 和 160° 到 180°。
    #S (Saturation，饱和度): 值越小，颜色越接近灰色；值越大，颜色越鲜艳。
    #V (Value，明度): 值越小，颜色越暗；值越大，颜色越亮。
    # 设置红色阈值（HSV颜色空间）
    lower_red1 = np.array([0, 3, 3])     #降低饱和度和明度的下限
    upper_red1 = np.array([18, 255, 255])  #稍微扩大色调范围
    lower_red2 = np.array([158, 3, 3])
    upper_red2 = np.array([180, 255, 255])

    # 确保输入是 NumPy 数组
    if not isinstance(color_array1, np.ndarray):
        color_array1 = np.array(color_array1, dtype=np.uint32)
    if not isinstance(color_array2, np.ndarray):
        color_array2 = np.array(color_array2, dtype=np.uint32)

    # 提取 R、G、B 通道
    r1 = (color_array1 >> 16) & 0xFF  # 右移 16 位，取低 8 位
    g1 = (color_array1 >> 8) & 0xFF  # 右移 8 位，取低 8 位
    b1 = color_array1 & 0xFF  # 取低 8 位
    r2 = (color_array2 >> 16) & 0xFF  # 右移 16 位，取低 8 位
    g2 = (color_array2 >> 8) & 0xFF  # 右移 8 位，取低 8 位
    b2 = color_array2 & 0xFF  # 取低 8 位
    # 将 R、G、B 合并为 3 通道图像
    bgr_image1 = np.stack([b1, g1, r1], axis=-1).astype(np.uint8)
    bgr_image2 = np.stack([b2, g2, r2], axis=-1).astype(np.uint8)

    # 显示图像
    #cv2.imshow("Decoded Image", bgr_image)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    # 将 BGR 图像转换为 HSV 颜色空间
    hsv1 = cv2.cvtColor(bgr_image1, cv2.COLOR_BGR2HSV)
    hsv2 = cv2.cvtColor(bgr_image2, cv2.COLOR_BGR2HSV)

    # 创建红色掩膜
    mask_low1 = cv2.inRange(hsv1, lower_red1, upper_red1)  # 提取图像中符合红色范围1的区域
    mask_upper1 = cv2.inRange(hsv1, lower_red2, upper_red2)  # 提取图像中符合红色范围2的区域。
    mask1 = cv2.bitwise_or(mask_low1, mask_upper1)  # 将 mask1 和 mask2 合并，得到完整的红色区域掩膜。

    mask_low2 = cv2.inRange(hsv2, lower_red1, upper_red1)  # 提取图像中符合红色范围1的区域
    mask_upper2 = cv2.inRange(hsv2, lower_red2, upper_red2)  # 提取图像中符合红色范围2的区域。
    mask2 = cv2.bitwise_or(mask_low2, mask_upper2)  # 将 mask1 和 mask2 合并，得到完整的红色区域掩膜。

    # 形态学操作去噪
    kernel = np.ones((2, 2), np.uint8)
    mask1 = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, kernel)
    mask2 = cv2.morphologyEx(mask2, cv2.MORPH_OPEN, kernel)

    # 查找轮廓
    contours1, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours2, _ = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 打印轮廓数量
    num_contours1 = len(contours1)
    num_contours2 = len(contours2)
    print(f"找到的轮廓1数量: {num_contours1}")
    print(f"找到的轮廓2数量: {num_contours2}")
    cx = 500
    cy = 500
    contours = contours2
    left = left2
    top = top2
    start_point_x = left1 + delta1
    start_point_y = top1 + delta1
    if num_contours1 > num_contours2:
        contours = contours1
        left = left1
        top = top1
        start_point_x = left2 + delta2
        start_point_y = top2 + delta2


    for cnt in contours:
        print('begin to detect')
        area = cv2.contourArea(cnt)
        if area < 2:  # 过滤小面积噪点
            print('fail1')
            continue

        # 计算圆度
        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            print('fail2')
            continue
        circularity = 4 * np.pi * area / (perimeter ** 2)

        if circularity > 0.03:  # 圆度阈值（1为完美圆）
            # 计算中心坐标
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                print(f"检测到红点坐标：({cx}, {cy})")
                break

                # 绘制结果
                # cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
                # cv2.drawContours(frame, [cnt], 0, (0, 255, 0), 2)
            else:
                print("未检测到红点在这一帧")
    c = 1
    cx = cx + left
    cy = cy + top
    return cx, cy, c, start_point_x, start_point_y


context = zmq.Context()
socket = context.socket(zmq.REP)
#服务器端（IP: 10.31.94.188）
socket.bind("tcp://0.0.0.0:0222") #服务器端

while True:
    message = socket.recv_json()
    # print("Received message:", message)  # Debugging: Print the received message
    try:
        # 提取 color_array
        color_array1, color_array2, left1, top1, left2, top2, delta1, delta2 = message['a'], message['a2'], message['left1'], message['top1'], message['left2'], message['top2'], message['delta1'], message['delta2']
        cx, cy, c, start_point_x, start_point_y = color_point_detect(color_array1, color_array2, left1, top1, left2, top2, delta1, delta2)
        # 创建包含多个参数的字典
        data_to_send = {
            'c': c,
            'cx': cx,
            'cy': cy,
            'start_point_x': start_point_x,
            'start_point_y': start_point_y
        }
        socket.send_json(data_to_send)
    except KeyError as e:
        print(f"KeyError: {e}. Message received: {message}")
        socket.send_json({'error': str(e), 'message': message})
