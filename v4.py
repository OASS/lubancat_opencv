import cv2
import numpy as np
from pyzbar.pyzbar import decode
from pyzbar import pyzbar
import serial  # 安装pyserial，但import serial，且不能安装serial
import time

Rad_hsv = [153, 203, 63, 177, 87, 221]
# Rad_hsv = [161, 197, 50, 205, 146, 224]  # lowerbH,upperbH,lowerbS,upperbS,lowerbV,upperbV
Yellow_hsv = [13, 36, 96, 186, 152, 255]  # lowerbH,upperbH,lowerbS,upperbS,lowerbV,upperbV
Blue_hsv = [93, 129, 251, 255, 60, 201]  # lowerbH,upperbH,lowerbS,upperbS,lowerbV,upperbV
White_hsv = [79, 100, 0, 66, 202, 255]

#roi设定################################################################################# 
x1, y1, w1, h1 = 10, 0, 620, 320 #roi1左上角坐标和宽度、高度roi = image[y:y+h, x:x+w]
x2, y2, w2, h2 = 0, 0, 640, 280 #roi2
x3, y3, w3, h3 = 0, 0, 640, 480 #roi3

while True:
    try:
        # 尝试连接串口
        uart = serial.Serial("/dev/ttyS3", 115200, 8, 'N', 1)  # 尝试连接串口/dev/ttys3
        # 如果连接成功，退出循环
        break
    except serial.SerialException:
        # 如果连接失败，打印错误信息
        print("未找到可用的串口或无法打开串口")
        # 等待一段时间后再次尝试连接


def delay_ms(TI):
    time_ms = TI / 1000
    time.sleep(time_ms)


# 命令表
# 鲁班猫发送：
cmd_rect = b'\x21'  # 方块（红蓝方相同）
cmd_circle = b'\x22'  # 圆环（红蓝方相同）
cmd_null = b'\x26'    #非识别目标（红蓝方相同）
cmd_R = b'\x23'  # 红球（转盘）
cmd_B = b'\x24'  # 蓝球（转盘）
cmd_Y = b'\x25'  # 黄球（转盘）
############################################################
# 鲁班猫接收
b'\x01'  # 转盘任务         
b'\x02'  # 红色方识别阶梯平台
b'\x03'  # 蓝色方识别阶梯平台
b'\x04'  # 识别柱台
b'\x05'  # 暂停
b'\x06'  # 终止

def Int_translation(number):  # 10进制转化成为两个16进制
    decimal_number = number
    x1 = int(decimal_number / 256)
    y1 = int(decimal_number % 256)
    return x1, y1


data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
x = 0


def UartReceiveDate():  # 接收，这个函数不能运行太快，否则会导致串口读取太快导致出错
    global Find_Task
    global Target_Num
    global x
    global data
    x = 0
    # uart_num = uart.in_waiting
    if uart.in_waiting:  # 如果收到数据
        data[0] = uart.read(1)
        data[1] = uart.read(1)
        data[2] = uart.read(1)
        data[3] = uart.read(1)
        data[4] = uart.read(1)
        data[5] = uart.read(1)
        if data[x] == b'\xa5' and data[x + 5] == b'Z' and x < 5:
            Find_Task = data[x + 2]
            Find_Task = Find_Task
            print(Find_Task)
            return Find_Task
            x = 0
        elif x >= 5:
            x = 0
        x += 1


def communication(com, data1, data2):  # 发送
    uart.write(bytearray([0xA5]))  # 发送一个十六进制数据头
    uart.write(bytearray([0x02]))  # 数据长度
    uart.write(com)  # 命令符号
    uart.write(bytearray([data1]))  # 高位数组
    uart.write(bytearray([data2]))  # 低位数据
    uart.write(bytearray([0x5A]))  # 发送一个十六进制数据尾


def find_cir(cropped_frame):
    CX = -1
    CY = -1
    CR = -1

    
    #1、转化为灰度图并图像平滑
    gray = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2GRAY)
    dst = cv2.medianBlur(gray, 5)
    cv2.imshow("滤波", dst)
    #canny边缘识别
    canny = cv2.Canny(dst, 50, 150)
    cv2.imshow("canny", canny)

    circle_r = []

    # 霍夫曼圆圈检测
    # circles = cv2.HoughCircles(image,● image：输入图像，即源图像，类型为8位的单通道灰度图像。
    #                            method, ● method：检测方法。
    #                            dp,     ● dp：累计器分辨率，它是一个分割比率，用来指定图像分辨率与圆心累加器分辨率的比例。例如，如果dp = 1，则输入图像和累加器具有相同的分辨率。
    #                            minDist,● minDist：圆心间的最小间距。该值被作为阈值使用，如果存在圆心间距离小于该值的多个圆，则仅有一个会被检测出来。
    #                                       因此，如果该值太小，则会有多个临近的圆被检测出来；如果该值太大，则可能会在检测时漏掉一些圆。
    #                            param1, ● param1：该参数是缺省的，在缺省时默认值为100。它对应的是Canny边缘检测器的高阈值（低阈值是高阈值的二分之一）。
    #                            param2, ● param2：圆心位置必须收到的投票数。只有在第1轮筛选过程中，投票数超过该值的圆，才有资格进入第2轮的筛选。
    #                                           因此，该值越大，检测到的圆越少；该值越小，检测到的圆越多。这个参数是缺省的，在缺省时具有默认值100。
    #                            minRadius,● minRadius：圆半径的最小值，小于该值的圆不会被检测出来。该参数是缺省的，在缺省时具有默认值0，此时该参数不起作用。
    #                            maxRadius) ● maxRadius：圆半径的最大值，大于该值的圆不会被检测出来。该参数是缺省的，在缺省时具有默认值0，此时该参数不起作用。
    # ● circles：返回值，由圆心坐标和半径构成的numpy.ndarray。

    circles = cv2.HoughCircles(dst, cv2.HOUGH_GRADIENT, 1, 1200, param1=100, param2=30, minRadius=30,
                               maxRadius=150)
    if circles is not None and circles.all():
        circles = np.uint16(np.around(circles))
        # 遍历
        for circle in circles[0, :]:
            # if ((circle[2]>150)and (circle[2]<190)):
            # 画出圆心
            cv2.circle(cropped_frame, (circle[0], circle[1]), 3, (0, 255, 0), -1)
            # 画出圆的边界
            cv2.circle(cropped_frame, (circle[0], circle[1]), circle[2], (0, 0, 255), 2)
            circle_r = circle
            CX = circle[0]
            CY = circle[1]
            CR = circle[2]

    print("CX:", CX)
    print("Cy:", CY)
    print("Cr:", CR)
    return CR

def Identify_QRcode():
    #循环3次
    for i in range(10):#识别几次
        cod, codeimge = usb_cap.read()
        codeimge = codeimge[y1:y1 + h1 , x1:x1 + w1] #roi1

        if cod:
            for barcode in decode(codeimge):
                print("barcode.data:", barcode.data)
                print("barcode.rect", barcode.rect)
                myData = barcode.data.decode('utf-8')
                print("myData", myData)
                if (myData == 'B'):
                    print("B")
                    return myData
                elif (myData == 'R'):
                    print("R")
                    return myData
    else:
        print("None")
        return "None"


def find_blob(frame):
    frame = frame[y2:y2 + h2 , x2:x2 + w2] #roi2

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    ball_rad_lower = np.array([Rad_hsv[0], Rad_hsv[2], Rad_hsv[4]])
    ball_rad_upper = np.array([Rad_hsv[1], Rad_hsv[3], Rad_hsv[5]])

    ball_blue_lower = np.array([Blue_hsv[0], Blue_hsv[2], Blue_hsv[4]])
    ball_blue_upper = np.array([Blue_hsv[1], Blue_hsv[3], Blue_hsv[5]])

    ball_yellow_lower = np.array([Yellow_hsv[0], Yellow_hsv[2], Yellow_hsv[4]])
    ball_yellow_upper = np.array([Yellow_hsv[1], Yellow_hsv[3], Yellow_hsv[5]])

    red_mask = cv2.inRange(hsv, ball_rad_lower, ball_rad_upper)  # 掩膜处理
    blue_mask = cv2.inRange(hsv, ball_blue_lower, ball_blue_upper)  # 掩膜处理
    yellow_mask = cv2.inRange(hsv, ball_yellow_lower, ball_yellow_upper)  # 掩膜处理

    # 合并三个颜色的二值图像
    mask = cv2.bitwise_or(red_mask, cv2.bitwise_or(yellow_mask, blue_mask))

    # 腐蚀和膨胀操作
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    max_area = 0
    max_contour = None
    color = ''
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > max_area:
            max_area = area
            max_contour = contour

    if max_contour is not None:
        # 计算最大轮廓的外接矩形
        x, y, w, h = cv2.boundingRect(max_contour)
        # 绘制外接矩形和中心点
        cv2.rectangle(hsv, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(hsv, (x + w // 2, y + h // 2), 5, (0, 255, 0), -1)
        cv2.imshow("mask", mask)

        # 判断最大色块的颜色
        if np.count_nonzero(red_mask) > 20000:
            print(np.count_nonzero(red_mask))
            communication(cmd_R, 0, 0)
            color = 'Red'

        elif np.count_nonzero(yellow_mask) > 20000:
            print(np.count_nonzero(yellow_mask))
            communication(cmd_Y, 0, 0)
            color = 'Yellow'
        elif np.count_nonzero(blue_mask) > 20000:
            print(np.count_nonzero(blue_mask))
            communication(cmd_B, 0, 0)
            color = 'Blue'
        # else:
        #     color = ''

        # 在窗口左上角显示颜色
        cv2.putText(hsv, color, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    return hsv, color


def find_red(frame):
    frame = frame[y1:y1 + h1 , x1:x1 + w1] #roi1

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    ball_rad_lower = np.array([Rad_hsv[0], Rad_hsv[2], Rad_hsv[4]])
    ball_rad_upper = np.array([Rad_hsv[1], Rad_hsv[3], Rad_hsv[5]])

    red_mask = cv2.inRange(hsv, ball_rad_lower, ball_rad_upper)  # 掩膜处理

    # 腐蚀和膨胀操作
    red_mask = cv2.erode(red_mask, None, iterations=2)
    red_mask = cv2.dilate(red_mask, None, iterations=2)

    # 查找轮廓
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    max_area = 0
    max_contour = None
    color = ''
    shape = ''
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > max_area:
            max_area = area
            max_contour = contour

    if max_contour is not None:
        # 计算最大轮廓的外接矩形
        x, y, w, h = cv2.boundingRect(max_contour)
        # 绘制外接矩形和中心点
        cv2.rectangle(hsv, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(hsv, (x + w // 2, y + h // 2), 5, (0, 255, 0), -1)
        cv2.imshow("mask", red_mask)
        cropped_frame = frame[y:y + h, x:x + w]#裁剪后的图像
        #cropped_red_mask = red_mask[y:y + h, x:x + w]#裁剪后的图像
        # 判断最大色块的颜色
        if np.count_nonzero(red_mask) > 10000:
            print(np.count_nonzero(red_mask))
            # 判断最大色块的形状
            CR = find_cir(cropped_frame)
            if CR > 30 and red_mask[y+h//2, x+w//2] == 0:
                communication(cmd_circle, 0, 0)
                print("圆形")
                return
            else:
                communication(cmd_rect, 0, 0)
                print("矩形")
                return
        else:
            # 进行二维码识别并输出结果
            color = Identify_QRcode()
            if color == "R":
                communication(cmd_rect, 0, 0)
                print("Rrect")
                return
            else:
                communication(cmd_null, 0, 0)
                print("null")
                return
    else:
        # 进行二维码识别并输出结果
        color = Identify_QRcode()
        if color == "R":
            communication(cmd_rect, 0, 0)
            print("Rrect")
            return
        else:
            communication(cmd_null, 0, 0)
            print("null")
            return


def find_blue(frame):
    frame = frame[y1:y1 + h1 , x1:x1 + w1] #roi1

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    ball_blue_lower = np.array([Blue_hsv[0], Blue_hsv[2], Blue_hsv[4]])
    ball_rad_upper = np.array([Blue_hsv[1], Blue_hsv[3], Blue_hsv[5]])

    blue_mask = cv2.inRange(hsv, ball_blue_lower, ball_rad_upper)  # 掩膜处理

    # 腐蚀和膨胀操作
    blue_mask = cv2.erode(blue_mask, None, iterations=2)
    blue_mask = cv2.dilate(blue_mask, None, iterations=2)

    # 查找轮廓
    contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    max_area = 0
    max_contour = None
    color = ''
    shape = ''
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > max_area:
            max_area = area
            max_contour = contour

    if max_contour is not None:
        # 计算最大轮廓的外接矩形
        x, y, w, h = cv2.boundingRect(max_contour)
        # 绘制外接矩形和中心点
        cv2.rectangle(hsv, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(hsv, (x + w // 2, y + h // 2), 5, (0, 255, 0), -1)
        cv2.imshow("mask", blue_mask)
        cropped_frame = frame[y:y + h, x:x + w]#裁剪后的图像
        #cropped_red_mask = red_mask[y:y + h, x:x + w]#裁剪后的图像
        # 判断最大色块的颜色
        if np.count_nonzero(blue_mask) > 10000:
            print(np.count_nonzero(blue_mask))
            # 判断最大色块的形状
            CR = find_cir(cropped_frame)
            if CR > 30 and blue_mask[y+h//2, x+w//2] == 0:
                communication(cmd_circle, 0, 0)
                print("圆形")
                return
            else:
                communication(cmd_rect, 0, 0)
                print("矩形")
                return
        else:
            # 进行二维码识别并输出结果
            color = Identify_QRcode()
            if color == "R":
                communication(cmd_rect, 0, 0)
                print("Rrect")
                return
            else:
                communication(cmd_null, 0, 0)
                print("null")
                return
    else:
        # 进行二维码识别并输出结果
        color = Identify_QRcode()
        if color == "R":
            communication(cmd_rect, 0, 0)
            print("Rrect")
            return
        else:
            communication(cmd_null, 0, 0)
            print("null")
            return




def find_red_and_blue(frame):
    frame = frame[y3:y3 + h3 , x3:x3 + w3]#roi3

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    ball_rad_lower = np.array([Rad_hsv[0], Rad_hsv[2], Rad_hsv[4]])
    ball_rad_upper = np.array([Rad_hsv[1], Rad_hsv[3], Rad_hsv[5]])

    ball_blue_lower = np.array([Blue_hsv[0], Blue_hsv[2], Blue_hsv[4]])
    ball_blue_upper = np.array([Blue_hsv[1], Blue_hsv[3], Blue_hsv[5]])

    red_mask = cv2.inRange(hsv, ball_rad_lower, ball_rad_upper)  # 掩膜处理
    blue_mask = cv2.inRange(hsv, ball_blue_lower, ball_blue_upper)  # 掩膜处理

    # 合并三个颜色的二值图像
    mask = cv2.bitwise_or(red_mask, blue_mask)

    # 腐蚀和膨胀操作
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    max_area = 0
    max_contour = None
    color = ''
    shape = ''
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > max_area:
            max_area = area
            max_contour = contour

    if max_contour is not None:
        # 计算最大轮廓的外接矩形
        x, y, w, h = cv2.boundingRect(max_contour)
        # 绘制外接矩形和中心点
        cv2.rectangle(hsv, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(hsv, (x + w // 2, y + h // 2), 5, (0, 255, 0), -1)
        cv2.imshow("mask", mask)

        # 判断最大色块的颜色
        if np.count_nonzero(red_mask) > 20000:
            print(np.count_nonzero(red_mask))
            print("red")
            communication(cmd_R, 0, 0)
            return

        elif np.count_nonzero(blue_mask) > 20000:
            print(np.count_nonzero(blue_mask))
            print("blue")
            communication(cmd_B, 0, 0)   
            return
        else:
            communication(cmd_null, 0, 0)
            print("null")
        return
    else:
        communication(cmd_null, 0, 0)
        print("null")
        return


            

            
cmd = 'k'
command = b'\xff'
if __name__ == "__main__":

    # 初始化摄像头并设置阙值
    usb_cap = cv2.VideoCapture(9)  # 主摄像头
    # 设置显示的分辨率，设置为640×480px
    usb_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    usb_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    #关闭自动白平衡
    usb_cap.set(cv2.CAP_PROP_AUTO_WB, 0)
    #关闭自动曝光
    usb_cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    ret, frame = usb_cap.read()
    show = 1

    while True:
        ret, frame = usb_cap.read()
        if ret:
            key = cv2.waitKey(1)

            if uart.in_waiting:  # 如果收到数据
                command = UartReceiveDate()
                print("收到命令:", command)

            elif (command == b'\x01') or (cmd == 'q'):  # 找球
                frame, color = find_blob(frame)
                print(color)

            elif (command == b'\x02') or (cmd == 'w'):  # 找红
                find_red(frame)
                command = b'\x05'

            elif (command == b'\x03') or (cmd == 'e'):  # 找蓝色
                find_blue(frame)
                command = b'\x05'
            elif (command == b'\x04') or (cmd == 'e'):  # 找蓝色
                find_red_and_blue(frame)
                command = b'\x05'   
            elif (command == b'\x05'):  # 暂停
                print("stop")
                delay_ms(1000)
            elif (command == b'\x06'):  # 终止
                usb_cap.release()
                cv2.destroyAllWindows()
                break
        cv2.imshow('My_puture', frame)

    # 释放摄像头并关闭所有窗口
    usb_cap.release()
    cv2.destroyAllWindows()

