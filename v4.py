import cv2
import numpy as np
from pyzbar.pyzbar import decode
from pyzbar import pyzbar
import serial  # ��װpyserial����import serial���Ҳ��ܰ�װserial
import time

Rad_hsv = [153, 203, 63, 177, 87, 221]
# Rad_hsv = [161, 197, 50, 205, 146, 224]  # lowerbH,upperbH,lowerbS,upperbS,lowerbV,upperbV
Yellow_hsv = [13, 36, 96, 186, 152, 255]  # lowerbH,upperbH,lowerbS,upperbS,lowerbV,upperbV
Blue_hsv = [93, 129, 251, 255, 60, 201]  # lowerbH,upperbH,lowerbS,upperbS,lowerbV,upperbV
White_hsv = [79, 100, 0, 66, 202, 255]

#roi�趨################################################################################# 
x1, y1, w1, h1 = 10, 0, 620, 320 #roi1���Ͻ�����Ϳ�ȡ��߶�roi = image[y:y+h, x:x+w]
x2, y2, w2, h2 = 0, 0, 640, 280 #roi2
x3, y3, w3, h3 = 0, 0, 640, 480 #roi3

while True:
    try:
        # �������Ӵ���
        uart = serial.Serial("/dev/ttyS3", 115200, 8, 'N', 1)  # �������Ӵ���/dev/ttys3
        # ������ӳɹ����˳�ѭ��
        break
    except serial.SerialException:
        # �������ʧ�ܣ���ӡ������Ϣ
        print("δ�ҵ����õĴ��ڻ��޷��򿪴���")
        # �ȴ�һ��ʱ����ٴγ�������


def delay_ms(TI):
    time_ms = TI / 1000
    time.sleep(time_ms)


# �����
# ³��è���ͣ�
cmd_rect = b'\x21'  # ���飨��������ͬ��
cmd_circle = b'\x22'  # Բ������������ͬ��
cmd_null = b'\x26'    #��ʶ��Ŀ�꣨��������ͬ��
cmd_R = b'\x23'  # ����ת�̣�
cmd_B = b'\x24'  # ����ת�̣�
cmd_Y = b'\x25'  # ����ת�̣�
############################################################
# ³��è����
b'\x01'  # ת������         
b'\x02'  # ��ɫ��ʶ�����ƽ̨
b'\x03'  # ��ɫ��ʶ�����ƽ̨
b'\x04'  # ʶ����̨
b'\x05'  # ��ͣ
b'\x06'  # ��ֹ

def Int_translation(number):  # 10����ת����Ϊ����16����
    decimal_number = number
    x1 = int(decimal_number / 256)
    y1 = int(decimal_number % 256)
    return x1, y1


data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
x = 0


def UartReceiveDate():  # ���գ����������������̫�죬����ᵼ�´��ڶ�ȡ̫�쵼�³���
    global Find_Task
    global Target_Num
    global x
    global data
    x = 0
    # uart_num = uart.in_waiting
    if uart.in_waiting:  # ����յ�����
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


def communication(com, data1, data2):  # ����
    uart.write(bytearray([0xA5]))  # ����һ��ʮ����������ͷ
    uart.write(bytearray([0x02]))  # ���ݳ���
    uart.write(com)  # �������
    uart.write(bytearray([data1]))  # ��λ����
    uart.write(bytearray([data2]))  # ��λ����
    uart.write(bytearray([0x5A]))  # ����һ��ʮ����������β


def find_cir(cropped_frame):
    CX = -1
    CY = -1
    CR = -1

    
    #1��ת��Ϊ�Ҷ�ͼ��ͼ��ƽ��
    gray = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2GRAY)
    dst = cv2.medianBlur(gray, 5)
    cv2.imshow("�˲�", dst)
    #canny��Եʶ��
    canny = cv2.Canny(dst, 50, 150)
    cv2.imshow("canny", canny)

    circle_r = []

    # ������ԲȦ���
    # circles = cv2.HoughCircles(image,�� image������ͼ�񣬼�Դͼ������Ϊ8λ�ĵ�ͨ���Ҷ�ͼ��
    #                            method, �� method����ⷽ����
    #                            dp,     �� dp���ۼ����ֱ��ʣ�����һ���ָ���ʣ�����ָ��ͼ��ֱ�����Բ���ۼ����ֱ��ʵı��������磬���dp = 1��������ͼ����ۼ���������ͬ�ķֱ��ʡ�
    #                            minDist,�� minDist��Բ�ļ����С��ࡣ��ֵ����Ϊ��ֵʹ�ã��������Բ�ļ����С�ڸ�ֵ�Ķ��Բ�������һ���ᱻ��������
    #                                       ��ˣ������ֵ̫С������ж���ٽ���Բ���������������ֵ̫������ܻ��ڼ��ʱ©��һЩԲ��
    #                            param1, �� param1���ò�����ȱʡ�ģ���ȱʡʱĬ��ֵΪ100������Ӧ����Canny��Ե������ĸ���ֵ������ֵ�Ǹ���ֵ�Ķ���֮һ����
    #                            param2, �� param2��Բ��λ�ñ����յ���ͶƱ����ֻ���ڵ�1��ɸѡ�����У�ͶƱ��������ֵ��Բ�������ʸ�����2�ֵ�ɸѡ��
    #                                           ��ˣ���ֵԽ�󣬼�⵽��ԲԽ�٣���ֵԽС����⵽��ԲԽ�ࡣ���������ȱʡ�ģ���ȱʡʱ����Ĭ��ֵ100��
    #                            minRadius,�� minRadius��Բ�뾶����Сֵ��С�ڸ�ֵ��Բ���ᱻ���������ò�����ȱʡ�ģ���ȱʡʱ����Ĭ��ֵ0����ʱ�ò����������á�
    #                            maxRadius) �� maxRadius��Բ�뾶�����ֵ�����ڸ�ֵ��Բ���ᱻ���������ò�����ȱʡ�ģ���ȱʡʱ����Ĭ��ֵ0����ʱ�ò����������á�
    # �� circles������ֵ����Բ������Ͱ뾶���ɵ�numpy.ndarray��

    circles = cv2.HoughCircles(dst, cv2.HOUGH_GRADIENT, 1, 1200, param1=100, param2=30, minRadius=30,
                               maxRadius=150)
    if circles is not None and circles.all():
        circles = np.uint16(np.around(circles))
        # ����
        for circle in circles[0, :]:
            # if ((circle[2]>150)and (circle[2]<190)):
            # ����Բ��
            cv2.circle(cropped_frame, (circle[0], circle[1]), 3, (0, 255, 0), -1)
            # ����Բ�ı߽�
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
    #ѭ��3��
    for i in range(10):#ʶ�𼸴�
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

    red_mask = cv2.inRange(hsv, ball_rad_lower, ball_rad_upper)  # ��Ĥ����
    blue_mask = cv2.inRange(hsv, ball_blue_lower, ball_blue_upper)  # ��Ĥ����
    yellow_mask = cv2.inRange(hsv, ball_yellow_lower, ball_yellow_upper)  # ��Ĥ����

    # �ϲ�������ɫ�Ķ�ֵͼ��
    mask = cv2.bitwise_or(red_mask, cv2.bitwise_or(yellow_mask, blue_mask))

    # ��ʴ�����Ͳ���
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # ��������
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
        # ���������������Ӿ���
        x, y, w, h = cv2.boundingRect(max_contour)
        # ������Ӿ��κ����ĵ�
        cv2.rectangle(hsv, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(hsv, (x + w // 2, y + h // 2), 5, (0, 255, 0), -1)
        cv2.imshow("mask", mask)

        # �ж����ɫ�����ɫ
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

        # �ڴ������Ͻ���ʾ��ɫ
        cv2.putText(hsv, color, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    return hsv, color


def find_red(frame):
    frame = frame[y1:y1 + h1 , x1:x1 + w1] #roi1

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    ball_rad_lower = np.array([Rad_hsv[0], Rad_hsv[2], Rad_hsv[4]])
    ball_rad_upper = np.array([Rad_hsv[1], Rad_hsv[3], Rad_hsv[5]])

    red_mask = cv2.inRange(hsv, ball_rad_lower, ball_rad_upper)  # ��Ĥ����

    # ��ʴ�����Ͳ���
    red_mask = cv2.erode(red_mask, None, iterations=2)
    red_mask = cv2.dilate(red_mask, None, iterations=2)

    # ��������
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
        # ���������������Ӿ���
        x, y, w, h = cv2.boundingRect(max_contour)
        # ������Ӿ��κ����ĵ�
        cv2.rectangle(hsv, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(hsv, (x + w // 2, y + h // 2), 5, (0, 255, 0), -1)
        cv2.imshow("mask", red_mask)
        cropped_frame = frame[y:y + h, x:x + w]#�ü����ͼ��
        #cropped_red_mask = red_mask[y:y + h, x:x + w]#�ü����ͼ��
        # �ж����ɫ�����ɫ
        if np.count_nonzero(red_mask) > 10000:
            print(np.count_nonzero(red_mask))
            # �ж����ɫ�����״
            CR = find_cir(cropped_frame)
            if CR > 30 and red_mask[y+h//2, x+w//2] == 0:
                communication(cmd_circle, 0, 0)
                print("Բ��")
                return
            else:
                communication(cmd_rect, 0, 0)
                print("����")
                return
        else:
            # ���ж�ά��ʶ��������
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
        # ���ж�ά��ʶ��������
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

    blue_mask = cv2.inRange(hsv, ball_blue_lower, ball_rad_upper)  # ��Ĥ����

    # ��ʴ�����Ͳ���
    blue_mask = cv2.erode(blue_mask, None, iterations=2)
    blue_mask = cv2.dilate(blue_mask, None, iterations=2)

    # ��������
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
        # ���������������Ӿ���
        x, y, w, h = cv2.boundingRect(max_contour)
        # ������Ӿ��κ����ĵ�
        cv2.rectangle(hsv, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(hsv, (x + w // 2, y + h // 2), 5, (0, 255, 0), -1)
        cv2.imshow("mask", blue_mask)
        cropped_frame = frame[y:y + h, x:x + w]#�ü����ͼ��
        #cropped_red_mask = red_mask[y:y + h, x:x + w]#�ü����ͼ��
        # �ж����ɫ�����ɫ
        if np.count_nonzero(blue_mask) > 10000:
            print(np.count_nonzero(blue_mask))
            # �ж����ɫ�����״
            CR = find_cir(cropped_frame)
            if CR > 30 and blue_mask[y+h//2, x+w//2] == 0:
                communication(cmd_circle, 0, 0)
                print("Բ��")
                return
            else:
                communication(cmd_rect, 0, 0)
                print("����")
                return
        else:
            # ���ж�ά��ʶ��������
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
        # ���ж�ά��ʶ��������
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

    red_mask = cv2.inRange(hsv, ball_rad_lower, ball_rad_upper)  # ��Ĥ����
    blue_mask = cv2.inRange(hsv, ball_blue_lower, ball_blue_upper)  # ��Ĥ����

    # �ϲ�������ɫ�Ķ�ֵͼ��
    mask = cv2.bitwise_or(red_mask, blue_mask)

    # ��ʴ�����Ͳ���
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # ��������
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
        # ���������������Ӿ���
        x, y, w, h = cv2.boundingRect(max_contour)
        # ������Ӿ��κ����ĵ�
        cv2.rectangle(hsv, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(hsv, (x + w // 2, y + h // 2), 5, (0, 255, 0), -1)
        cv2.imshow("mask", mask)

        # �ж����ɫ�����ɫ
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

    # ��ʼ������ͷ��������ֵ
    usb_cap = cv2.VideoCapture(9)  # ������ͷ
    # ������ʾ�ķֱ��ʣ�����Ϊ640��480px
    usb_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    usb_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    #�ر��Զ���ƽ��
    usb_cap.set(cv2.CAP_PROP_AUTO_WB, 0)
    #�ر��Զ��ع�
    usb_cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    ret, frame = usb_cap.read()
    show = 1

    while True:
        ret, frame = usb_cap.read()
        if ret:
            key = cv2.waitKey(1)

            if uart.in_waiting:  # ����յ�����
                command = UartReceiveDate()
                print("�յ�����:", command)

            elif (command == b'\x01') or (cmd == 'q'):  # ����
                frame, color = find_blob(frame)
                print(color)

            elif (command == b'\x02') or (cmd == 'w'):  # �Һ�
                find_red(frame)
                command = b'\x05'

            elif (command == b'\x03') or (cmd == 'e'):  # ����ɫ
                find_blue(frame)
                command = b'\x05'
            elif (command == b'\x04') or (cmd == 'e'):  # ����ɫ
                find_red_and_blue(frame)
                command = b'\x05'   
            elif (command == b'\x05'):  # ��ͣ
                print("stop")
                delay_ms(1000)
            elif (command == b'\x06'):  # ��ֹ
                usb_cap.release()
                cv2.destroyAllWindows()
                break
        cv2.imshow('My_puture', frame)

    # �ͷ�����ͷ���ر����д���
    usb_cap.release()
    cv2.destroyAllWindows()

