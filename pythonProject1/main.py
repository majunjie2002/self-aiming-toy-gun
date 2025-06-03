import cv2
import serial
#import pyserial

global kp
global ki
global kd
global last_err
global err_i
global err_i_MAX
global output_MAX
global speed
global trackflag

kp=0.03
ki=0 #0.001
kd=0
last_err=0
err_i=0
err_i_MAX=0 #5000
output_MAX=10
speed=[0xEE,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
trackflag=False

global isTracking
global bbox
global img2

def on_mouse(event, x, y, flags, param):
    global img2, point1, point2, g_rect, isTracking
    if event == cv2.EVENT_LBUTTONDOWN:  # 左键点击,则在原图打点
        point1 = (x, y)
        #cv2.circle(img2, point1, 10, (255, 255, 255), 2)
        cv2.imshow('input image', img2)

    elif event == cv2.EVENT_MOUSEMOVE and (flags & cv2.EVENT_FLAG_LBUTTON):  # 按住左键拖曳，画框
        cv2.rectangle(img2, point1, (x, y), (255, 255, 255), thickness=2)
        cv2.imshow('input image', img2)

    elif event == cv2.EVENT_LBUTTONUP:  # 左键释放，显示
        point2 = (x, y)
        cv2.rectangle(img2, point1, point2, (255, 255, 255), thickness=2)
        cv2.imshow('input image', img2)
        if point1 != point2:
            min_x = min(point1[0], point2[0])
            min_y = min(point1[1], point2[1])
            width = abs(point1[0] - point2[0])
            height = abs(point1[1] - point2[1])

            # 定义一个初始边界框
            bbox = (min_x, min_y, width, height)
            # 用第一帧和包围框初始化跟踪器
            tracker.init(frame, bbox)
            isTracking=True

def track_function(frame):
    global img2, bbox, isTracking
    cv2.setMouseCallback('input image', on_mouse)
    img2 = frame
    if isTracking:
        ok, bbox = tracker.update(frame)
        if ok:
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255, 255, 255), 2, 1)
            print(str(bbox[0]+bbox[2]/2)+" "+str(bbox[1]+bbox[3]/2))
            #print(str(get_PID_speed(bbox[0]+bbox[2]/2-320))+" "+str(get_PID_speed(bbox[1]+bbox[3]/2-250)))
            if trackflag:
                if get_PID_speed(bbox[0]+bbox[2]/2 - 320) > 0:
                    speed[1] = 0
                    speed[2] = int(get_PID_speed(bbox[0]+bbox[2]/2 - 320))
                else:
                    speed[1] = 1
                    speed[2] = int(-get_PID_speed(bbox[0]+bbox[2]/2 - 320))
                if get_PID_speed(bbox[1]+bbox[3]/2 - 250) > 0:
                    speed[3] = 0
                    speed[4] = int(get_PID_speed(bbox[1]+bbox[3]/2 - 250))
                else:
                    speed[3] = 1
                    speed[4] = int(-get_PID_speed(bbox[1]+bbox[3]/2 - 250))
    cv2.line(frame, (280, 250), (360, 250), (255, 255, 255), 2)
    cv2.line(frame, (320, 210), (320, 290), (255, 255, 255), 2)
    cv2.imshow("input image", frame)

def face_detect_function(img):
    global speed

    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    face_detect = cv2.CascadeClassifier('haarcascade_frontalface_alt2.xml')
    face = face_detect.detectMultiScale(gray_img)
    for x, y, w, h in face:
        cv2.rectangle(img, (x, y, w, h), color=(255, 255, 255), thickness=2)
        print(str(x+w/2)+" "+str(y+h/2))
        #print(str(get_PID_speed(x+w/2-320))+" "+str(get_PID_speed(y+h/2-250)))
        if trackflag:
            if get_PID_speed(x+w/2-340)>0:
                speed[1]=0
                speed[2]=int(get_PID_speed(x+w/2-340))
            else:
                speed[1]=1
                speed[2]=int(-get_PID_speed(x+w/2-340))
            if get_PID_speed(y+h/2-230)>0:
                speed[3]=0
                speed[4]=int(get_PID_speed(y+h/2-230))
            else:
                speed[3]=1
                speed[4]=int(-get_PID_speed(y+h/2-230))
            '''
            speed[5] = 1
            if x+w/2-340<40 and y+h/2-230<40:
                speed[6] = 1
            '''

    cv2.line(img, (300, 230), (380, 230), (255, 255, 255), 2)
    cv2.line(img, (340, 190), (340, 270), (255, 255, 255), 2)
    cv2.imshow("input image", img)

def get_PID_speed(err):
    global kp
    global ki
    global kd
    global last_err
    global err_i
    global err_i_MAX
    global output_MAX

    err_i=err+err_i
    if err_i>err_i_MAX:
        err_i=err_i_MAX
    if err_i<-err_i_MAX:
        err_i=-err_i_MAX
    output=kp*err+ki*err_i+kd*(err-last_err)
    last_err = err
    if output>output_MAX:
        output=output_MAX
    if output<-output_MAX:
        output=-output_MAX
    return output

video = cv2.VideoCapture(1)
cv2.namedWindow("input image", 0)
cv2.resizeWindow("input image", 1200,900)
tracker = cv2.TrackerKCF_create()
isTracking=False

ser = serial.Serial(port='COM6', baudrate=115200, bytesize=8, stopbits=1,timeout=0.5)
if ser.isOpen():
    print("打开串口成功, 串口号: %s" % ser.name)
else:
    print("打开串口失败")

while True:
    flag, frame = video.read()
    if not flag:
        break
    '''
    speed[2] = 0
    speed[4] = 0
    speed[5] = 0
    speed[6] = 0
    '''
    face_detect_function(frame)
    #track_function(frame)

    pressedKey = cv2.waitKey(1) & 0xFF
    if pressedKey == ord('w'):
        speed[3]=1
        speed[4]=2
    elif pressedKey == ord('s'):
        speed[3] = 0
        speed[4] = 2
    elif pressedKey == ord('a'):
        speed[1] = 1
        speed[2] = 2
    elif pressedKey == ord('d'):
        speed[1] = 0
        speed[2] = 2
    elif pressedKey == ord('k'):
        speed[2] = 0
        speed[4] = 0
        speed[5] = 0
        speed[6] = 0
        trackflag=False
    elif pressedKey == ord('l'):
        err_i=0
        trackflag=True
    elif pressedKey == ord('j'):
        speed[5] = 1
    elif pressedKey == ord('i'):
        speed[6] = 1
    elif pressedKey == ord('q'):
        break

    ser.write(speed)

cv2.destroyAllWindows()
video.release()