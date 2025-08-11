# 라이브러리 import
import cv2
import cv2.aruco as aruco
from picamera2 import Picamera2
import numpy as np
import threading
import pigpio
import time

# GPIO pin set up
camera_degree_pin = 18
steeringPin = 19
motorOut1Pin = 23
motorOut2Pin = 24
motorOut3Pin = 27
motorOut4Pin = 22
ultrasonicSensorTrigPin = 26
ultrasonicSensorEchoPin = 6
ledPin = 5

# 전역변수 선언
system_running = True
target_ids = [1,2,3,4]
current_target_index = 0
current_target_id = target_ids[current_target_index]
idState = None
marker_length = 0.15 
motor_lock = False
force = False


pi = pigpio.pi()   # pigpio 객체 생성 

# GPIO pin IN/OUT set up
pi.set_mode(motorOut1Pin, pigpio.OUTPUT)
pi.set_mode(motorOut2Pin, pigpio.OUTPUT)
pi.set_mode(ultrasonicSensorTrigPin, pigpio.OUTPUT)
pi.set_mode(ultrasonicSensorEchoPin, pigpio.INPUT)
pi.set_mode(ledPin, pigpio.OUTPUT)

# picamera2 객체 생성 및 기초 set up, 해상도 설정
picam2 = Picamera2()
picam2.preview_configuration.main.size = (800, 600)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# 4x4(16bit)의 크기의 Aruco 마커가 담긴 딕셔너리를 담은 객체 선언
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50) 
 

camera_matrix = np.array([[600, 0, 400], [0, 600, 300], [0, 0, 1]], dtype=np.float32) # 초점 거리, 중심좌표 생성
dist_coeffs = np.zeros((5, 1))  # 카메라 왜곡 없음 설정

# 시스템 종료시 GPIO를 LOW로 설정
def shutdown_gpio():
    pi.write(motorOut1Pin, 0)
    pi.write(motorOut2Pin, 0)
    pi.write(motorOut3Pin, 0)
    pi.write(motorOut4Pin, 0)
    pi.write(camera_degree_pin, 0)
    pi.write(steeringPin, 0)
    pi.write(ultrasonicSensorTrigPin, 0)
    pi.write(ultrasonicSensorEchoPin, 0)
    pi.write(ledPin, 0)
    pi.set_servo_pulsewidth(camera_degree_pin, 0)
    pi.set_servo_pulsewidth(steeringPin, 0)
    motorEnd()

# survo 모터를 카메라 각도별로 조절하기 위한 함수 (500 -> 0도 , 2500 -> 180도)
def angle_to_pulse(angle):
    return int(500 + (angle / 180.0) * 2000)


# 모터를 가동하는 함수 
def motorStart():
    if motor_lock == False or force == True:
        pi.write(motorOut1Pin, 1)
        pi.write(motorOut2Pin, 1)
        pi.write(motorOut3Pin, 1)
        pi.write(motorOut4Pin, 1)
    else:
        return 

# 모터를 정지하는 함수
def motorEnd():
     if motor_lock == False or force == True:
        pi.write(motorOut1Pin, 0)
        pi.write(motorOut2Pin, 0)
        pi.write(motorOut3Pin, 0)
        pi.write(motorOut4Pin, 0)
     else:
         return
     
# 차량의 방향을 조절하는 함수, angle_to_pulse 에서 return한 값을 바탕으로 pwm신호를 survo 모터에 전송
def set_steering(degree):
    if 60 <= degree <= 120:
        pulse = angle_to_pulse(degree)
        pi.set_servo_pulsewidth(steeringPin, pulse)
    

# 초음파 센서로 거리를 측정한 다음 값을 return
def get_distance():
    pi.write(ultrasonicSensorTrigPin, 0)  # 초기 Trigger를 LOW로 설정
    time.sleep(0.002)
    pi.write(ultrasonicSensorTrigPin, 1) # Trigger를 10us 동안 HIGH로 설정 
    time.sleep(0.00001)
    pi.write(ultrasonicSensorTrigPin, 0) # 다시 LOW로 설정 함으로서 센서를 가동

    start = time.time()
    timeout = start + 1
    while pi.read(ultrasonicSensorEchoPin) == 0 and start < timeout: # Echo가 HIGH가 될 때 까지 대기 
        start = time.time()

    end = time.time()
    timeout = end + 1
    while pi.read(ultrasonicSensorEchoPin) == 1 and end < timeout: # Echo가 LOW가 될 때 까지 대기 
        end = time.time()

    pulse_duration = end - start # Echo 가 HIGH에서 LOW가 될 때까지의 시간 값(초음파가 발생하여 다시 수신될 때 까지의 시간)
    distance = pulse_duration * 17150 # 거리 = 시간(pulse_duration) * 속도(34300 cm/s(음속)) / 2 
    distance = round(distance, 2) 

    return distance


# 마커를 못 찾을시 마커 탐색 함수
def detectMarker():
    global idState
    while idState is None:
        turnLeft = 60
        set_steering(turnLeft)
        motorStart()
        time.sleep(0.2)
        motorEnd()
        time.sleep(0.1)
    
    
# 마커로 부터 x좌표 값을 받아 차량이 움직여야 할 각도를 계산하여 반환하는 함수
def vehicleRotation(horizon, sensitivity):
    if abs(horizon) < 25: # 25mm까지의 거리 차이는 무시
        return 90
    
    rotation = horizon * sensitivity # 거리 차이당 각도 계산(10mm -> 1도)
    angle = 90 + rotation # 90도가 중앙임으로 90도를 기준으로 각도를 계산

    angle = max(60,min(120,angle)) # 60~120도 까지가 안정적임으로 최대, 최소값을 설정
    
    return int(angle)


# 초음파센서가 작동할시 모터를 정지하는 기능의 함수
def object_checking(range):
    global motor_lock
    while system_running:
        dist = get_distance()
        while dist < range:
            motorEnd()
            motor_lock = True
            pi.write(ledPin, 1)
            print("Object Detect!! please take action!!")
            dist = get_distance()
            if dist >= range:
                pi.write(ledPin, 0)
                motor_lock = False
                break


# 카메라 작동 함수
def camera_loop():

    global idState
    marker_id = 0

    while system_running:
    
        frame = picam2.capture_array() # 카메라가 캡쳐한 이미지를 변수에 저장

        corners, ids, _ = aruco.detectMarkers(frame, aruco_dict) # 이미지에서 아루코 마커를 탐색 해서 값을 찾을시 각 변수에 값을 저장

        if ids is not None:
        
            aruco.drawDetectedMarkers(frame, corners, ids) # 인식한 마커를 시각적으로 표시
       
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs) # 설정한 값 들을 기준으로 마커의 회전 벡터, 이동 벡터를 각 변수에 배열로 저장

            for i in range(len(ids)):
            
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.03) # 각 마커에 회전 벡터와 이동 벡터의 표시를 시각화함

                marker_id = ids[i][0] 

                if marker_id == current_target_id:

                    x, y, z = tvecs[i][0]
                    
                    x_mm = x * 1000
                    y_mm = y * 1000
                    z_mm = z * 1000

                    # 마커이미지에 id,거리를 수치로 표시
                    coords_text = f"ID {ids[i][0]} Pos: (x={x_mm:.2f} y={y_mm:.2f} z={z_mm:.2f})mm"
                    cv2.putText(frame, coords_text, tuple(corners[i][0][0].astype(int)),  
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                    R_mat, _ = cv2.Rodrigues(rvecs[i])

                    text_pos = corners[i][0][1].astype(int)
                    text_pos[1] += 20
                    
                    # 마커이미지에 각도를 수치로 표시
                    yaw = np.arctan2(R_mat[1,0], R_mat[0,0]) * 180 / np.pi
                    pitch = np.arctan2(-R_mat[2,0], np.sqrt(R_mat[2,1]**2 + R_mat[2,2]**2)) * 180 / np.pi
                    roll = np.arctan2(R_mat[2,1], R_mat[2,2]) * 180 / np.pi
                    angles_text = f"(Yaw={yaw:.1f} Pitch={pitch:.1f} Roll={roll:.1f})deg"
                    cv2.putText(frame, angles_text, tuple(text_pos), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1, cv2.LINE_AA)
                    
                    # idState에 현재 마커의 정보를 저장
                    idState = {'id':marker_id , 'x': x_mm, 'y':y_mm , 'z':z_mm , 'yaw':yaw ,'pitch': pitch , 'roll': roll}
                

        else:
            idState = None # 마커를 인식하지 못 할시 idState에 None을 저장
        
        cv2.imshow("ArUco Detection", frame) # 이미지를 모니터에서 확인
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            motorEnd()
            break

    picam2.close()
    cv2.destroyAllWindows()

# 카메라에서 얻은 데이터를 기반으로 모터를 제어하는 함수
def operate_systemDevice():

    global idState, current_target_id ,current_target_index 

    set_steering(90) # 초기 정면(90도)을 세팅
    time.sleep(1)

    motorEnd()

    while system_running:
        if idState is not None:

            z_mm = idState['z']
            x_mm = idState['x']

            print(f"Moving.... -> Target:{idState['id']}  Distance={z_mm:.2f}mm  horizon={x_mm:.2f}mm")
            
            # 마커의 거리에 따른 방향 조절
            rotationAngle=vehicleRotation(x_mm, 0.1)
            set_steering(rotationAngle) 
            motorStart()
            
            # 마커에 일정 거리 이상 가까워질시 다음 마커로 목표를 바꿈
            if z_mm <= 380:
                print(f"Success!!! Target:{idState['id']}")

                motorEnd()
                time.sleep(2)

                if current_target_index < len(target_ids) - 1:
                    current_target_index += 1
                    current_target_id = target_ids[current_target_index]
                    idState = None  
                else:
                    current_target_index = 0
                    current_target_id = target_ids[current_target_index]
                    idState = None 
                
                
        
        else:
            # 마커를 인식 하지 못할 시 마커를 찾기 위해 회전
            print("Aruco Marker Not Find!!!")
            print("Target Reserching.....")
            motorEnd()
            time.sleep(0.25)
            if idState is not None:
                continue
            detectMarker()
    

# 카메라 , 목표추적 및 모터 , 초음파 부분을 3개의 스레드로 구성
camera_thread = threading.Thread(target= camera_loop)
camera_thread.daemon = True

operate_systemDevice_thread = threading.Thread(target=operate_systemDevice)
operate_systemDevice_thread.daemon = True


object_thread = threading.Thread(target= object_checking , args=(20,))
object_thread.daemon = True


# 스레드들을 실행하고 종료시 GPIO를 종료하고 권한을 반환
try:
    
    camera_thread.start()
    operate_systemDevice_thread.start()
    object_thread.start()

    camera_thread.join()
    operate_systemDevice_thread.join()
    object_thread.join()


except KeyboardInterrupt:
    print("System Shutdown")

finally:
    system_running = False
    force = True
    time.sleep(1)
    shutdown_gpio()
    pi.stop()

