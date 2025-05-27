RPI=True

import RPi.GPIO as GPIO


from time import sleep
from time import time


class RocketProtocol:
    def __init__(self):
        ##initial
        self.mSeperationServoPin = 33 ##change

        self.m2ndServoPin = 32

        self.mIgnitionRelayPin = 15 ##change
        #각 변수에 값 할당. 밑 코드에서 GPIO.BOARD형식으로 PIN 할당

        GPIO.setmode(GPIO.BOARD) #GPIO.BOARD방식으로 PIN번호 할당
        GPIO.setup(self.mSeperationServoPin,GPIO.OUT)
        self.mSeperationServo = GPIO.PWM(self.mSeperationServoPin, 50) #GPIO.PWM(pin, frequency): mSeperationServo 변수는 mSeperationServoPin에 할당된 pin번호로 50Hz PWM신호를 출력해주는 기능
        GPIO.setup(self.m2ndServoPin,GPIO.OUT)
        self.m2ndServo = GPIO.PWM(self.m2ndServoPin, 50) #GPIO.PWM(pin, frequency)
    
        GPIO.setup(self.mIgnitionRelayPin, GPIO.OUT)
        GPIO.output(self.mIgnitionRelayPin,False) #GPIO 출력핀에 0V를 내보낸다.(False) True일 때는 핀 최고 전압을 출력(3.3V, 5V)
    
        self.mSeperationServo.start(0)
        self.m2ndServo.start(0) #Duty cycle 0으로 시작, 서보모터의 초기값을 0으로 설정

        self.SERVO_MAX_DUTY = 12.5 # servo Max
        self.SERVO_MIN_DUTY = 2.5 #servo min

        self.RocketStep=0
        self.RocketMaxStep=4

        # 이그나이터 상태, 단분리 상태, 1단 2단 서보 상태
        self.IsIgnition=False
        self.IsSeperation=True
        self.Is1stServo=False
        self.Is2stServo=True

        self.Is1stAccel=False
        self.Is1stAccelThreshold=10 #m/s

        self.Is2stAccel=True
        self.Is2stAccelThreshold=10 #m/s

    def set2ndServoPos(self,degree): # 각도를 입력하면 duty비를 알아서 설정해주는 함수
        if degree > 180:
            degree = 180
        GPIO.setup(self.m2ndServoPin,GPIO.OUT)
        duty =self.SERVO_MIN_DUTY+(degree*(self.SERVO_MAX_DUTY-self.SERVO_MIN_DUTY)/180.0) # 서보 모터 관련 수식(서보 관련 기본 지식 참고) 모터, MCU 스펙에 따라 함수 달라짐. 각도를 듀티비로 환산하는 식
        self.m2ndServo.ChangeDutyCycle(duty) # 구한 값을 다른 함수에 넣어라. 환산한 Duty비를 서보에 전달
        sleep(0.5) # 0.5sec 대기
        GPIO.setup(self.m2ndServoPin,GPIO.IN) # 서보 제어 끝난 뒤 PIN을 입력모드로 전환. 하중이 걸리지 않는 상태에서는 서보 회전 후 입력모드로 전환하면 서보 발열&전력소모 최소화 가능. 단, 서보에 하중이 걸리는 상황이라면 output모드로 유지할 필요 있음.

    def set2ndServoBoolean(self,booldata): # 서보 모터 키고 끄는 프로토콜, 서보모터 초기 각도 세팅값 만질때 수정. 처음 실행할 때는 무조건 true이므로 70도로 고정됨. 상황에 따라 self.Is2stServo=False가 되면 105도로 각도가 변경됨.
        self.Is2stServo=booldata
        if self.Is2stServo:
            self.set2ndServoPos(70); #true 일때 degree=70
        else:
            self.set2ndServoPos(105); #false 일때 degree=105

    def setSeperationServoPos(self,degree):
        if degree > 180:
            degree = 180
        GPIO.setup(self.mSeperationServoPin,GPIO.OUT)
        duty =self.SERVO_MIN_DUTY+(degree*(self.SERVO_MAX_DUTY-self.SERVO_MIN_DUTY)/180.0)
        self.mSeperationServo.ChangeDutyCycle(duty)
        sleep(0.5)
        GPIO.setup(self.mSeperationServoPin,GPIO.IN)

    def setSeperationServoBoolean(self,booldata): # set2ndServoBoolean과 똑같음. 서보 각도 세팅값을 조절할 수 있음(90, 65 값을 상황에 맞게 바꿈)
        self.IsSeperation=booldata
        if self.IsSeperation:
            self.setSeperationServoPos(90);
        else:
            self.setSeperationServoPos(65);
    
    def setIgnition(self,booldata):
        self.IsIgnition=booldata
        if booldata: # true일 때
             GPIO.setup(self.mIgnitionRelayPin, GPIO.OUT)  # mIgnitionRelayPin PIN단자를 output모드로 전환
             GPIO.output(self.mIgnitionRelayPin,self.IsIgnition) # mIgnitionRelayPin PIN단자를 output모드로 전환 후 True값을 보내 PIN에 최대 전압을 인가(line 27 code 참고)
        else: # IsIgnition 값이 False로 변했을 때
             GPIO.setup(self.mIgnitionRelayPin, GPIO.OUT)
             GPIO.output(self.mIgnitionRelayPin,self.IsIgnition)
             #GPIO.setup(self.mIgnitionRelayPin, GPIO.IN)

    
    def Cleanup(self): # 라즈베리파이의 모든 GPIO 핀 설정을 초기화하는 함수(GPIO.setup, GPIO.PWM 등등 GPIO로 설정했던 핀들을 모두 해제해줌)
        if RPI: # line 1에 true 값으로 저장되어있음.
            GPIO.cleanup()


    def Algorithm1Check(self,data):
        # 1차 점화 완료 체크
        # 가속도 Global z값 확인
        if self.Is1stAccel: # 초기값: false
            self.previous_value = self.current_value
            self.current_value = data[5]
            
            if self.increasing: # 따로 초기값 없음. 다른 code에서 self.increase=True 초기값 설정 후 함수 불러올 듯. 1단부 가속도가 +값일 때인 듯.
                if self.current_value < self.previous_value: # 1단부 가속도의 나중값이 처음 값보다 작아질 때(가속도가 +이지만 값이 감소했을 때)
                    self.increasing = False
                return False
            else: # self.increasing 값이 False일 때
                if self.current_value < self.Is1stAccelThreshold: # 1단부 가속도가 10m/s보다 작아질 때
                    self.RocketStep+=1 # Rocket stage를 1단계 상승시킴
                    if RPI:
                        self.setSeperationServoBoolean(True) # setSeperationServoBoolean 함수에 True값을 입력하여 함수값 출력시킴(line 75 참고해서 어떻게 작동하는 지 확인)
                    print("Rocket1")
                    self.increasing = False # 값을 False로 유지시켜 단 분리 이후에도 코드 사용할 수 있게 함.
                    self.Rocket1time=time();
                    return True # return 값을 true로 반환해 알고리즘을 계속 사용할 때 line 110 if문을 계속 사용할 수 있게 함.
                else:
                    return False # return 값을 false로 반환해 알고리즘 사용 시 line 110 if문 사용할 수 없게 됨.

        if abs(data[5])>=self.Is1stAccelThreshold: # threshold 확인
            self.Is1stAccel=True
            self.increasing=True
        return False    
        
    def Algorithm2Check(self,data):
        # 2차 점화전 시작 체크
        # 속도 Global z값 확인(1m/s 이상 True)
        # 각속도 Global x,y값 확인
        self.Rocket2time=time()-self.Rocket1time
        if self.Rocket2time>=0.5:
            if ((data[1]*data[1])+(data[2]*data[2]))**(1/2)<=20: #각도 체크
                self.RocketStep+=1
                print("Rocket2")
                return 2
            else:
                self.RocketStep=4
                return 1
        else:
            return 0    
      
    def Algorithm3Check(self,data):
        # 2차 점화후 완료 체크
        # 알고리즘2 체크후 2초동안 확인, 2초 지나면 점화 중지
        # 가속도 Global z값 확인(가속도가 높아지지 않으면 점화 안된 것)
        # 속도 Global z값 확인(1m/s 이상 유지해야함)
        # 각속도 Global x,y값 확인( 로켓 자세가 이상하게 떨어지면 점화 중지 )
        
        self.RocketStep+=1
        self.Rocket3time=time();
        print("Rocket3")
        return True   
     
    def Algorithm4Check(self,data):
        # 2차 추력 완료 체크 == 알고리즘 1번과 같음, 한번 쭉 읽어볼 것
        self.Rocket4time=time()-self.Rocket3time
        if self.Is2stAccel:
            self.previous_value = self.current_value
            self.current_value = data[5]
            
            if self.increasing:
                if self.current_value < self.previous_value:
                    self.increasing = False
                return False
            else:
                if self.current_value < self.Is1stAccelThreshold:
                    self.RocketStep+=1
                    if RPI:
                        self.set2ndServoBoolean(True)
                    print("Rocket4")
                    return True
                else:
                    return False

        if abs(data[5])>=self.Is1stAccelThreshold: # threshold 확인
            self.Is2stAccel=True
            self.increasing=True
            
        if self.Rocket4time>=1:
            self.RocketStep+=1
            self.set2ndServoBoolean(True)
            return True
            
        return False  
    
    def AlgorithmProcess(self,mSensorqueue): #기본적인 시작 멈춤 알고리즘. 시작일시 dt 마다 계속 실행됨.
        # 알고리즘 전체
        data = mSensorqueue.get()
#        print(data)
        if(self.RocketStep==0):
#            print("Rocket step1")
            self.Algorithm1Check(data)

        elif(self.RocketStep==1):
#            print("Rocket step2")
            num=self.Algorithm2Check(data);
            if(num!=0):
                if(num==1):
                    print("End")
                if(num==2):
                    print("Engine")

        elif(self.RocketStep==2):
#            print("Rocket step3")
            self.Algorithm3Check(data)

        elif(self.RocketStep==3):
#            print("Rocket step4")
            self.Algorithm4Check(data)

        elif(self.RocketStep==4):
            self.RocketStep+=1
            print("Rocket finished")

        return self.RocketStep>=self.RocketMaxStep    
    
