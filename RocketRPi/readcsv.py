import csv
import queue
from time import sleep
from time import time


class RocketProtocol:
    def __init__(self):
        ##initial
        self.mSeperationServoPin = 33 ##change

        self.m2ndServoPin = 32

        self.mIgnitionRelayPin = 37 ##change

        self.SERVO_MAX_DUTY = 12.5 # servo Max
        self.SERVO_MIN_DUTY = 2.5

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

    # CSV 파일에서 데이터를 읽어 sensor_queue에 넣는 함수
    def load_csv_to_queue(self,csv_file_path, sensor_queue):
        with open(csv_file_path, newline='') as csvfile:
            csv_reader = csv.reader(csvfile)
            for row in csv_reader:
                # 각 행(row)을 숫자(float) 배열로 변환
                sensor_data = [float(value) for value in row]
                if len(sensor_data) == 9:
                    sensor_queue.put(sensor_data)
                    print(f"Loaded Sensor Data: {sensor_data}")
                else:
                    print(f"Invalid row length: {row}")

    def Algorithm1Check(self,data):
        # 1차 점화 완료 체크
        # 가속도 Global z값 확인
        if self.Is1stAccel:
            self.previous_value = self.current_value
            self.current_value = data[3]
            
            if self.increasing:
                if self.current_value < self.previous_value:
                    self.increasing = False
                    print("IncreasingFalse")
                return False
            else:
                if self.current_value < self.Is1stAccelThreshold:
                    self.RocketStep+=1
                    print("단분리 실행")
                    print("Rocket1")
                    self.increasing = False
                    self.Rocket1time=time();
                    return True
                else:
                    print("Increasing222False")
                    return False

        elif abs(data[3])>=self.Is1stAccelThreshold: # threshold 확인
            self.current_value=data[3]
            self.Is1stAccel=True
            self.increasing=True
            print("Increasing")
        return False    
        
    def Algorithm2Check(self,data):
        # 2차 점화전 시작 체크
        # 속도 Global z값 확인(1m/s 이상 True)
        # 각속도 Global x,y값 확인
        self.Rocket2time=time()-self.Rocket1time
        if self.Rocket2time>=0.5:
            if ((data[1]*data[1])+(data[2]*data[2]))**(1/2)<=3.14/9: #각도 체크
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
        # 2차 추력 완료 체크 == 알고리즘 1번과 같음
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
                    print("2단 낙하산 실행")
                    print("Rocket4")
                    return True
                else:
                    return False

        if abs(data[5])>=self.Is1stAccelThreshold: # threshold 확인
            self.Is2stAccel=True
            self.increasing=True

        if self.Rocket4time>=1:
            self.RocketStep+=1
            return True    
        return False  
    
    def AlgorithmProcess(self,mSensorqueue):
        # 알고리즘 전체
        data = mSensorqueue.get()
        print(data)
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
            #self.set2ndServoBoolean(True)
            print("Rocket finished")

        return self.RocketStep>=self.RocketMaxStep    

rocketProtocol=RocketProtocol()
# Queue 생성
sensor_queue = queue.Queue()

# CSV 파일 경로 설정
csv_file_path = 'myexfile.csv'

# CSV 파일에서 데이터를 읽어 Queue에 추가
rocketProtocol.load_csv_to_queue(csv_file_path, sensor_queue)

# Queue에서 데이터를 가져와서 처리 (예제)
while not sensor_queue.empty():
    
    rocketProtocol.AlgorithmProcess(sensor_queue)
    sleep(0.02)
