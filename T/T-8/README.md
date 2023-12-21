## 기초로봇공학실험 화요일반 8조 코드
desktop의 lane_detection에서 1~3번 어플을 통해 구동하는 방식.
관련 소스코드는 wego_ws/limo_applicationmaster에서 확인 및 수정 가능.
구동 코드는 scripts의 .py파일, 기준 값은 param의 .yaml파일에서 수정 가능.
마커인식과 LKAS구동의 동시실행을 위해 control.py와 lane_detection.launch에 마커인식 코드를 합친 방식
(마커코드는 catkin_ws의 5.markerIDcontrol을 활용)


[lane_detect.py]: (줄 36에서 38) 기존코드는 원하는 이미지 영역인 _img[420:480, 0:320]을 바로 return하지만 opencv함수인 cv2.blur를 이용하여 검출한 이미지 영역에 블러처리(블러처리할 영역 범위는 홀수여야 함을 유의). 주석처리된 cv2.GaussianBlur는 계산량이 많아 딜레이가 걸릴 수 있음(하드웨어 성능 혹은 코드 복잡도에 따라 고려).
--> 블러처리 이점: 자잘한 값 혹은 멀리 있는 값을 지워 LKAS주행 안정성을 올릴 수 있음(방지턱을 넘을 때 변화한 화각으로 인해 멀리 있는 차선을 보고 회전하는 문제 해결).
(줄 74에서 76) 주석처리된 부분은 기존코드는 활성화 되어 있음(카메라 영상 띄워주는 코드, 딜레이 감소를 위해 비활성화 시킴)


[control.py]: __init__(self)함수 내에 선언된 변수(self.)는 LimoController 클래스 내에서 전역변수처럼 사용가능(필요에 따라 선언).
마커인식을 위해 markerIDcontrol의 marker_CB(self, data)함수를 응용(줄6에서 15에 해당하는 헤더파일 비교 후 없는 것 추가 필요).
해당함수의 기존코드 변수는 control.py에서 사용불가. 따라서 마커인식부분 외의 부분은 삭제하였음.

control.py에서 차량 구동에 대한 코드를 다루기 위해선 drive_callback(self, _event)함수에서 사용가능한 drive_data.linear.x , drive_data.angular.z가 필요. 따라서 인식된 marker.id값을 전역변수에 저장하여 drive_callback함수에서 해당 값을 사용할 수 있게 하였음.(drive_data.linear.x --> 전진, 후진 값. drive_data.angular.z --> 좌우 조향 각도)
self.drive_pub.publish(drive_data)는 드라이브 데이터를 차량에 보내기 위한 코드, 해당 코드가 선언되지 않으면 코드로 설정한 조향값이 차량으로 보내지지 않을 수 있으므로 유의.

인식된 마커에 따른 구동코드는 (줄227에서 374). 이 부분은 하드웨어 혹은 기본속도값에 따라 최적화할 필요가 있음. 해당부분에서 마커에 따른 모션 수행 후 LKAS구동으로 복귀하기 위해 전역변수에 저장된 marker.id값 초기화(self.MARKER = None)

줄193~201은 벽에서 e-stop이 발생하였을 때 차선복귀를 위한 코드, e-stop발동 후 시간이 10초가 넘으면 회전하는 방식.
self.calcTimeFromDetection(self.pedestrian_stop_last_time)로 시간측정, 
self.pedestrian_stop_last_time = rospy.Time.now().to_sec()는 측정시간 초기화 --> 돌발상황, 차단기와 같은 상황에서 발동한 e-stop에서 기록된 시간이 누적되기 때문에 이를 방지하기 위해 필요(줄221)


[lane_detection.launch]: 기존 limo_applicationmaster의 launch의 .launch파일에 markerIDcontrol의 launch을 합친 방식. 
이때 기존 5.markerIDcontrol.launch파일에 오류가 있어 해당부분 수정하여 두 파일을 합쳤음.
