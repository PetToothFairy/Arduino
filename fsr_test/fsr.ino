const int fsrPin = A0;   // FSR-400 센서를 A0 핀에 연결
int fsrValue = 0;        // FSR 값을 저장할 변수

void setup() {
  Serial.begin(9600);    // 시리얼 모니터 시작 (속도: 9600bps)
  pinMode(fsrPin, INPUT); // FSR 핀을 입력 모드로 설정
}

void loop() {
  // FSR 센서에서 아날로그 값 읽기
  fsrValue = analogRead(fsrPin); 

  // 시리얼 모니터에 출력
  Serial.print("FSR Value: ");
  Serial.println(fsrValue);

  // 짧은 대기 시간 (100ms)
  delay(100);
}
