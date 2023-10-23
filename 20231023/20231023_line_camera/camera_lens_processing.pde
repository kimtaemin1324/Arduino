import processing.serial.*;

int i; // 반복문에서 사용되는 변수 선언

Serial myPort; // 시리얼 통신을 위한 Serial 객체 선언
float data; // 시리얼 포트에서 받은 데이터를 저장하는 변수
float[] line_data = new float[128]; // 그래프를 그리기 위한 데이터 배열, 크기 128
int center_x = 0; // 그래프 중심의 x 좌표값을 저장하는 변수

// 상태를 나타내는 상수 선언
public static int ON = 1; // ON 상태를 나타내는 상수
public static int OFF = 0; // OFF 상태를 나타내는 상수

void setup() {
  // 시리얼 통신 설정
  String portName;
  size(1000, 1000); // 화면 크기 설정
  delay(1000); // 1초 대기

  println(Serial.list()); // 사용 가능한 시리얼 포트 목록 출력
  portName = Serial.list()[0]; // 첫 번째 시리얼 포트를 사용

  myPort = new Serial(this, portName, 115200); // 시리얼 통신 객체 초기화 (포트 번호, 전송 속도 설정)
}

void draw() {
  background(255); // 화면 배경을 흰색으로 설정

  // 그래프 그리기
  int i;
  for (i = 0; i < 128; i++) {
    rect(20 + 5 * i, 950, 5, -line_data[i]); // 막대 그래프 그리기
    fill(0, 0, 255); // 파란색으로 채우기
  }
  fill(255, 0, 0); // 빨간색으로 채우기
  rect(20 + 5 * center_x, 950, 5, -400); // 중심 그래프 그리기
}

// 시리얼 데이터를 읽는 함수
void serialEvent(Serial port) {
  int i, data_temp;
  String input = port.readStringUntil('\n'); // 시리얼 데이터 읽기
  if (input != null) {
    input = trim(input); // 문자열 앞뒤의 공백 제거
    String[] values = split(input, " "); // 공백을 기준으로 문자열 분리
    println(values.length); // 분리된 문자열 배열의 길이 출력
    if (values.length == 129) { // 올바른 데이터 길이 확인

      // 그래프 데이터 읽기
      for (i = 0; i < 128; i++) {
        data_temp = int(values[i]); // 문자열을 정수로 변환
        line_data[i] = map(data_temp, 0, 256, 0, 800); // 데이터 매핑하여 그래프 값 설정
        print(line_data[i]); // 매핑된 그래프 데이터 출력
        print(" ");
      }
      center_x = int(values[i]); // 중심 x 좌표값 읽기
      print(center_x); // 중심 x 좌표값 출력
      println(" ");
    }
  }
}
