# UWB와 OCAM을 위한 ROS 패키지 실행 가이드
이 문서는 UWB (초광대역) 센서와 OCAM 카메라를 ROS 환경에서 설정하고 실행하는 방법을 안내합니다.

## UWB 시작하기
1. UWB 장치 연결 및 포트 설정
기존 UWB 센서를 USB 포트에 연결하면 일반적으로 /dev/ttyACM0 또는 /dev/ttyUSB0과 같은 이름으로 시스템에서 식별됩니다. 하지만 이 포트 이름은 연결 순서나 다른 장치에 따라 바뀔 수 있어 불편합니다. udev 규칙을 사용하여 항상 /dev/uwb_port라는 고정된 이름으로 장치를 인식하도록 설정할 수 있습니다.

2. udev 규칙 설정 방법
udev는 리눅스 커널의 장치 관리자로, 장치가 연결되거나 연결 해제될 때 특정 동작을 수행하도록 규칙을 설정할 수 있습니다.

3. UWB 장치 정보 확인

    먼저 터미널을 열고 아래 명령어를 입력하여 연결된 USB 장치 목록과 상세 정보(idVendor, idProduct 등)를 확인합니다.
    ```bash
    lsusb
    ```
  
4. udev 규칙 파일 생성

    다음 명령어로 새로운 udev 규칙 파일을 생성합니다.
    ```bash
    sudo nano /etc/udev/rules.d/99-usb-serial.rules
    ```
5.  규칙 내용 작성

    열린 편집기 안에 아래와 같은 형식으로 규칙을 작성합니다. idVendor와 idProduct 부분에는 a단계에서 확인한 자신의 장치 값으로 변경해주세요. SYMLINK는 우리가 원하는 고정된 장치 이름(/dev/uwb_port)을 지정합니다.

    SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="uwb_port"

6.  udev 규칙 적용

    아래 명령어를 차례대로 실행하여 변경된 규칙을 시스템에 적용합니다.

    ```bash
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```

    이제 UWB 장치를 USB 포트에서 분리했다가 다시 연결하면 /dev/uwb_port라는 이름으로 장치가 생성된 것을 확인할 수 있습니다. ls /dev/ 명령어로 확인해보세요.

7.  UWB ROS 노드 실행

    uwb_pkg 내의 script 폴더로 이동하여 아래 명령어로 UWB 퍼블리셔 노드를 실행합니다.
    
    uwb_pkg/script 폴더로 이동
    ```bash
    cd <your_catkin_ws>/src/uwb_pkg/script
    ```
    파이썬 스크립트 실행
    ```bash
    python3 uwb_pub.py
    ```
## OCAM 시작하기
1. OCAM 시리얼 번호 확인

    각 OCAM 카메라는 고유한 시리얼 번호를 가지고 있습니다. 이 번호는 OCAM 제조사에서 제공하는 뷰어 프로그램을 사용하여 확인할 수 있습니다.

    참고 링크: [withrobot oCam-1CGN-U-T 2](https://withrobot.cafe24.com/camera/ocam-1cgn-u-t2/)

2. OCAM ROS 노드 실행
    ocam_pkg 내의 launch 폴더로 이동하여 카메라 개수에 맞는 런치 파일을 실행합니다.

    카메라 한 대 실행 시:

    ```bash
    roslaunch ocam_pkg one_ocam_ros.launch
    ```

    여러 카메라 실행 시: (ocam_ros.launch 파일 내에 각 카메라의 시리얼 번호가 설정되어 있어야 합니다.)

    ```bash
    roslaunch ocam_pkg ocam_ros.launch
    ```

## 함께 시작하기 (UWB + OCAM)

config_sensor 패키지를 사용하면 설정된 모든 센서(UWB, OCAM 등)를 한 번에 실행할 수 있습니다.

config_sensor 패키지의 launch 디렉토리로 이동하여 아래의 런치 파일을 실행하세요.

```bash
roslaunch config_sensor sensor.launch
```
