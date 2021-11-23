# Xycar Auto Drive

## Brief
로봇비전시스템 과목 기말 프로젝트 코드입니다. [Xycar](http://xytron.co.kr/?page_id=682)를 이용하여 차선인식 주행, 정지선 및 신호등 인식 등의 자율주행을 구현했습니다.

## Mission
Hough transform을 이용한 차선 인식(곡선 차로 주행). 정지선에서 정지 후 신호등에 따라 지정된 동작 수행.    
openCV를 이용한 비전 알고리즘 구현 및 ROS를 통한 토픽 관리.

## Environment
 - NVIDIA Xavier
 - Ubuntu 18.04
 - ROS Melodic
 - openCV
 - python 3

## Auto drive - `hough_drive.py`
 1. line detection & driving   
    canny_edge + hough_transform -> line detection & slope   
    left_slope + right_slope + middle_point -> angle + speed   
    angle + speed + publisher_node -> motor_control
    ```python
    hough(image)
    drive(Angle, Speed)
    start()
    ```
 2. stop line & traffic light   
    ROI + threshold -> stop line image   
    ROI + HSV image + inRange -> traffic light mask image   
    stop line image, traffic light image + countNonZero -> mission detection
    ```python
    detect_stopline(image, mid_x, low_threshold_value)
    traffic(image)
    ```

Xycar 로컬 환경에서 작업하고 소스코드만 가져온 것입니다.
