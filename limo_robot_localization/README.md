# limo_robot_localization

LIMO를 위한 **EKF 기반 2D 로컬라이제이션** 패키지입니다.  
출력은 `/odometry/filtered` (및 선택적으로 `odom→base_link` TF)이며,  
`map→odom`은 **AMCL** 등에서 제공하는 구조를 권장합니다.

## 노드/런치
- `launch/ekf_localization.launch`
  - `robot_localization/ekf_localization_node` 1개 인스턴스(2D)
  - 입력: `/odom`, `/imu/data`
  - 출력: `/odometry/filtered` (+ TF 옵션)

### 실행
```bash
roslaunch limo_robot_localization ekf_localization.launch
