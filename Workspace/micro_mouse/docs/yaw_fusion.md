# Fusion yaw IMU + encoder

## PID vong kin 200 Hz

Duong tin hieu: `MPU_Get_Calibrated_GZ_RadS()` (calibration + Kalman 1D
van toc goc trong mpu6500.c) + delta encoder -> `YawFusion_Update()` ->
`robot_pose.w_angular` va tich phan `robot_pose.theta` -> PID -> PWM motor.
Khong goi them `MPU_Update_Angle()` trong vong nay: se doc va cap nhat Kalman
lan thu hai trong cung chu ky.

Di thang: PID van toc tien dung trung binh encoder nhu truoc. PID huong dung
`wrap(target_heading - theta_fused)`; P/I tren sai so goc, D la
`-Kd*w_fused`. PWM trai = PWM tien - PWM huong, phai = PWM tien + PWM huong.
Giu hieu chinh IR va dong bo banh hien co o sau dau ra PID huong.

Quay: PID rieng dung sai so goc quay con lai tu fusion va van toc goc co dau
theo huong lenh. Them feedforward theo profile van toc goc. Giu lead phanh,
dung khi dat dung sai va timeout 1600 ms hien co. PID khong co chuc nang
giu nguyen huong sau khi lenh ket thuc, vi MOTION_STOP phanh motor.

`feedback_pid.h` co D tren do luong (tranh derivative kick khi doi setpoint),
gioi han I va conditional integration tai gioi han dau ra PID. Cac gioi han
PWM sau mix/dong bo/min PWM chua duoc hoi tiep de back-calculation I.
He so khoi dau trong `motion.h`: HEADING_PID_KP/KI/KD, TURN_PID_KP/KI/KD/KFF.
Chua duoc tune tren phan cung. Kiem tra dau truoc, tune P va D voi I=0,
sau do tang I nho neu co sai so on dinh. PID khong xoa duoc bias cua cam bien.

`tests/feedback_pid_test.c` kiem tra saturation, I, D va hoi tu hai chieu
tren mo hinh than xe gia lap don gian; khong thay the kiem tra robot.

Goc dieu khien va goc gui UART hien lay tu `robot_pose.theta` (radian noi bo,
doi sang do khi gui). CSV van gom 3 cot: encoder trai, encoder phai, yaw fusion.
`mpu_yaw_rad` giu goc IMU rieng de doi chieu.

## Nguyen nhan va thay doi

- Cong co dinh 20% encoder vao heading se tao goc gia khi banh quay tren de.
- Bo tu dong dao dau gyro theo encoder: banh truot khong the xac dinh dau IMU.
- Lay delta xung co dau va xu ly tran TIM2 32-bit / TIM3 16-bit. Reset ca moc
  delta khi bat dau moi lenh. Tong xung quang duong duoc cong tu tri tuyet doi
  delta, tranh nhay khi TIM3 vuot 32767.
- Dung mot bo fusion van toc goc cho dung, di thang, re va cho phanh.
- Loc van toc encoder voi hang so thoi gian 30 ms. Chi chap nhan khi gyro
  >= 0.02 rad/s, cung dau, va sai khac < 0.08 + 0.20*abs(gyro) rad/s lien tuc
  hon 100 ms. Trong so encoder toi da 15%, giam theo sai khac; loai ngay
  khi khong dat dieu kien. Nguong nay la gia tri khoi dau, can do thuc te.
- Khong tinh lai goc tu tong xung sau khi phanh. Khong tich phan truc tiep
  sai so khoang cach IR vao yaw; van giu IR de dieu khien bam vach.
- Chi bat SysTick sau can chinh gyro, tranh hai luong cung doc SPI2 trong
  luc calibration.

## Kiem tra dau truoc khi chay tren san

`motion.h` quy uoc tien la duong cho ca hai banh, re trai (CCW, nhin tu tren)
la yaw duong. Gia tri hien tai: encoder trai -1, phai +1 (theo dau du lieu UART
hien co), gyro Z -1 (theo quy uoc MPU_Update_Angle hien co).
Day la cau hinh can xac minh tren phan cung, khong tu dong nhan dang.

1. Quay tung banh theo chieu tien: delta da nhan dau phai duong. Neu nguoc,
   sua `MOTION_ENCODER_LEFT_SIGN` hoac `MOTION_ENCODER_RIGHT_SIGN`.
2. Tat motor, quay than xe sang trai bang tay: `debug_fusion_gyro_rad_s`
   va `robot_pose.theta` phai tang. Neu nguoc, sua `MOTION_GYRO_Z_SIGN`.
3. Chay banh tren de, than dung yen: `debug_fusion_encoder_weight` phai bang 0
   khi gyro gan 0. Theo doi `debug_fusion_encoder_rad_s` de thay goc encoder gia.
4. Tren san, re trai/phai: gyro va encoder phai cung dau. Neu khong, sua dau
   truoc khi dieu chinh nguong. Kiem tra lai dung goc va PID sau thay doi fusion.

Khong dung ket qua x/y hay quang duong de danh gia chuyen dong than xe tren de:
encoder van do banh quay, nen vi tri van co the tang. Lenh re tren de se cho
den timeout 1600 ms neu than khong quay, thay vi bao re xong theo encoder.

## Gioi han

Day la fusion van toc co kiem tra tinh phu hop, khong phai moc yaw tuyet doi.
Khong tu hoc bias gyro khi banh quay, vi khong the phan biet chac chan bias
voi quay cham that chi tu hai cam bien nay. Neu gyro co bias/nhieu khi motor
chay, yaw van co the troi. Quay cham < 0.02 rad/s dung gyro rieng; deadband
gyro hien co 0.00436 rad/s cung lam mat chuyen dong rat cham.
Ban truot nhe co the lot qua nguong va anh huong nho den yaw. De rang buoc
yaw lau dai can them tham chieu huong hop le (vision, moc tuong duoc mo hinh
hoa hinh hoc, hoac magnetometer da hieu chuan va xu ly nhieu motor).

Test toan hoc: `tests/yaw_fusion_test.c`, build voi compiler C host va
`-ICore/Inc`. Ket qua 0 nghia la tat ca kiem tra dat; khac 0 la dong test loi.
