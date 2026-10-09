# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Vũ Tiến Linh
- MSSV: 2A202602657
- Email: vutienlinh95@gmail.com
- Link repo (fork): https://github.com/Linh-Huong/K4-L2L3-DAY23-VuTienLinh-2A202602657-SensorFusion
- Commit hash nộp (`git rev-parse HEAD`): 33f24c7498e6188bae54ea37977cec9909d35ebf

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: `compare`, `[0, 198]`, `training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`, `0`
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: `precision = 0.9700934579439252` (~0.9701), `recall = 0.7004048582995951` (~0.7004), `tp = 519`, `fp = 16`, `fn = 222`
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: `rmse = 0.15032270040636328` (~0.1503 m), `matches = 502`, `sum_sq_err = 11.343650957245547` (~11.3437 m²), `ghost_track_frames = 0`, `missed_gt_frames = 239`, `mean_confirmed_tracks = 2.522613065326633`
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: `rmse = 0.1358667802927653` (~0.1359 m), `matches = 502`, `sum_sq_err = 9.266810557535527` (~9.2668 m²), `ghost_track_frames = 0`, `missed_gt_frames = 239`, `mean_confirmed_tracks = 2.522613065326633`
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss:
  - **Về vòng đời track (Lifecycle & Association)**: Cả hai mode `lidar` và `fused` có số lượng confirmed tracks (`mean_confirmed_tracks = 2.5226`), số lượng track được ghép với nhãn ground-truth (`matches = 502`), số ghost tracks (`ghost_track_frames = 0`), và số nhãn bị bỏ sót (`missed_gt_frames = 239`) giống hệt nhau trên từng frame (đạt `precision_track = 502 / (502 + 0) = 1.0` và `coverage = 502 / 519 = 96.72%`). Điều này hoàn toàn nhất quán với thiết kế kiến trúc **track-then-fuse** của lab: việc khởi tạo (init), tính điểm tồn tại (score update), xác nhận (confirmed), và xóa (delete) track hoàn toàn do cảm biến LiDAR quyết định. Cảm biến camera không can thiệp vào lifecycle của track.
  - **Về độ chính xác vị trí (RMSE 3D)**: Chế độ `fused` đạt sai số vị trí tốt hơn so với `lidar`: `rmse_fused = 0.1359 m` so với `rmse_lidar = 0.1503 m`, tương ứng mức giảm sai số `rmse_fused - rmse_lidar = -0.0145 m` ($\le 0.05$ m, thỏa mãn điều kiện nhất quán của rubric), và tổng bình phương sai số `sum_sq_err` giảm từ `11.3437 m²` xuống `9.2668 m²`. Nguyên nhân là camera FRONT cung cấp thêm các phép đo góc 2D có độ phân giải cao trên mặt phẳng ảnh; sau khi được liên kết qua cổng Mahalanobis, bước EKF update camera đã giúp tinh chỉnh và hiệu chỉnh tọa độ ước lượng của track, làm tăng độ chính xác vị trí 3D so với chỉ dùng một mình LiDAR.

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E–H — tự viết)

1. Khác biệt đo lidar 3D và camera 2D trong EKF (`z`, `R`)?
   - **Vector đo $z$**:
     - LiDAR đo trực tiếp tọa độ vị trí 3D trong hệ quy chiếu xe: $z_{\text{lidar}} = [x, y, z]^T \in \mathbb{R}^3$. Mô hình đo là tuyến tính $z = Hx$ với ma trận quan sát $H_{\text{lidar}} = \begin{bmatrix} I_{3\times 3} & 0_{3\times 3} \end{bmatrix} \in \mathbb{R}^{3 \times 6}$ (cài đặt trong `student/workspace/kalman.py`).
     - Camera đo tọa độ pixel 2D trên mặt phẳng ảnh của camera FRONT: $z_{\text{cam}} = [u, v]^T \in \mathbb{R}^2$. Mô hình đo là phi tuyến $z = h(x)$ dựa trên mô hình chiếu pinhole sau khi biến đổi tọa độ từ xe sang camera $p_s = R_{\text{sens}} p_{\text{veh}} + t_{\text{sens}}$: $u = f_x \frac{y_s}{x_s} + c_x$, $v = f_y \frac{z_s}{x_s} + c_y$ (cài đặt trong `student/workspace/camera_fusion.py`). Do phi tuyến, EKF phải dùng ma trận Jacobian camera $H_{\text{cam}} = \frac{\partial h}{\partial x} \in \mathbb{R}^{2 \times 6}$.
   - **Ma trận hiệp phương sai sai số đo $R$**:
     - LiDAR: $R_{\text{lidar}} \in \mathbb{R}^{3 \times 3}$, thường là ma trận đường chéo $\mathrm{diag}(\sigma_x^2, \sigma_y^2, \sigma_z^2)$ với đơn vị mét vuông ($\text{m}^2$), phản ánh độ bất định không gian hình học 3D.
     - Camera: $R_{\text{cam}} \in \mathbb{R}^{2 \times 2}$, có dạng đường chéo $\mathrm{diag}(\sigma_u^2, \sigma_v^2)$ với đơn vị $\text{pixel}^2$, phản ánh độ bất định rời rạc trên lưới điểm ảnh (cài đặt trong `build_camera_measurement`).

2. Vì sao cần gating Mahalanobis trước khi gán?
   - Khoảng cách hình học Euclidean thuần túy $\|z - h(x)\|$ xem mọi hướng sai số đều có trọng số như nhau và bỏ qua hoàn toàn độ bất định phân phối của trạng thái track (thể hiện qua ma trận hiệp phương sai $P$) cũng như độ nhiễu của cảm biến ($R$).
   - Khoảng cách Mahalanobis bình phương $d_M^2 = \gamma^T S^{-1} \gamma$ (với $\gamma = z - h(x)$ là innovation residual và $S = H P H^T + R$ là hiệp phương sai innovation) thực hiện chuẩn hóa sai số theo toàn bộ ellipsoid bất định tổng hợp của track và phép đo.
   - Gating bằng phân phối Chi-bình phương ($\chi^2$-gate) theo số bậc tự do của phép đo (bậc 3 cho LiDAR với ngưỡng $\approx 7.81$, bậc 2 cho camera với ngưỡng $\approx 5.99$ ở mức tin cậy 95% trong `student/workspace/association.py`) giúp loại bỏ dứt điểm các cặp đo - track nằm ngoài vùng xác suất thống kê có ý nghĩa trước khi thực hiện thuật toán gán greedy. Điều này ngăn ngừa gán sai mục tiêu (ID switch), loại trừ clutter, và tránh làm EKF cập nhật thông tin sai lệch dẫn tới phân kỳ bộ lọc.

3. Pipeline là track-then-fuse hay fuse-then-track? Chỉ ra trên log `fusion-run-lab`.
   - Pipeline của bài lab là **track-then-fuse** (cụ thể là sequential track-level sensor fusion). Hệ thống không ghép dữ liệu cảm biến ở mức raw/feature trước khi phát hiện vật thể (fuse-then-track), mà xây dựng và duy trì một danh sách các track 3D; tại mỗi chu kỳ frame, EKF thực hiện dự báo (`predict`) một lần, rồi lần lượt liên kết và cập nhật trạng thái (`update`) nối tiếp theo từng cảm biến: đầu tiên là LiDAR (`AssocL`), tiếp theo là Camera (`AssocC`).
   - Minh chứng trên log `fusion-run-lab` và `grade_run.log`:
     - Ở mỗi frame, tracker gọi một lần EKF predict duy nhất. Tiếp đó chạy bước `AssocL` để gán đo LiDAR 3D, cập nhật EKF và cập nhật vòng đời track (`manage_tracks`). Sau đó, nếu ở mode `fused`, bước `AssocC` được gọi để gán các đo camera FRONT 2D và cập nhật EKF nối tiếp cho các track đã có, tuyệt đối không tạo mới hay xóa track.
     - Trên `grade_run.log`, giữa hai chế độ `lidar` và `fused`, các chỉ số về số lượng track được xác nhận (`confirmed = 3` ở frame 0; trung bình `2.52261`), số track ghép thành công (`matches = 502`), số ghost (`ghosts = 0`), và số miss (`misses = 239`) ở hai mode hoàn toàn giống hệt nhau trên từng frame. Chỉ có tổng bình phương sai số vị trí `sum_sq_err` là giảm ở mode `fused`. Điều này chứng minh camera chỉ tham gia tinh chỉnh trạng thái EKF của các track đã được LiDAR quản lý, đúng bản chất của track-then-fuse.

4. Nếu camera lệch calibration, triệu chứng gì trên innovation/residual?
   - Ngoại tham số camera ($R_{\text{sens}}, t_{\text{sens}}$) hoặc nội tham số ($f_x, f_y, c_x, c_y$) bị lệch (drift) sẽ khiến hàm chiếu phi tuyến $h(x)$ chiếu vị trí 3D dự báo của vật thể sang tọa độ pixel $(u, v)$ bị lệch một khoảng dịch chuyển có hệ thống (systematic bias).
   - Triệu chứng trên innovation $\gamma = z - h(x)$:
     1. **Kỳ vọng innovation khác 0** ($\mathbb{E}[\gamma] \neq 0$): Innovation không còn là nhiễu trắng Gauss zero-mean mà xuất hiện độ lệch một chiều (systematic offset/bias) lớn theo phương ngang hoặc đứng trên ảnh.
     2. **Khoảng cách Mahalanobis $d_M^2 = \gamma^T S^{-1} \gamma$ tăng vọt**: Thường xuyên vượt quá ngưỡng cổng $\chi^2$-gate ($5.99$), khiến thuật toán association từ chối gán đo camera cho track (tỷ lệ ghép camera giảm mạnh, camera measurements bị coi là clutter).
     3. **Phân kỳ hoặc kéo lệch track nếu lọt gate**: Nếu một phần đo vẫn lọt qua gate, Kalman gain sẽ cập nhật trạng thái kéo lùi vị trí 3D của track lệch khỏi quỹ đạo thực tế để bù cho sai số chiếu, làm tăng đột biến RMSE vị trí 3D và làm hỏng ma trận hiệp phương sai $P$.

5. Vì sao `associate_and_update(..., sensor)` cần sensor tường minh ở frame rỗng?
   Giải thích vì sao lidar quyết định score/init/delete còn camera chỉ EKF update.
   - **Cần `sensor` tường minh ở frame rỗng**:
     - Kể cả khi danh sách đo `meas_list` rỗng (không có detection nào trong frame), hàm `associate_and_update` trong `student/workspace/association.py` vẫn bắt buộc phải gọi `manager.manage_tracks(unassigned_tracks, unassigned_meas, sensor)`.
     - Hàm quản lý vòng đời `manage_tracks` cần biết sensor hiện tại là `lidar` hay `camera`. Nếu là lượt của `lidar`, tất cả các confirmed/unconfirmed track nằm trong tầm nhìn LiDAR mà không có đo tương ứng (unassigned) phải bị trừ điểm tồn tại (score bị phạt trừ $1 / \text{window}$). Nếu bỏ qua khi `meas_list` rỗng hoặc không truyền rõ sensor, hệ thống sẽ bỏ sót việc phạt giảm score khi mất dấu xe, làm tồn tại các track "ma" (ghost tracks).
   - **Vì sao LiDAR quyết định lifecycle còn camera chỉ EKF update**:
     - LiDAR đo trực tiếp vị trí không gian 3D $(x, y, z)$ với độ chính xác khoảng cách cao và góc quét toàn cảnh $360^\circ$ quanh xe tự hành, cung cấp đủ bằng chứng vật lý để khởi tạo trạng thái 6D ban đầu cũng như xác định sự tồn tại/biến mất của vật thể trong không gian 3D.
     - Camera (đặc biệt là mono camera FRONT) chỉ cung cấp thông tin góc 2D trên ảnh phẳng $(u, v)$, hoàn toàn thiếu đo trực tiếp độ sâu (khoảng cách $x_s$), và có góc nhìn hẹp (FOV bị giới hạn). Một điểm đo 2D trên ảnh tương ứng với một tia vô hạn trong không gian 3D nên không thể tự khởi tạo trạng thái 3D nếu không có giả định phụ trợ. Ngoài ra, khi xe chạy ra khỏi tầm nhìn hẹp của camera FRONT nhưng vẫn nằm trong tầm quét của LiDAR, nếu camera được phép trừ điểm hay xóa track thì sẽ xóa nhầm các vật thể hợp lệ đang di chuyển bên hông hoặc phía sau xe. Do đó, thiết kế bất đối xứng (asymmetric lifecycle) phân định rõ: LiDAR đóng vai trò "anchor" quyết định sinh/tử, còn camera chỉ đóng vai trò hỗ trợ tinh chỉnh ước lượng trạng thái EKF.

6. Nêu điều kiện xác nhận, giữ confirmed sau miss, và điều kiện xóa track.
   - Căn cứ theo các tham số tracking từ `get_tracking_params()` và cài đặt trong `student/workspace/track_management.py`:
     1. **Điều kiện xác nhận (Confirmed)**:
        - Khi một track được gán với đo LiDAR (hit), score của track được cộng thêm $1 / \text{window}$ (với $\text{window} = 6$, tương đương $+0.1667$, tối đa là $1.0$).
        - Khi `score > params.confirmed_threshold` (ngưỡng mặc định là $0.6$), trạng thái track được chuyển thành `confirmed`.
     2. **Điều kiện giữ confirmed sau miss**:
        - Khi một track đã ở trạng thái `confirmed` bị miss LiDAR (nằm trong FOV nhưng không có đo được gán), score bị trừ $1 / \text{window}$.
        - Một lần miss đơn lẻ chỉ làm giảm score từ $1.0$ xuống khoảng $0.8333$, giá trị này vẫn lớn hơn `params.confirmed_threshold` ($0.6$) và lớn hơn `params.delete_threshold` ($0.4$). Trạng thái `confirmed` của track vẫn được giữ nguyên, đảm bảo tính liên tục của quỹ đạo khi xe bị che khuất tạm thời 1-2 frame.
     3. **Điều kiện xóa track (`should_delete_track`)**: Track bị xóa (trả về `True`) khi vi phạm ít nhất một trong ba tiêu chí:
        - **Bất định vị trí quá lớn**: Phương sai vị trí $P[0,0] > \text{params.max\_P}$ hoặc $P[1,1] > \text{params.max\_P}$ (với $\text{max\_P} = 100.0$).
        - **Track đã confirmed bị miss nhiều lần**: `track.state == "confirmed"` và `score < params.delete_threshold` (ngưỡng mặc định $0.4$).
        - **Track chưa confirmed (unconfirmed / initialized)**: `score <= 0.0`.

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

- Không.

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Antigravity IDE (Gemini 2.5 Flash / Advanced Agentic Assistant)
- Dùng cho phần nào (hàm, câu hỏi, debug): Hỗ trợ phân tích công thức chuyển đổi tọa độ xe-camera và mô hình pinhole projection (Part G), rà soát công thức ma trận EKF 6D (Part E), logic Chi-square gating và gán Greedy trong association (Part F), đối chiếu quy tắc vòng đời track (Part H), và tổng hợp các số liệu định lượng từ `metrics.json` và `grade_run.log` cho báo cáo.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): Tự chạy toàn bộ kiểm thử đơn vị `pytest student/tests/ -v` (128/128 passed), đối chiếu công thức với tài liệu `docs/HUONG_DAN_KY_THUAT.md`, chạy thực nghiệm tích hợp `fusion-run-lab` trên 198 frames dữ liệu Waymo, và kiểm tra tính toàn vẹn bất biến dữ liệu bằng script kiểm tra `python tools/check_submission.py`.

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền đủ file này, gồm khai báo AI
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [x] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [x] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
