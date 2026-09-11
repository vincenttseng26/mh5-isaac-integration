# ZED + ChArUco 相機 / 手眼標定準備

本目錄提供 ZED 相機搭配 ChArUco 板做「相機內參標定」與「手眼標定 (hand-eye calibration)」所需的板圖產生、偵測、pose estimation 架構。**不會**自動連接相機或機器人 —— 所有硬體存取都是明確呼叫對應腳本才會發生,測試套件完全不碰硬體。

沿用並取代 repo 根目錄舊有的 `generate_calibration_boards.py`(該檔案保留未動,僅作為舊版板圖的參照)。舊腳本用的是 OpenCV legacy aruco API(`CharucoBoard_create` / `board.draw`),本目錄的程式對新舊 API 都做了相容處理,詳見下方「已知 OpenCV 問題」。

## 目錄結構

```
camera_calibration/
├── config.py              # BoardConfig dataclass,唯一的板規格來源
├── configs/board_config.yaml
├── compat.py               # OpenCV aruco 新/舊 API 相容層
├── board_generator.py      # 板圖產生(PNG + 鎖定實體尺寸的 PDF)
├── detector.py              # ChArUco 偵測 + pose estimation
├── capture.py               # 影像來源:檔案 / 一般攝影機 / ZED(可選)
├── handeye.py                # eye-in-hand / eye-to-hand 手眼標定求解
├── robot_pose_schema.py    # transport-agnostic 唯讀 robot pose sample schema + 驗證(見「MH5 + FS100 資料收集」)
├── isolation.py              # subprocess 隔離,避免 segfault 拖垮呼叫端
├── diagnose.py                # 環境診斷(找 segfault 常見成因)
├── scripts/                    # 可執行的 CLI 入口
│   ├── generate_boards.py
│   ├── detect_offline.py
│   ├── detect_live.py
│   ├── collect_handeye_samples.py  # 支援 --pose-jsonl 離線 dry-run
│   └── validate_pose_samples.py     # 離線驗證 robot pose sample JSON/JSONL
├── tests/                       # pytest,合成板圖 smoke test,不需相機
└── calib_data/                  # 輸出目錄(板圖、標定結果)
```

## 板規格(config/board_config.yaml)

```yaml
squares_x: 5
squares_y: 7
square_length_m: 0.03     # 30 mm
marker_length_m: 0.0225   # 22.5 mm
dictionary: DICT_6X6_250
legacy_pattern: true
```

所有程式都從這份設定讀值,不要在程式碼裡另外硬編尺寸。若要換板規格,改這份 YAML(或用 CLI 的 `--squares-x` 等參數覆蓋),同時記得**重新列印**並在偵測/手眼腳本上使用相同的 `--config`。

`legacy_pattern`:OpenCV 4.7 以後 `CharucoBoard` 的黑白格排列方式跟舊版 `CharucoBoard_create` 不同。repo 根目錄那組舊板圖是用舊版 API 產生的,所以預設 `legacy_pattern: true` 以維持相容;之後若重新產生並重印新板子,再視情況改成 `false`。

## A2 板規格(獨立第二組板子,`configs/board_config_a2.yaml`)

跟上面的 5x7 板**互相獨立、不互相覆蓋**,實體板來源以 `configs/a2_board_source/`(SVG + `manifest.json` + `provenance.json`,均已用 sha256 核對過)為唯一真實來源:

```yaml
squares_x: 10
squares_y: 8
square_length_m: 0.045    # 45 mm — 設計值,METRIC UNCONFIRMED(見下)
marker_length_m: 0.032    # 32 mm — 設計值,METRIC UNCONFIRMED(見下)
dictionary: DICT_5X5_100
legacy_pattern: false
```

- `legacy_pattern: false` **不是猜的**:把來源 SVG 內嵌的 4500x3600px 光柵板圖抽出來,分別用 `legacy_pattern=True/False` 在本機重繪同尺寸的板子做逐像素比對 —— `True` 有 62.0% 像素不同(排列錯誤),`False` 只有 0.33% 不同(且差異都是邊緣反鋸齒的小雜訊,中位數 92px,遠小於一個完整 45mm 方格的 202500px)。之後又在 cyc-server 實際拍到的 3 張 frame 上做交叉驗證:同一張圖用 `legacy_pattern=True` 一律偵測到 0 個 ChArUco corner,用 `False` 則偵測到 28~54 個(滿板理論值 `(10-1)*(8-1)=63`)。完整數據見 `configs/a2_board_source/provenance.json`。
- **metric-unconfirmed**:`square_length_mm=45.0` / `marker_length_mm=32.0` 目前只是 `manifest.json` 記載的設計值,尚未有人拿尺實測印出來的 100mm 比例尺或 45mm 方格確認印表機是否 100% 原尺寸輸出。在使用者實測回報前,任何用這組設定算出來的度量姿態(pose/hand-eye)一律視為未驗證。
- 讀取時用 `--config configs/board_config_a2.yaml` 明確指定路徑,不要修改 `DEFAULT_CONFIG_PATH`(那個仍指向 5x7 板)。

## 安裝(隔離虛擬環境,不污染系統 Python)

本目錄用 `camera_calibration/.venv` 一個獨立的 venv,系統 Python / 系統 site-packages 完全不動(`pyvenv.cfg` 裡 `include-system-site-packages = false`)。這台機器的系統沒裝 `python3.12-venv`(`ensurepip` 不可用),所以用 `--without-pip` 建立空殼 venv,再用官方 `get-pip.py` 手動裝 pip,全程不需要 `apt install`、不需要 sudo:

```bash
cd camera_calibration
python3 -m venv --without-pip .venv
curl -sS https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py
.venv/bin/python /tmp/get-pip.py
.venv/bin/python -m pip install -r requirements.txt
```

之後所有指令都用 `.venv/bin/python`(或 `source .venv/bin/activate` 之後直接下 `python`),不要用系統的 `python3` —— 系統 `python3` 目前裝的是 `opencv 4.6.0`(見下一節,是本次問題的環境之一,但**不是**唯一根因)。

**實測鎖定版本**(2026-09-08,本機重跑通過):

| 套件 | 版本 |
|---|---|
| Python | 3.12.3 |
| opencv-contrib-python | 4.14.0.94 |
| numpy | 2.5.3 |
| pyyaml | 6.0.3 |
| matplotlib | 3.11.1 |
| pytest | 9.1.1 |

**只裝一個** `opencv-contrib-python`,不要同時裝 `opencv-python` / `opencv-python-headless` / `opencv-contrib-python-headless`。混裝是 `cv2.aruco` native segfault 常見成因之一(見下一節),`python -m camera_calibration.diagnose` 會檢查這件事。

ZED SDK 是可選的:沒裝 ZED SDK / `pyzed` 時,板圖產生、離線偵測、pose estimation 全部可以用一般攝影機或現成影像測試,只有 `--zed` 那條路徑會在建構 `ZedCameraSource` 時丟出清楚的 `RuntimeError`,不會讓其他功能連帶壞掉。

## 已知 OpenCV 問題:4.6 版 cv2.aruco segfault

你提到目前 OpenCV 4.6 的偵測會 segfault。Python 的 `try/except` **無法攔截原生 segfault**(程序在控制權回到 Python 前就已經死亡),所以本目錄採取兩層做法:

1. **診斷**(`diagnose.py`):檢查最常見的成因 —— 同時安裝多個 `opencv-*` pip 套件(例如 `opencv-python` 又裝 `opencv-contrib-python`,或連 `-headless` 版本也混進來)。這些套件的原生 `.so` 會互相覆蓋、造成 aruco 內部符號 ABI 不一致,是實務上最常見的 4.6 segfault 主因。執行:
   ```bash
   python -m camera_calibration.diagnose
   ```
   若報告出現 `warning_conflicting_opencv_packages`,先解決:
   ```bash
   pip uninstall opencv-python opencv-python-headless opencv-contrib-python opencv-contrib-python-headless
   pip install "opencv-contrib-python>=4.8,<5"
   ```
   若版本仍卡在 4.6.x 系列,建議直接升級 —— 4.6 有多起已回報的 `detectMarkers` / `interpolateCornersCharuco` 原生崩潰,4.8+ 沒有重現。

2. **隔離**(`isolation.py`):即使升級後仍要假設某些環境會崩潰,所有「風險呼叫」(板圖 render、marker/corner 偵測)在測試與 `detect_offline.py` 裡預設都透過 `run_isolated()` 丟到用 `multiprocessing.get_context("spawn")` 開的獨立子行程執行。子行程若被訊號(如 SIGSEGV)殺死,父行程會偵測到負的 exit code,回報成 `status="crashed"` 的結構化結果,而不是讓整個 pytest / 腳本一起死掉。用 `spawn` 而非 `fork` 是因為 fork 一個已經初始化過 OpenCV/BLAS 執行緒的行程,本身就是另一類常見的原生崩潰來源。

   `detect_offline.py` 預設就是走隔離路徑;要換成行程內直接呼叫(較快,但崩潰會直接終止腳本)用 `--no-isolate`。

   **已修正的 IPC 死結(與 OpenCV 版本無關)**:實測發現原本的 `run_isolated()` 在資料量較大時(例如一張板圖影像)會 100% 卡到 timeout,而不是真的崩潰。原因是先 `proc.join(timeout)` 整段阻塞、才去 `parent_conn.recv()`:子行程把結果經 `multiprocessing.Pipe` 送回時,若資料超過 OS pipe buffer(Linux 上約 64KB),`Connection.send()` 會卡住等父行程讀取;但父行程要等 `join()` 逾時才會去讀,兩邊互卡,`join()` 永遠等不到子行程結束。用一顆 100x100 的合成 `numpy` 陣列(不含任何 cv2 呼叫)即可穩定重現:陣列夠小時正常回傳,超過 pipe buffer 時每次都 timeout。現已改成一邊用 `parent_conn.poll()` 輪詢、一邊檢查行程存活狀態,資料一到就立刻讀走,不再等 `join()` 逾時。這代表先前回報的「board render 子行程 timeout」主因其實是這個 IPC 死結,不是 OpenCV 4.6 本身的 native crash(不過 4.6 仍建議依上一節升級,不排除同時存在)。

## 板圖產生

```bash
python camera_calibration/scripts/generate_boards.py
# 或自訂輸出目錄 / dpi / 覆蓋規格
python camera_calibration/scripts/generate_boards.py --out-dir camera_calibration/calib_data/boards --dpi 600
```

輸出兩個檔案:

- `*.png`:純板圖(供螢幕顯示、偵測測試用),像素尺寸依照 `squares_x/y * square_length_m` 與指定 dpi 精算,cv2 板繪製時 `marginSize=0`,確保這張圖**完全對應**實體尺寸,沒有 OpenCV 自帶的邊界誤差。
- `*.pdf`:**鎖定實體尺寸、可 100% 列印**的版本。用 matplotlib 的向量 PDF backend,把板圖用絕對物理座標(mm/inch)放進頁面,不受螢幕/印表機 DPI 影響。頁面下方會印一條 **50.00 mm 校驗尺標**,連同文字提醒。

**列印時務必:**
1. 印表機設定選「100% / 實際尺寸(Actual Size)」,**不要**選「符合頁面(Fit to Page)」或「縮放至邊界」—— 這兩者會悄悄改變 square_length,讓後續所有 pose 的公尺數都跟著錯。
2. 印出來後,拿尺實際量頁面下方那條線,必須剛好 50.00 mm。差超過 0.5 mm 就重印。
3. 貼板子到硬平面(壓克力板/鋁板),避免紙張翹曲造成 square_length 局部失真。

## 離線 / 即時 ChArUco 偵測

離線(不需要相機,吃現成影像檔),預設每張圖都在隔離子行程內偵測:
```bash
python camera_calibration/scripts/detect_offline.py --image frame.png
python camera_calibration/scripts/detect_offline.py --dir frames/ --pattern "*.jpg"
```

即時(需要顯示器 + 實體攝影機,人工手動執行,不會被測試或其他腳本自動呼叫):
```bash
# 一般攝影機,不需要 ZED SDK
python camera_calibration/scripts/detect_live.py --camera-index 0

# ZED(需先裝好 ZED SDK + pyzed)
python camera_calibration/scripts/detect_live.py --zed --camera-matrix intrinsics.yaml
```

相機內參 YAML 格式:
```yaml
camera_matrix: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
dist_coeffs: [k1, k2, p1, p2, k3]
```
用 ZED 且沒給 `--camera-matrix` 時,會直接讀 ZED SDK 回報的出廠標定值;要用你自己跑 `cv2.aruco.calibrateCameraCharuco` 得到的精確內參,就把結果存成上面格式傳進去。

## MH5 + FS100 資料收集(唯讀 pose sample 層)

機器人是 Yaskawa MH5,控制器已確認是 **FS100**。本目錄的資料收集層**不連接任何機器人網路協定**——它只定義一個 transport-agnostic 的唯讀 pose sample schema(`robot_pose_schema.py`),未來由一個獨立的 FS100 adapter(不在本目錄範圍內)負責把 FS100 實際吐出來的資料轉換成這個 schema。這裡刻意不猜測、不硬編兩類 FS100 常見但未查證的細節:

1. **關節角度的 pulse↔degree 換算**:FS100 的原始關節回授常是 encoder pulse 數,每軸的 pulse-per-degree 比例存在控制器的絕對值編碼器參數檔(依機型/減速比而定),不是通用常數。`robot_pose_schema.py` 的 `ALLOWED_JOINT_UNITS` 只接受 `"degree"` / `"radian"`,任何 `"pulse"`(或其他原始計數單位)一律在 schema 驗證階段被拒絕,並附上明確錯誤訊息——換算必須在 FS100 adapter 端用已確認的比例完成,不在這一層猜測。
2. **TCP 姿態的旋轉表示**:FS100 常見的 TCP 姿態是位置 + Rx/Ry/Rz 尤拉角,但其軸序(內旋/外旋、XYZ 或 ZYX...)與正負號慣例屬於控制器/文件版本細節,本層未查證前不予假設。`ALLOWED_ORIENTATION_REPRESENTATIONS` 只接受 `"quaternion_wxyz"` 或 `"rotation_matrix_3x3"` 這兩種無歧義表示法,任何 `"euler_*"` 一律被拒絕——轉換同樣必須在 adapter 端用已確認慣例完成。
3. **傳輸協定**:`ALLOWED_TRANSPORTS` 目前只有 `"unconfirmed"` 一個值。FS100 常見的乙太網路存取方式是選購的「Ethernet Function」(High-Speed Ethernet Server,俗稱 HSES,UDP port 10040/10041),但本任務範圍內**不接機器人**,也不驗證這條路徑,所以本層不預設任何特定協定實作,只保留一個明確的「尚未確認」佔位值,並在 sample 上標記 `is_production_ready=False`,提醒使用者這批資料還不能當作真正的手眼標定輸入。

### Robot pose sample schema(`robot_pose_schema.py`)

```json
{
  "schema_version": "1.0",
  "sample_id": "0000",
  "timestamp": {
    "monotonic_ns": 1234567890123,
    "utc_iso8601": "2026-09-08T03:14:15.926535Z"
  },
  "source": {
    "controller": "FS100",
    "robot_model": "MH5",
    "adapter": "<adapter 名稱,待 FS100 adapter 補上>",
    "adapter_version": "0.0.0",
    "transport": "unconfirmed"
  },
  "frame_id": "robot_base",
  "tool_id": 0,
  "user_frame_id": null,
  "joints": {
    "unit": "degree",
    "values": [0.0, -10.0, 20.0, 0.0, 30.0, 0.0]
  },
  "tcp_pose": {
    "position": {"unit": "m", "x": 0.4, "y": 0.0, "z": 0.3},
    "orientation": {"representation": "quaternion_wxyz", "values": [1.0, 0.0, 0.0, 0.0]}
  },
  "raw": null
}
```

欄位說明:

| 欄位 | 必要性 | 說明 |
|---|---|---|
| `timestamp.monotonic_ns` | 必要 | 本機單調時鐘(如 `time.monotonic_ns()`),用來做同一次收集內的順序/時間差檢查,不受 NTP/手動調時影響 |
| `timestamp.utc_iso8601` | 必要 | UTC 牆鐘時間,必須帶時區(`Z` 或 `+00:00`),用來跟外部日誌(FS100 控制器、相機擷取行程)對時 |
| `frame_id` | 必要 | `joints`/`tcp_pose` 所表示的參考座標系名稱(例如 `robot_base`),由 adapter 保證正確 |
| `tool_id` | 必要,非負整數 | FS100 工具編號(拍攝當下生效的 tool no.) |
| `user_frame_id` | 可為 `null` | FS100 使用者座標編號;`null` 代表用機器人/base 座標(未啟用 user frame) |
| `joints.unit` | 必要,只能是 `degree`/`radian` | 見上方「不猜測」說明;`pulse` 一律拒絕 |
| `joints.values` | 必要,長度須為 6 | MH5 是 6 軸機器人 |
| `tcp_pose.position.unit` | 必要,只能是 `m`/`mm` | 明確標單位,不用預設值 |
| `tcp_pose.orientation.representation` | 必要,只能是 `quaternion_wxyz`/`rotation_matrix_3x3` | 見上方「不猜測」說明;`euler_*` 一律拒絕 |
| `source.transport` | 必要,目前只能是 `unconfirmed` | 見上方「傳輸協定」說明 |
| `raw` | 選填 | FS100 原始欄位的 passthrough,僅供除錯/稽核,驗證邏輯不讀它 |

### 離線驗證器

```bash
.venv/bin/python scripts/validate_pose_samples.py --input samples.jsonl
.venv/bin/python scripts/validate_pose_samples.py --input samples.json \
    --frames-dir calib_data/session1 --max-skew-ms 50
```

會逐筆檢查 schema(不合法的表示法/單位/傳輸值直接列出拒絕原因)、跨樣本時間同步(見下方)、以及(給 `--frames-dir` 時)`sample_id` 是否有對應的 `frame_<sample_id>.png`。有任何問題 exit code 非 0。

### 時間同步差檢查

`robot_pose_schema.check_time_sync()`(驗證器與收集器都會呼叫)對排序後的相鄰樣本比較「單調時鐘差」與「UTC 時鐘差」:

- 兩者差距超過門檻(預設 50ms,`--max-skew-ms` 可調)→ 判定為時鐘飄移/不同步,列為問題。
- UTC 時間倒退但單調時鐘前進 → 判定為收集過程中發生時鐘跳動(NTP 校正、手動調時),列為問題。
- `sample_id` 重複 → 列為問題。

這層只檢查 pose sample 檔案內部的時間一致性;pose sample 與相機影像之間的實際同步精度(例如觸發延遲)仍需由收集流程/FS100 adapter 自行量測與記錄。

### Hand-eye collector 的 JSONL dry-run

`scripts/collect_handeye_samples.py` 新增 `--pose-jsonl` 參數,可以完全離線測試收集流程,不接任何裝置:

```bash
.venv/bin/python scripts/collect_handeye_samples.py \
    --dir calib_data/dryrun_session \
    --camera-matrix intrinsics.yaml \
    --mode eye_in_hand \
    --pose-jsonl calib_data/dryrun_session/poses.jsonl \
    --allow-unconfirmed-transport \
    --out calib_data/handeye_result_dryrun.yaml
```

行為:

- 用 `sample_id` 對應 `--dir` 底下的 `frame_<sample_id>.png`(取代預設的 `pose_XXXX.yaml` 逐檔配對)。
- 讀進來的每一筆先過 `robot_pose_schema.RobotPoseSample.from_dict` 驗證,任何未查證表示法/單位/傳輸值直接丟出例外中止,不會靜默略過。
- 呼叫 `check_time_sync`,有問題直接中止,不會拿有時間同步疑慮的資料去解手眼標定。
- 只要有任何樣本 `source.transport == "unconfirmed"`(目前必然如此,見上方),預設拒絕求解;要跑 dry-run 必須明確加 `--allow-unconfirmed-transport`,避免不小心把佔位資料當成正式結果。

### 最少姿態數 / 姿態多樣性 / 時間同步 / eye-in-hand vs. eye-to-hand 對應表

| 項目 | 建議值 | 說明 |
|---|---|---|
| 最少姿態組數 | ≥ 15–20(`solve_hand_eye` 硬性擋 < 10) | 同既有「驗收門檻」章節 |
| 旋轉多樣性 | 至少 3 個以上不共線旋轉軸,相鄰姿態夾角 > 15–20° | AX=XB 方程組需要旋轉變化才能收斂,平移不夠不會補償旋轉不足 |
| 時間同步 | 影像與 pose sample 需在同一次「手臂靜止」窗口內擷取;本層用 `check_time_sync` 抓 pose sample 檔案內部的時鐘飄移,實際擷取延遲需 adapter/收集流程自行量測 | 見上方「時間同步差檢查」 |
| eye-in-hand 對應 | `frame_id` 建議為 `robot_base`,`tcp_pose` 代表 gripper/法蘭在 base 下的 pose;相機隨手臂動,板子固定 | 對應 `handeye.py` 的 `mode="eye_in_hand"`,求解 `cam→gripper` |
| eye-to-hand 對應 | `frame_id` 一樣是 `robot_base`,但 `tcp_pose` 是**夾持板子的** gripper pose;相機固定,板子隨手臂動 | 對應 `mode="eye_to_hand"`,`collect_handeye_samples.py`/`handeye.py` 會自動反轉成 `base→gripper` 再求解,得到 `cam→base` |

### 待 Agy 確認、目前刻意留白的欄位

- `source.transport` 的實際值(例如 FS100 Ethernet Function/HSES 的 port、封包格式,或改用其他管道)——確認後把新值加進 `robot_pose_schema.ALLOWED_TRANSPORTS`。
- `joints` 若要接受 pulse 原始值,需要每軸 pulse-per-degree 比例的確認來源(控制器參數檔匯出或官方文件段落),目前 `ALLOWED_JOINT_UNITS` 沒有開這個口子。
- `tcp_pose.orientation` 若要接受 FS100 原生 Rx/Ry/Rz 尤拉角,需要確認軸序與正負號慣例(並附引用來源),屆時應新增一個帶版本/日期的明確 literal(例如 `euler_fs100_<confirmed_convention>_<date>`),而不是開放一個籠統的 `"euler"`。
- `tool_id` / `user_frame_id` 的實際編號範圍與本次標定要用哪一組,需要 Agy 確認現場設定。

## Eye-in-hand vs. Eye-to-hand 資料收集

**Eye-in-hand**:相機鎖在機器人末端,板子固定在世界(桌面)上。相機隨手臂移動,拍到板子的相對姿態會隨手臂姿態變化。

**Eye-to-hand**:相機固定在世界(三腳架/機構上),板子鎖在機器人末端。手臂帶著板子在相機視野內移動到多個姿態。

兩種情況共用同一套資料收集流程:

1. 用 `detect_live.py` 或你自己的擷取程式,在**手臂每次移動到新姿態、完全靜止後**拍一張影像存成 `frame_XXXX.png`。
2. 同一瞬間,從機器人控制器讀出**末端(gripper)在機器人 base 座標系下的 pose**,存成對應的 `pose_XXXX.yaml`(格式見 `scripts/collect_handeye_samples.py` 檔頭註解,支援 4x4 矩陣或 translation+quaternion 兩種寫法)。
3. 姿態要有足夠旋轉多樣性(至少涵蓋 3 個以上不共線的旋轉軸,相鄰姿態旋轉角度建議 > 15–20°),否則 `cv2.calibrateHandEye` 的 AX=XB 方程組會病態、解出來的外參不穩定。
4. 至少收集 **15–20 組**(`solve_hand_eye` 預設 `min_samples=10` 會擋掉過少的樣本,但實務上建議更多以平均掉雜訊)。
5. 執行求解:
   ```bash
   python camera_calibration/scripts/collect_handeye_samples.py \
       --dir calib_data/session1 \
       --camera-matrix intrinsics.yaml \
       --mode eye_in_hand \
       --out calib_data/handeye_result.yaml
   ```
   `--mode eye_to_hand` 時,程式會自動把讀進來的 `gripper→base` pose 反轉成 `base→gripper` 再丟進 `cv2.calibrateHandEye`(這是 OpenCV 官方文件建議的作法),解出來的是 `cam→base`(相機在機器人base座標系下的位置),而不是 `cam→gripper`。

## 輸出座標與座標系慣例

- **相機座標系**:OpenCV 慣例,原點在相機光心,X 右、Y 下、Z 朝前(沿光軸)。
- **`rvec`/`tvec`**(`estimate_pose_charuco` 回傳):`rvec` 是 Rodrigues 旋轉向量,`tvec` 是公尺為單位的平移,兩者合起來描述 **板子座標系相對於相機座標系** 的 pose(`target→cam`)。板子座標系原點在板子第一個(左上角)ChArUco 角點,X/Y 沿板面,Z 垂直板面朝外。
- **手眼標定結果**(`HandEyeResult`):
  - `eye_in_hand` 模式 → `cam→gripper`:相機相對於機器人末端法蘭的固定變換。
  - `eye_to_hand` 模式 → `cam→base`:相機相對於機器人 base 的固定變換。
  - 輸出檔(`calib_data/handeye_result.yaml`)同時給 `matrix_4x4`(齊次矩陣,公尺)與 `quaternion_xyz`(`qw,qx,qy,qz` + `x,y,z`,方便丟進 ROS/機器人控制器)。

## 驗收門檻(建議預設值,依應用精度需求自行調整)

| 項目 | 建議門檻 | 說明 |
|---|---|---|
| 單張影像 ChArUco 角點數 | ≥ 6(`min_corners` 參數),建議 ≥ board 內角點總數的一半(此板 24 個內角點,建議 ≥ 12) | 角點太少時 PnP 解不穩定 |
| 相機內參標定重投影誤差(`cv2.aruco.calibrateCameraCharuco`) | < 0.5 px 優、< 1.0 px 可接受,超過需重新標定 | 直接影響後續所有 pose 的精度上限 |
| 列印尺寸校驗 | 50 mm 校驗線量測誤差 < 0.5 mm | 見「板圖產生」章節 |
| 手眼標定樣本數 | ≥ 15–20 組,旋轉軸多樣 | 樣本太少或旋轉太相近會讓 AX=XB 病態 |
| 手眼標定殘差(自行用留一法或重投影驗證) | 平移殘差 < 2–3 mm、旋轉殘差 < 1° | 本目錄目前不含自動殘差驗證腳本,需要的話再擴充 |

## 測試

```bash
cd camera_calibration
PYTHONPATH="$(cd .. && pwd)" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests/ -v
```

**務必用 `.venv/bin/python -m pytest`,不要直接下裸的 `pytest`**,且**務必加上這兩個環境變數**,原因是這台機器(以及任何裝了 ROS 2 的機器)全域 `PYTHONPATH` 會被 ROS 工作空間(例如 `/opt/ros/jazzy/...`、`~/ros2_ws/install/...`)污染。這件事會造成兩個獨立問題:

1. **`import camera_calibration` 需要 repo root 在 `sys.path` 上**,所以仍要顯式帶 `PYTHONPATH="$(cd .. && pwd)"`(或直接用 repo 絕對路徑),不能單靠 venv 本身。
2. **`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 是必要的**:ROS 2 的 `/opt/ros/jazzy/lib/python3.12/site-packages` 裡的 `launch_testing` 套件註冊了一個 `pytest11` setuptools entry point,只要它在 `PYTHONPATH` 上,pytest 啟動時就會嘗試自動載入這個外部 plugin ——這一步發生在 pytest 讀我們的 `conftest.py`/測試檔**之前**,而 `launch_testing` 這個 plugin import 時需要 `lark` 套件;這台機器的系統 Python 沒裝 `lark`,於是整個 `pytest` **在收集任何測試之前**就直接丟 `ModuleNotFoundError` 掛掉,跟 `camera_calibration` 本身的程式碼或 OpenCV 版本完全無關。已用不含任何 camera_calibration 程式碼的最小重現(裸跑 `pytest` vs. 加上 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`)在本機驗證過這個因果關係。因為問題根源在全域環境變數而不是本目錄的程式碼,修法就是**跑測試時一律帶上這兩個環境變數**,而不是去改動 ROS 安裝或系統 site-packages。

測試不連接任何相機或機器人,只用合成產生的板圖影像(以及若 repo 根目錄剛好有現成的 `ChArUco_Board_5x7_30mm.png` 就順便測一下,沒有就自動跳過)。板圖 render 與偵測都跑在獨立 subprocess(`isolation.run_isolated`)裡:如果你的 OpenCV build 會 segfault,對應的測試會**以清楚的失敗訊息結束**(並附上 `diagnose.py` 的環境報告),而不會讓整個 `pytest` 行程跟著崩潰。

**本機實測結果(2026-09-08,`camera_calibration/.venv`,opencv-contrib-python 4.14.0.94)**:`40 passed in <1s`,exit code 0(含本輪新增的 `test_robot_pose_schema.py`、`test_collect_handeye_dryrun.py`、`test_validate_pose_samples.py`,全部是合成資料,不接相機或機器人)。

## 已驗證的板圖輸出

用隔離環境實際跑過一次板圖產生 + 偵測 smoke test(合成影像,非拍照,不碰任何相機):

```bash
.venv/bin/python scripts/generate_boards.py --out-dir output --dpi 600
```

- `output/charuco_5x7_30mm.png`:4961x3543 px(=squares_y/x × 30mm × 600dpi,與規格精算值完全一致)。
- `output/charuco_5x7_30mm.pdf`:單頁 PDF,`pdfinfo` 量測頁面尺寸 510.236 x 765.354 pts,換算回 mm 正好等於「板子 150mm x 210mm + 左右上邊界 15mm + 下方 30mm 頁尾」的設計值,確認是鎖定物理尺寸輸出,非隨螢幕/印表機 DPI 縮放。
- 合成影像偵測(`detect_in_subprocess`):`num_markers=17`、`num_charuco_corners=24`(=`(5-1)*(7-1)`,滿板全部內角點都偵測到)。
- 板規格確認:5x7 squares、`square_length=30.00mm`、`marker_length=22.50mm`、`dictionary=DICT_6X6_250`、`legacy_pattern=True`。

## A2 板:cyc-server 既有 probe frame 的離線偵測結果

用 `scripts/detect_a2_probe_report.py` 對 cyc-server 上**既有**(未重新拍攝、未動相機設定)的 `calib_data/probe/frame_01.png`~`frame_03.png` 做離線偵測,結果與腳本同步寫回 `calib_data/probe/a2_probe_summary.json`:

| frame | markers | charuco corners(滿板理論 63) | legacy_pattern=False vs True(corners) | 錯誤字典(`DICT_6X6_250`)控制組 |
|---|---|---|---|---|
| frame_01 | 26 | 28 | 28 vs 0 | markers=0(正確拒絕) |
| frame_02 | 35 | 49 | 49 vs 0 | markers=0(正確拒絕) |
| frame_03 | 37 | 54 | 54 vs 0 | markers=0(正確拒絕) |

- 三張圖的 `legacy_pattern` 交叉檢查全部支持 `False`(跟 SVG 像素比對的結論一致),`legacy_pattern=True` 在真實拍攝影像上一律偵測到 0 個 corner。
- 錯誤字典控制組(同一張圖、同樣板幾何,只把 `dictionary` 換成舊 5x7 板的 `DICT_6X6_250`)在三張圖上都正確拒絕(0 markers),證明前面的偵測結果是字典匹配的結果,不是偵測器過度寬鬆誤判。這也解釋了為什麼**先前**用舊 5x7 config(`DICT_6X6_250`)對同一批相機畫面做的 probe(`logs/probe_stage2_camera.log`)會回報 `markers=0 corners=0` —— 那是因為當時用錯字典/板規格對到了這塊 A2 板,並非相機或偵測流程本身有問題。
- Overlay 圖(`frame_0X_a2_overlay.png`)另存新檔,不覆蓋原本 5x7 probe 留下的 `frame_0X_overlay.png`。
- `metric_status: metric-unconfirmed` 已寫入 `a2_probe_summary.json`,原因同上一節。

## 未解風險 / 待辦

- `cv2.aruco` 在极端環境下仍可能在**渲染板圖**(不只是偵測)時崩潰;目前 `board_generator.render_board_png` 只有在測試裡走隔離路徑,`scripts/generate_boards.py` 本身沒有走 `isolation.py`(因為那是一次性、人工執行的 CLI,崩潰時終端機會直接看到 traceback/Segmentation fault,不需要額外包裝)。若你想讓它也安全失敗,可以仿照 `detect_offline.py` 的寫法包一層。
- 手眼標定殘差驗證(留一法交叉驗證、重投影一致性檢查)目前只有門檻建議,沒有對應的自動化腳本,需要的話可以再加。
- ZED 內參(`ZedCameraSource.get_left_intrinsics`)讀的是 SDK 回報的出廠標定,不是你自己跑 ChArUco 標定的結果;高精度應用建議自行標定後用 `--camera-matrix` 覆蓋。
- 目前只在這台機器(ROS 2 Jazzy + Python 3.12.3 + opencv-contrib-python 4.14.0.94)驗證過;若換到別台機器,仍請先跑 `.venv/bin/python -m camera_calibration.diagnose` 確認沒有多套 opencv-* 混裝的警告。
