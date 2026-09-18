# Isaac Sim × Motoman MH5 整合檢查清單

最後更新：2026-09-11

## 0. 目前可重現硬體基線（2026-09-11）

- [x] Isaac Sim 5.0.0：RGB live 與已校正彩色點雲可顯示。
- [x] CYC host `192.168.50.10`：arm `8769`、MoveIt `8770`、gripper `8768` 可連線。
- [x] ZED sender 由 CYC loopback `8765` 經 SSH tunnel 提供給 Isaac PC。
- [x] 實體 MH5 staged `pre_grasp → grasp → close → lift → home` 已成功驗證。
- [x] 向下開合夾爪 grasp waypoint 已加入 `+18 mm` 高度補償。
- [ ] Isaac Sim → 實體 MH5 雙向控制（刻意未啟用，待安全控制器與低速空載測試）。

這份檔案是雙電腦 Isaac Sim／實體 MH5 整合的唯一進度清單。只有在實際完成並驗證後，才將 `[ ]` 改成 `[x]`；單純建立檔案、看到節點或啟動程式不算完成。

安全原則：目前維持 `實體 → Isaac` 的只讀鏡像。除非「實機安全閘門」章節全部完成並經人工確認，不得建立 Isaac 到實體手臂的命令路徑。

## 1. 雙電腦與只讀鏡像基線

- [x] 實機電腦與 Isaac 電腦以專用 Ethernet 直連。
- [x] 實機電腦使用 `192.168.50.10/24`，Isaac 電腦使用 `192.168.50.20/24`。
- [x] FS100 專用網路保持獨立：實機電腦 `192.168.0.107/24`、FS100 `192.168.0.11/24`。
- [x] 兩台電腦互相 ping 成功，FS100 也可由實機電腦 ping。
- [x] SSH 僅綁定 Isaac 專線，使用受限公鑰登入。
- [x] Fast DDS、`ROS_DOMAIN_ID=42` 與網卡白名單設定完成。
- [x] Humble 實機電腦與 Jazzy Isaac 電腦完成雙向 heartbeat 實測。
- [x] Isaac Sim 5.1 使用 pip venv 與內建 Jazzy ROS libraries，未混入 system Python 3.12 路徑。
- [x] Isaac ROS 2 Bridge 可成功載入 internal `rclpy`。
- [x] 實機端只啟動 ROS 1 `/joint_state`，沒有 motion streaming、trajectory action、I/O relay 或 `/robot_enable`。
- [x] ROS 1 `/joint_states` 經隔離名稱 `/real/joint_states` 傳到 ROS 2。
- [x] Isaac read-only mirror 只訂閱 `/real/joint_states`，沒有 ROS publisher、service client 或 `/clock`。
- [x] 透過教導盒移動實體 MH5 時，Isaac 虛擬六軸會同步移動。
- [x] 已驗證六軸名稱與順序：`joint_1_s` 至 `joint_6_t`。
- [x] 已驗證鏡像不會控制 6 個 RG2-FT follower DOF。
- [x] 已建立可重現的啟動腳本與停止方式。

## 2. 固定目前可工作的基線

- [x] 將目前專案變更整理成 Git commit，避免後續修改破壞已通過的 mirror。
- [x] 記錄並保存 clean USD 五個 layer 的 SHA-256。
- [x] 將 clean URDF、xacro、mesh 與 USD 複製到第二個備份位置。
- [x] 記錄 Isaac Sim、extension、GPU driver、ROS 及 bridge image 的固定版本。
- [x] 做一次重開兩台電腦後的冷啟動測試，確認所有 helper 仍可重現結果。

Isaac read-only mirror／`sim_train` 基線已固定於 Git commit `f83be7f`。2026-09-04 已獨立審查 `README.md` 內較早且來源混合的改寫：修正 ZED2 誤標為實測的第一代 ZED，移除 `/robot_enable`、`bridge-all-topics`、hardware launch 與 MTC 的可複製動作指令，並改為只讀 mirror 安全邊界與受控文件連結。回歸測試會阻擋這些未審查指令再次出現於 README code block。因目前工作樹還有其他未提交驗證變更，不自動代替操作員建立混合 commit。

2026-08-20 冷啟動第一次嘗試偵測到另一個 agent 啟動完整 ROS 1 motion runtime 並呼叫 `/robot_enable`；操作員解除 Servo、停止並行 agent 後，新增 host／Docker／ROS 1 preflight 與每 `0.5 s` runtime guard。以 `--network none` 假 motion process 驗證 guard 會拒絕啟動並中止運行中的 bridge，最後重跑 heartbeat、read-only state、固定 bridge、Isaac applied 與 clean shutdown 均通過。證據見 `docs/validation/mh5-cold-start-20260820.json`。

## 3. ZED 相機數位分身

- [x] 確認實體相機的精確型號、解析度、FPS 與左右眼 intrinsics。
- [x] 保留 clean robot USD 不變，建立獨立的 camera／scene composition layer。
- [x] 將現有 `base_link → zed_left_camera_frame` eye-on-base 標定轉成 USD camera 座標軸。
- [x] 在 Isaac 中加入 ZED 左眼、右眼或深度相機 prim。
- [x] 驗證相機位置、朝向、視野與影像上下左右方向。
- [x] 建立 `/sim/camera/...` namespace，避免與實體 `/real/camera/...` topic 衝突。
- [x] 驗證 synthetic RGB、depth、camera_info 與 point cloud。
- [x] 使用已知標定板或固定物體，比較實體與虛擬相機投影誤差。

2026-08-20 使用四個 `DICT_4X4_50` Marker 建立 `481 × 375 mm` 中心距離矩形；四點位於同一水平面，Marker 中心距桌面約 `340 mm`。實體 ZED HD1080 影像與 Isaac disposable validation layer 的四點中心投影誤差為 `16.10 px RMSE`、`20.22 px max`，通過預先設定的 `20 px RMSE／30 px max` 門檻。Isaac 測試期間 ROS 2 Bridge 關閉，production camera scene SHA-256 前後皆為 `694512f66c0f60e239010ee27813c47761c7b58a7657c2f7b8940e968292c926`。因 `340 mm` 是約值，且板面 X／Y／yaw 仍由同一張實體影像擬合，此結果證明跨引擎投影一致性，不宣稱完成獨立的 eye-on-base 絕對精度驗收。證據見 `docs/validation/zed-physical-virtual-projection.json`。

2026-08-21 以 ZED SDK 唯讀列舉與 USB descriptor 確認序號 `24180` 是第一代 Stereolabs `ZED`（USB `2b03:f582`），不是 ZED 2。依官方 `zed.stl` 的 bounds 在標定 frame 下加入機身與雙鏡頭 visual；五個 visual prim、`120.006 mm` stereo baseline、近拍 render 及 scene immutability 均通過。因支架接觸尺寸未量測，相機外觀維持無碰撞。證據見 `docs/validation/zed-physical-model.json` 與 `docs/validation/mh5-physical-workcell-build.json`。

2026-09-08 完成實體 ZED（序號 `24180`）到 Isaac Sim 的唯讀即時 RGB／彩色點雲串流與 ChArUco A2 固定場景校正。實體板規格為 `DICT_5X5_100`、`10 × 8` squares、square `45 mm`、marker `32 mm`；清除板上遮擋物後，30/30 幀有效，平均 `48.87/63` 個 ChArUco corners，平移最大偏差 `4.08 mm`、旋轉最大偏差 `0.938°`、重投影 RMS 最大 `0.574 px`，通過既定 `5 mm / 1° / 1 px` 門檻。桌面平面與兩條可見桌邊另以剛體補償對齊 Isaac 桌面；操作員先接受 world-Y `-50 mm` 的 A/B 結果，再以 30 幀 ChArUco 幾何中心平均值消除剩餘 `+2.684 mm` Y 偏差，全程未改旋轉、X 或 Z。後續唯一允許的 viewer 校正輸入為 Isaac PC 的 `mh5_isaaclab_reach_lift_v0/zed_tabletop_edge_alignment_charuco_centered_20260908.json`；禁止退回舊版或省略 `--tabletop-correction`。RGB 只顯示於獨立 2-D UI，3-D viewport 僅保留 metric point cloud。此成果是 camera-to-workcell 感知基線，不代表完成 FS100 hand-eye 或允許實機動作。

## 4. 機器人物理模型

- [ ] 取得或量測 MH5、quick changer、角鐵與 RG2-FT 的可信質量和慣量。
- [ ] 取代目前由 Isaac 依幾何推算的 mass/inertia，並保留來源紀錄。
- [x] 檢查所有 link 的 collision mesh、縮放、穿模與 self-collision。
- [x] 驗證六軸 joint limit、velocity limit、effort limit 與 URDF／MoveIt／Isaac runtime 一致。
- [x] 驗證 RG2-FT `finger_joint` 與五個 mimic follower 的方向與比例。
- [x] 定義 SRDF `home` `[0, 0, 0, 0, 0, 0.105]` rad，驗證載入與 reset 可重現。
- [x] 在獨立 training layer 設定 drive stiffness/damping；不修改 clean import 基線。
- [x] 以單軸與六軸小步階測試 drive gain，未出現震盪、爆衝或非有限狀態。

靜態稽核已確認 32 個 mesh URI、scale 與有限非零 bounds，以及 16 個 Isaac collision rigid body。2026-08-21 另在 session layer 啟用 self-collision，依 SRDF 套用 35 組 reviewed filtered pair，對其餘 85 組 pair 執行 seed `24180` 的 128 姿態掃描；home 無意外接觸，所有狀態有限且 production scene SHA 前後一致。掃描也如實找出 6 組 base 與 wrist／tool 的意外接觸、4 個 self-collision samples 與 24 個工作站接觸 samples，因此「joint-limit 內」不能視為「collision-free」，這些姿態必須由未來 planner／gateway 拒絕。完整報告 SHA-256 與摘要見 `docs/validation/mh5-self-collision-sweep.json`。MH5 七個主要 link 在原始 URDF 仍沒有可信 inertial，詳見 `docs/isaac-physics-audit.md`。

2026-08-21 實體銘牌已視覺確認為 `MOTOMAN-MH5F / YR-MH0005F-A00`，整機 mass `27 kg`、payload `5 kg`。這排除了機型歧義，但銘牌沒有逐 link mass／CoM／inertia；原圖 bytes 也仍需放入 workspace 並固定 SHA-256，故上方兩項動力學工作不能勾選，更不能把 27 kg 任意分配到七個 links。

2026-09-04 RG2-FT V2 銘牌已人工讀取為整組重量 `0.98 kg`與 payload `2 kg`。該重量可當作夾爪 assembly-level domain-randomization constraint，但不提供逐 link mass、CoM 或 inertia；圖片原始 bytes 也尚未進入 workspace。因此工具端 links 仍維持 `existing_unsourced`，上方兩項不勾選。衍生證據見 `docs/validation/rg2ft-nameplate-observation-20260904.yaml`。

Quick changer 由操作員初步辨識為 OnRobot `QC-R v3`。原廠 datasheet 確認 robot side `0.06 kg / 0.13 lb`、tool side `0.14 kg / 0.31 lb`，兩側約 `0.20 kg`；`130 g` 不是表格中的規格。因尚未有實體銘牌或裝配邊界 artifact，registry 身分狀態仍為 `operator_reported_pending_artifact`，上方兩項不勾選。

## 5. 明確分離啟動模式

- [x] `mirror`：實體 joint state 單向更新 Isaac，無任何實機命令。
- [x] `sim_train`：Isaac 自己控制虛擬手臂，不啟動實機 driver／bridge。
- [ ] `shadow`：策略讀取實機狀態並計算 action，但 action 只送到 shadow simulation 和 log。
- [ ] `real`：只有經安全閘門與人工 armed 後，才允許已驗證 action 送到實機。
- [ ] 為四種模式建立不同 launcher、namespace 與明顯的終端提示。
- [ ] 加入互斥檢查，禁止 `sim_train` 或未 armed 的 Isaac process 接觸實機 command interface。

目前 `start_sim_train.sh` 會清除 ROS／DDS 環境、拒絕與另一個 Isaac process 同時啟動，並在 Python 端強制關閉 ROS 2 Bridge；`run_mirror_bridge.sh` 也會在啟動前及執行期間拒絕 motion-capable host、Docker 與 ROS 1 interface。四模式共用的總體 command gateway 尚未建立，因此最後兩項維持未完成。

## 6. Isaac 模擬控制能力

- [x] 建立只控制六軸的虛擬 articulation controller。
- [x] 建立獨立的 RG2-FT 主關節控制，讓 mimic joints 正確跟隨。
- [x] 對 position/action 做 finite、名稱、順序、joint-limit 與每步最大變化驗證。
- [x] 在模擬中完成單關節 `+0.03 rad` 小角度動作測試。
- [x] 在模擬中完成六軸 home pose 往返測試。
- [x] 在模擬中完成無碰撞的測試軌跡。
- [x] 驗證 Stop／reset 後 controller 可恢復且不跳動。

2026-08-21 將實測工作站接入平行的 `sim_train_workcell`。16 個 robot rigid body 套用 PhysX Contact Report API；四個受 `0.05 rad` step guard 限制的 motion phase 最大誤差為 `3.58e-6 rad`，期間 robot contact event 為 `0`、disallowed contact 為空，reset 最大 home 誤差為 `5.22e-7 rad`，因此這一組受限測試軌跡可判定無碰撞。這不代表完整 pairwise self-collision sweep、完整 workspace 或任意未來軌跡已驗收。

## 7. 工作場景與感知

- [ ] 建立地面、工作台、底座與固定障礙物的正確尺寸和座標。
- [x] 匯入第一個訓練物體，設定 collision、`0.25 kg`、靜摩擦 `0.6`、動摩擦 `0.5`。
- [x] 設定燈光、材質與背景，建立可重現的基準場景。
- [x] 驗證相機看到的物體位置與 robot base/world frame 一致。
- [x] 建立 synthetic RGB-D／point-cloud 資料流程。
- [x] 驗證模擬點雲可被目前 point-cloud repair pipeline 讀取。

2026-08-21 已記錄桌板 `2000 × 1200 × 66 mm`、桌面離地 `805 mm`、兩台 MH5 底座中心距 `1200 mm`，以及固定相機鋁架的立柱與橫樑尺寸／位置。simulation-only 工作站在 Isaac PC 建置及 collider preflight 通過，ROS 2 Bridge 關閉、禁止介面掃描為空，production ZED SHA-256 前後不變；GUI 目視確認桌板／鋁架方向正確。故障 MH5 因關節姿態不可讀，以手機量得目前鎖定姿態約 `393.7 mm` 水平半徑、`1066.8 mm` 高，再加入裕度形成 `450 mm × 1150 mm` 保守碰撞圓柱；目視確認未與可用手臂重疊。第一代 ZED 外觀已依官方 mesh bounds 加入；由於故障手臂精確姿態、桌架及相機安裝支架接觸尺寸仍未知，第一項維持未勾選。證據見 `docs/validation/mh5-physical-workcell-build.json` 與 `docs/validation/zed-physical-model.json`。

平行的 `sim_train_workcell` 已 reference 含 ZED 外觀的實測 layer，不覆寫舊 `sim_train`。五個 physical collider、受限 motion/contact/reset、target cube 與 ZED render 均通過；重建後 target 投影誤差 `5.54 px`、深度誤差約 `0.0602 m`。證據見 `docs/validation/mh5-sim-train-workcell-build.json`、`docs/validation/mh5-sim-train-workcell-smoke.json` 與 `docs/validation/mh5-sim-train-workcell-perception.json`。既有隔離輸出 `/sim/perception/repaired_object_pc` 曾收到 8,634 個有限點。未啟動 grasp planner、MTC 或 MoveIt execution。

## 8. 訓練環境

- [x] 明確決定第一個目標：MoveIt 規劃、模仿學習，或 reinforcement learning grasping。
- [x] 定義 observation、action、控制頻率、episode reset 與成功條件。
- [x] 定義 reward／loss，避免鼓勵碰撞、超限或不必要的大動作。
- [x] 建立正式 Isaac Lab RL environment（`DirectRLEnv`，含 per-episode dones／reset 與 contact-sensor 驅動的碰撞終止）。
- [x] 加入物體位置、摩擦、質量、光照、相機雜訊與控制延遲 domain randomization。
- [x] 建立固定 seed 的 smoke test，確認環境可重現。
- [x] 完成第一個 baseline policy／planner。
- [ ] 在未見過的模擬場景評估成功率、碰撞率與 joint-limit violation。
- [ ] 保存模型、設定、版本、seed、metrics 與訓練 log。

第一個目標固定為 `mh5_rg2ft_reach_lift_v0` reinforcement-learning grasping 的 reach-and-lift prerequisite。契約定義 27 維 observation、7 維 joint-delta action、`120 Hz` physics／`20 Hz` control、reset、success、termination 與 collision／joint-limit 負 reward。Isaac Lab `2.3.1` 在 Isaac Sim `5.1.0` 以 8 個平行場景、seed `24180` 跑完固定種子 smoke；相同 seed 完全重現、不同 seed 改變 reset。物體位置、mass、friction／restitution完成 runtime readback；隨機 dome-light intensity 寫入 stage 並讀回；camera-derived task-space position noise 確實改變 observation；0–2 step control delay 的首次 action 套用步數逐環境吻合。cube 與 home 穩定且無超限。

2026-08-21 正式 `DirectRLEnv`（`transfer/isaac_pc_setup/mh5_reach_lift_env.py`）已在 Isaac Lab `2.3.1` 完成模擬端驗收。8 個平行環境、seed `24180`，實測涵蓋 27 維 observation、7 維 action、全部 8 個 YAML reward term、per-episode dones 與 `_reset_idx`：NaN action 與 joint-limit 探測各自只終止並重置該環境（`episode_length_buf` 歸零），其餘 6 個環境不受影響。四個固定碰撞姿態全部重現 6 組已審查的 disallowed pair 並觸發終止，且沒有任何 SRDF 已審查（允許）pair 被誤判為 disallowed。三個獨立 Kit process 的驗證顯示相同 seed 逐欄位完全一致、seed `24181` 改變隨機化且同樣通過。ROS 2 Bridge 關閉、stage 無禁止介面、四個輸入 artifact 的 SHA-256 前後不變。證據見 `docs/validation/mh5-direct-rl-contract-smoke.json`。

本輪修掉兩個只在正式 env 才會顯現的缺陷。其一，`_reset_idx` 原本只把被 reset 的子集傳給 `root_physx_view.set_masses()`，但 `omni.physics.tensors` 的 setter 要求整個 view 的緩衝區加 indices，因此第一次發生「部分環境結束」就會拋 `Incompatible size of mass tensor`；batch smoke 每次都整批寫入，所以驗不出來。其二，Isaac Lab 的 `SimulationContext` 會無條件把 `/physics/disableContactProcessing` 設為 `True`，只有建立 `ContactSensor` 時才會關掉——在此之前 PhysX 仍會排斥穿透（關節被推出限位），但**完全不派送接觸報告**，等於整個 fail-closed 碰撞政策靜默失效而外觀正常。修法是改用 Isaac Lab `ContactSensor` 並在啟動後斷言該旗標為 `False`，兩者都已加上原始碼層級迴歸測試。

因語料庫姿態是直接 teleport 進入深度穿透，PhysX 排斥過程會讓 RG2-FT 子連桿再撞出 11 組額外的 disallowed pair。這些在報告中如實列為 `additional_depenetration_pairs`，不做抑制；驗收條件因此是「6 組已審查 pair 全數重現且不得誤判允許 pair」，而非集合相等。

因 MH5 inertia 未經量測，v0 仍明確關閉 robot-link gravity、保留 object gravity，不宣稱真實 torque dynamics。光照目前是每 batch 共用，camera noise 是 task-space observation noise 而非 raw RGB/depth pixel noise。尚未實作 trainer interface，zero-action 只算環境 smoke，不冒充 baseline policy。設定與證據另見 `config/training/mh5_reach_lift_v0.yaml`、`docs/validation/mh5-isaaclab-batch-smoke.json`。

2026-09-04 已加入獨立、opt-in 的 gravity-on provisional 路徑，不改變上述 v0 smoke。`mh5_randomized_dynamics_v0.yaml` 將 MH5 `27.0 kg` 與 RG2-FT `0.98 kg` 當作 assembly-level 精確總量約束，對未知的逐 link mass／CoM／等效 box inertia 與 actuator gains 做 bounded randomization；QC-R v3 robot side 使用 `0.058–0.062 kg`。1,000 個離線 seeds 已通過有限值、bounds、正定 inertia、triangle inequality、精確總量與 reproducibility 測試。另已實作 simulation-only DLS `open → approach → descend → close → verify → lift` 候選 baseline；`verify` 以短距離 probe 確認物體真的隨 TCP 上升，空抓會 fail closed。runner 會清除 ROS／DDS、停用 Bridge 並檢查 artifact immutability。由於尚未在 Isaac PC 取得 `8/8` 抬升成功、零碰撞／限位／NaN／timeout 的 runtime report，「完成第一個 baseline」仍維持未勾選；domain randomization 也不會把 strict inertial readiness 改成通過。

## 9. Shadow mode 與 sim-to-real 驗證

- [ ] 讓策略讀取 `/real/joint_states`，但 action 只套用到 shadow robot。
- [ ] 記錄策略 action、實機狀態、shadow 狀態、延遲與拒絕原因。
- [ ] 比較 shadow 預測與實際示教器動作，確認 joint sign/order/units。
- [ ] 測量 DDS、bridge、策略 inference 與控制迴圈延遲／jitter。
- [x] 模擬封包中斷、舊 timestamp、NaN、超限與節點當機。
- [x] 確認 timeout/watchdog 發生時 action 會被丟棄，而不是沿用舊命令。

目前完成的是 transport-free fault matrix：21/21 fake scenarios 通過，涵蓋 disarmed、wrong order/unit、NaN、舊／未來 timestamp、joint limit、step、acceleration、rate limit、workspace／collision callback rejection、disconnect、E-stop、robot disable、dead-man timeout、restart 與 watchdog。watchdog 後 fake sink 保持只有原本一筆 target，後續 intent 因 disarmed 被拒絕，沒有重播舊命令。這不包含 live `/real/joint_states`、DDS／bridge latency 或 shadow-vs-real 比對，所以前四項仍維持未完成。證據見 `docs/validation/mh5-offline-gateway-fault-matrix.json`。

2026-09-08 建立 `read_only_mh5_mirror.py` 的 shadow extension 與 `start_shadow_mh5.sh`：只訂閱 `/real/joint_states`，本地 JSONL action 經六軸、rad、`0.01 rad` step guard 後只套用 Isaac shadow articulation，並寫入 `shadow_logs/*.jsonl`；明確記錄 `command_authority=shadow`、`real_transport_touched=false`。兩筆 smoke action 的離線驗證 PASS；用完整 Isaac Jazzy/FastDDS launcher 做 live preflight 時，Isaac articulation 與 ROS bridge 初始化 PASS，但測試期間未收到有效 `/real/joint_states`，故尚未宣稱 live shadow 通過，也未建立任何 FS100 command path。證據與腳本位於 Isaac PC `mh5_isaaclab_reach_lift_v0/start_shadow_mh5.sh`、`validate_shadow_actions.py` 與 `shadow_logs/`。

### 2026-09-05 scripted baseline 修正（等待 Isaac runtime）

本機程式審查修正 DLS measured-position displacement 被累加到舊 joint target 的追蹤落後累積問題；approach／descend 現在需姿態誤差不超過 5 度才允許下一階段；夾爪發出的 delta 與內部 capped target 一致；runner 對 aborted／completed lanes 清除 action 與 smoothing。5 度是待 runtime 評估的 simulation-only 門檻，不是實機校正驗收值。未改動 actuator gains、TCP offset 或成功判定。

離線測試通過：randomized dynamics／scripted controller 18 項（包含實際 DLS tensor method 的 stalled-joint 回歸）、DirectRL contract 19 項、integration 24 項。這些不構成抓取成功證據。當時 Isaac PC `192.168.50.20` SSH 逾時、本機專線網卡 DOWN，且本機未安裝 Isaac Lab；尚未同步或執行新版 runtime。夾爪無法張開的實際原因仍待查。恢復連線後應先保存遠端現有設定／報告並比對差異，再驗證 reset hold、開合與 mimic、姿態對正、完整 8-env baseline；「完成第一個 baseline」維持未勾選。

### 2026-09-05 runtime 續查（姿態對正穩定性未通過）

恢復 Isaac PC SSH 後，先將既有 runtime 目錄封存為 `/home/vincent/Desktop/mh5_isaaclab_reach_lift_v0_pre_sync_20260905T105354.tar.gz`（SHA-256 `874442892217cfb1b8c81b368bca14ce1df2cffdebb027d88c5c6de079b80943`），再同步本機已測試的 controller、runner 與 environment。昨晚最新 `grasp_verify_visual.json` 證明夾爪可開合且五個 mimic follower 同向運作；失敗已縮小為短暫雙側接觸但物體最高只上升約 `0.06 mm`，由 `empty_grasp_verification_failed` 正確拒絕。

新版 8-env runtime 揭露更早的 blocker：離開 open 後約 `85°` 的末端姿態對正期間，TCP 從約 `0.64 m` 下沉至 `0.012–0.019 m`，造成 inner-finger／knuckle 對 ground 或 target 的 disallowed contact。三輪報告分別為 `scripted_randomized_baseline_20260905_fixed.json`（7 collision、1 timeout）、`scripted_randomized_baseline_20260905_fixed_alignment_target.json`（8 collision）及暫時降低 orientation gain 的 `scripted_randomized_baseline_20260905_gain005.json`（7 collision、1 timeout）；皆為 `0/8`，無 joint-limit 或 non-finite。無效的 gain 調整已撤回。controller 現在凍結 alignment world-position target，open tolerance 以設定檔明列為 `0.10 rad`，本機 randomized dynamics／controller、DirectRL contract、integration 共 63 項測試通過。

下一個 runtime 校正必須先建立 gravity-on 的 reset-pose／orientation-only hold 測試，記錄每步 position/orientation error、raw DLS joint delta、clamped action 與 measured/target joint tracking；在 TCP 高度能維持且零接觸之前，不調整 descend height、TCP offset 或 preload，也不勾選 baseline 完成。

### 2026-09-05 MoveIt 抓取基線（已完成模擬抓取與多位置驗證）

改以 MoveIt 規劃取代 scripted DLS 作為 §8 baseline planner 的路線。scripted DLS 已由 600 步 orientation-hold 診斷確認會卡在局部極小值（action 掉到 0.000–0.029、姿態誤差凍結 39–53°、漂移凍結 65–102 mm），為局部 IK 的本質限制；替其姿態誤差加每步上限的方案已完整實作並掃描後否決（孤立漂移指標 202→88 mm，但 8-env 碰撞率 0.25→0.88），改動全數退回。

已解決：arm drive `stiffness 3000 / damping 300` 改為 `12000 / 600`（端點誤差 20.28→8.55 mm、零震盪、scripted 碰撞率 7/8→2/8；36000 無額外收益故不採用）；`export_moveit_grasp_plan.py` 由 pose goal 改為先解 `/compute_ik` 再以 joint goal 規劃，使 IK 分支可重現（兩次規劃關節值完全相同、端點誤差穩定 5.4 mm）；在此基礎上 TCP 標定兩次迭代收斂，雙指中點偏差 −9.72 → +0.02 mm，左右指接觸相差一步；夾爪閉合改為跟隨 measured，擠壓量由失控的 0.39 rad 收斂至 0.0100 rad。另實測 RG2-FT 碰撞幾何窗口（指墊 TCP−0.8~+59.2 mm、指節 TCP+32.7~+80.8 mm），據以將目標物由 80 mm 立方改為 `50 × 50 × 60 mm`、`object_center_z_m: 0.03`、抓取 TCP 0.040，該組態全程零 disallowed contact。

已成功抓起物件：以 contact sensor 的 `net_forces_w` 取得指墊淨力向量後定案——力方向正確（沿閉合軸水平），但量值僅 0.9 N，而 0.25 kg 物體需約 2.0 N/指。根因為 clean import 的 `gripper_master` 驅動過弱（`stiffness 2500 / effort_limit 50`）。改為 `stiffness 40000 / damping 850 / effort_limit_sim 200` 後，指墊較弱側保持力 6.61 N，完整 reach → descend → close → lift 成功，物體舉起 `89.5 mm`，`passed: true`、`terminated_reason: success`。餘裕檢核：隨機化最壞情況（0.35 kg 配動摩擦 0.35）每指需 4.91 N，本組態經 gripper stiffness `×0.8` 後仍有 5.29 N。實體 RG2-FT 夾持力規格為 3–40 N，故 6.6 N 合理，原本的 0.9 N 才是偏離規格的建模值。

先前否決的三個假設（預壓大小、follower 力矩上限、PhysX mimic 約束剛度）方向皆不正確，但其排除過程確認了 mimic follower 在受載時傳遞力正常，問題不在連桿。

後續已在 Isaac PC 補齊四個校正位置（near `(0.52, 0.00)`、far `(0.67, 0.00)`、left `(0.61, 0.12)`、right `(0.61, -0.12)`）與 seeds `24180`、`24181`、`24182` 的嚴格回放，共 `12/12` 成功；物體抬升約 `87.7–89.9 mm`，最大端點誤差約 `9.5 mm`。校正補償以 `grasp_calibration_offset_m` 與真實 cube pose 分開保存，舊的 `mh5_jg_t40.json` 亦通過相容性重跑。因此「完成第一個 baseline policy／planner」已可勾選；但這是四個已校正位置的多 seed 驗證，不等同未見物體、未校正位置或完整 workspace 的泛化評估，下一項仍保持未完成。原始成功證據見 `docs/validation/mh5-moveit-grasp-success-20260905.json`，後續詳細報告保存在 Isaac PC 的 `/home/vincent/Desktop/mh5_isaaclab_reach_lift_v0/workspace_grid` 與 `demonstration_reports`。

本次全程未接觸實體手臂：MoveIt 以 `mock_components/GenericSystem` 執行，未啟動 FS100 的 ROS 1 bridge、motoman driver 或 `/robot_enable`；規劃產物均帶 `execution_allowed: false` 且回放端強制檢查。

完整過程、已否決假設的證據與復現方式見 `docs/moveit-grasp-baseline-20260905.md`。

### 2026-09-05 MoveIt demonstrations 與 behavior-cloning warm start

12 個成功 MoveIt episode 已轉為 `4,716` 筆對齊 transition 的示範資料集；每筆包含 27 維 observation、7 維 action、next observation、reward、termination、episode、timestep 與 scripted segment。第一筆 transition 因 object relocation 後 observation 尚未刷新而明確丟棄。資料集 SHA-256 為 `7314ee4e4d2793e215c78d6946dfbb5afc0a450e2da49835d6d38346851c3e9f`。

Feed-forward MLP 雖有低離線 validation loss，閉迴路仍在 58 步內漂移，離 cube 約 `0.661 m`。加入 phase memory 的 incremental-action GRU 是目前最佳 BC 候選：在 near 點的 held-out seed `24182` 到達 cube `7.0 mm` 內、取得雙側 contact，訓練 seed `24180` 則到達 `5.1 mm` 內；兩者都因下降末段數毫米橫向偏差造成單側 finger ground contact，尚未完成 80 mm lift。absolute-joint-target GRU 及 action-cap 變體均已實測否決。因此 BC／RL policy 尚未完成，不得送往實體或用來勾選未見場景評估。

上述 dataset、checkpoint、metrics 與逐次 JSON report 目前保存在 Isaac PC 的 `/home/vincent/Desktop/mh5_isaaclab_reach_lift_v0`。本 repo 的 `docs/isaac-training-progress-20260905.md` 是詳細實驗附件；本 checklist 仍是唯一正式整合進度清單。由於 artifacts 尚未全部納入 `mh5_run_artifacts.py` 的 retention manifest，且 BC policy 尚未通過閉迴路抓取，「保存模型、設定、版本、seed、metrics 與訓練 log」暫不勾選。

### 2026-09-05 高離地裕度示範基線（垂直裕度修正）

逐筆比對 GRU 失敗軌跡後修正診斷：主因是**垂直安全裕度不足，而非橫向誤差**。失敗前一刻的橫向誤差在 validation seed 為 `3.586 mm`、training seed 僅 `0.240 mm`；而 MoveIt 示範本身的抓取 TCP z 為 `0.036140 m`，GRU 觸地於 `0.035170 m`——相差不到 1 mm。原示範等於在碰地門檻邊緣採樣，sub-mm 的 BC 誤差即翻轉成敗，不適合直接用於 DAgger。

另更正兩項先前的過度解讀：`253/294` 的 joint-limit projection 中，所有大於 `1e-5` 的投影都來自 `finger_joint`（160 步、最大 `0.0512`），六軸手臂最大僅 `2.38e-6`，屬數值邊界，不能解讀為手臂大幅越界；`5.27 sigma` 發生在 observation 第 3 維（`joint_4_r` normalized position），該維訓練標準差僅 `0.000352`，屬 normalization floor 問題。

四個校正位置的 MoveIt plan 已全部以 `--grasp-offset 0.020` 重新產生（抓取 TCP z 由 `0.040`／`0.044` 提高到 `0.050`／`0.054`），真實物體座標與校正偏移維持不變。完成 4 位置 × 3 seeds（24180／24181／24182）共 12 次回放驗證：**12/12 通過**，全部 `terminated_reason: success`，抬升 `85.95`–`86.93 mm`（門檻 `80 mm`），無終止碰撞記錄、無任一步 `disallowed_contact`、無關節超限、無非有限值。最低 TCP z `0.04650`–`0.04986`，對 GRU 觸地點的最差離地裕度為 `11.33 mm`（far／seed 24182），對照原基線的不到 1 mm。

plan 身分以 SHA-256 驗證：12 份報告記錄的 `plan_sha256` 全部等於對應的 `_hc` plan，且無任何一份指向已被取代的 `_v2`／`_v3` plan；舊 plan 未修改，保留供對照。抬升量略低於原基線的 `87.7`–`89.9 mm`，因抓取位置提高使指墊接觸長度由約 `24 mm` 降為約 `14 mm`，提高後的夾爪驅動仍足以保持。

證據見 `docs/validation/workspace_grid_highclearance/`（4 個 plan 與 12 份報告），細節見 `docs/isaac-training-progress-20260905.md`。

尚未以此高裕度資料重建 dataset 或重訓 GRU，故 §8 的「在未見過的模擬場景評估成功率」仍未勾選。

### 2026-09-05 Dataset v1、normalization floor 與 incremental GRU 重訓

以四個高裕度位置的 12 次成功回放重建示範資料集 `mh5_moveit_demonstrations_v1.npz`（v0 保留未動）：12 episodes、`4,711` 筆對齊 transition，SHA-256 `b7ccf485f2bf6c197d92907336b4c966ded43afb2403f69e7104f64953e10d3d`。manifest 另記錄 grasp offset `0.020`、seeds `[24180, 24181, 24182]`、四個 plan 雜湊、成功率 `12/12`、抬升範圍與最低垂直裕度 `11.33 mm`。回放腳本已改為在報告內記錄 `seed` 與抓取來源，重跑後 dataset 雜湊不變，確認新增欄位未影響任何 transition。

對齊問題複查：12/12 的第一筆 observation 仍為物體重新定位前的舊值（同 seed 下跨四位置完全相同），故每集丟棄第一筆仍屬必要；丟棄後 `next_observation[k]` 與 `observation[k+1]` 的最大絕對差為 `0.000e+00`。

Normalization floor：新增 arm position `dims 0:6 >= 0.002`，保留 previous action `dims 20:27 >= 0.05`。實際受 floor 約束者僅三維（3、20、23）；`dim 3`（`joint_4_r` normalized position）原始 std 僅 `0.000350`，是先前 `5.27 sigma` 的來源，`dims 0:6` 其餘五軸為 `0.041`–`0.194`，floor 精準只作用在該軸。`joint_4_r` 的 fixed-action mask 維持不變。

以 dataset v1 重訓 incremental-action GRU（`mh5_bc_recurrent_hc_inc_v1.pt`，validation MSE `2.16e-4`）。閉迴路評估：**訓練 seed `24180` 四個位置 4/4 通過**（抬升 `84.5`–`87.4 mm`、xy 誤差 `0.01`–`1.65 mm`）；**held-out seed `24182` 僅 1/4 通過**（僅 far 成功，抬升 `91.12 mm`）。訓練 seed 全數成功代表訓練配方無誤，失敗集中於未見 seed，屬泛化落差，因此 DAgger corrective 資料確有必要。

held-out 失敗並未用到新增的裕度：near／right 下沉至 `0.0379`／`0.0390`、left 至 `0.0349`，較示範的 `0.0476` 低 `6`–`13 mm`；left 的失效為 `inner_knuckle` 撞擊方塊（抓取過深），與另兩者的觸地不同。橫向誤差亦較 v0 策略退步（near `12.36 mm`、right `9.34 mm`，v0 為 `0.24`–`3.59 mm`）。據此，後續擾動範圍垂直須涵蓋「過深」側，橫向則以 `±10 mm` 而非 `±5 mm` 較符合實測。

過程中記錄一項作業疏失：首次重訓誤用磁碟上已被改為**絕對關節目標**（先前已否決的變體）的訓練腳本，導致動作自第 0 步即飽和於 `1.000`、normalized input 達 `31.8 sigma` 且手臂不下降；該 checkpoint `mh5_bc_recurrent_hc_v1.pt` 無效，僅留作負面證據。訓練腳本現已加入 `--target-mode {incremental,absolute}` 明確開關並寫入對應 `output_encoding`。

BC 相關腳本（`build_mh5_moveit_demonstrations.py`、`train_mh5_behavior_cloning.py`、`train_mh5_recurrent_behavior_cloning.py`、`run_mh5_bc_policy_eval.py`）此次一併納入 repo。dataset 與 checkpoint 依既有慣例仍保存於 Isaac PC。證據見 `docs/validation/mh5-moveit-demonstrations-v1.manifest.json` 與 `docs/validation/bc_eval_highclearance/`。

§8「在未見過的模擬場景評估成功率」維持未勾選：held-out 成功率為 `1/4`。

## 10. 實機安全命令閘門（完成前禁止 Isaac 控制實機）

- [ ] 選定唯一實機介面，例如受控的 `FollowJointTrajectory` adapter；禁止任意 topic 直通 FS100。
- [x] command gateway 只接受六個白名單 joint names，並再次檢查順序與單位。
- [ ] 加入 position、velocity、acceleration、workspace 與每步最大變化限制。
- [ ] 加入碰撞檢查與軌跡有效性檢查。
- [x] 加入 freshness timestamp、固定 rate limit、watchdog 與斷線停止策略。
- [x] 加入人工 `armed`／dead-man gate，預設永遠為 disarmed。
- [x] `armed` 狀態不得跨重啟保存，且必須能由急停／robot disable 立即解除。
- [x] 先用 fake controller／simulated FS100 完成 gateway 測試。
- [ ] 再做不送出命令的 dry-run，人工逐筆審核產生的 trajectory。
- [ ] 現場確認急停、低速模式、工作區淨空與操作人員分工。
- [ ] 取得人工明確批准後，才進行第一個實機低速、單關節、小角度測試。
- [ ] 完成安全停機、故障復原與 rollback 演練。

`scripts/offline_safety_gateway.py` 只是一個沒有 ROS、DDS、socket、subprocess 或 FS100 transport 的 fail-closed core；唯一 sink 是記憶體中的 fake controller。它已驗證 canonical 六軸順序、`rad`、URDF position limits、`0.01 rad` step、保守 velocity／acceleration、freshness、rate、dead-man、watchdog、disconnect、E-stop 與 restart-disarmed。workspace 與 collision 目前只是必須存在且 fail-closed 的 callback contract，尚未接上 production model；因此包含 workspace 的綜合限制項與 collision／trajectory 項仍不打勾。也刻意不選定或建立 real adapter，避免在沒有人工審查時產生 Isaac→FS100 路徑。設定見 `config/safety/mh5_gateway_core_v0.yaml`。

## 11. 最終驗收與維護

- [ ] 建立 `sim_train`、`mirror`、`shadow`、`real` 的完整操作手冊。
- [ ] 每個 launcher 都有 preflight、清楚的模式名稱及目前 command authority 顯示。
- [x] 建立自動測試：joint mapping、limits、topic isolation、USD runtime、camera transform、mimic、reset 與 perception smoke evidence。
- [x] 建立 log 與 dataset 的儲存、版本及清理規則。
- [ ] 驗證重開機、網路中斷、Isaac crash、bridge crash 與 FS100 重啟後的行為。
- [ ] 完成最終備份與 release tag。

log／dataset／checkpoint 的 storage roots、run manifest 必填欄位、hash、保留期與人工清理規則已固定於 `config/artifacts/retention_v1.yaml` 與 `docs/artifact-retention-policy.md`。清理預設只允許 dry-run，Git validation evidence 與已接受 calibration raw capture 禁止自動刪除。

## 下一個建議工作

1. 若要驗收 eye-on-base 絕對精度，精量桌面到 `base_link` 的偏移、Marker 高度及板面 X／Y／yaw，再重跑獨立投影測試。
2. 取得 MH5／附件的可信 mass、center of mass 與 inertia，取代幾何推算值。
3. 針對 sweep 找到的 6 組 base-vs-wrist/tool 接觸，將已有的 `mh5_disallowed_contact_regression_v0.json` 接入 MoveIt planner／production gateway 的連續軌跡拒絕，並以更密集的 workspace sampling 驗證；不得把 DirectRLEnv 的固定姿態回歸當作完整 planner 驗收。
4. 以目前通過 12/12 的 MoveIt planner 作 teacher，在 pre-grasp、descent 與 close 附近加入受控狀態擾動與 corrective action，建立 DAgger-style aggregation dataset；先讓 incremental-action GRU 在四個位置與 held-out seeds 閉迴路成功，再考慮 PPO warm start。用 `mh5_run_artifacts.py` 納管接受的 dataset、checkpoint、seed、metrics 與 log；strict inertial audit 的 `physics_parameterization_ready=false` 仍不得被 domain randomization 繞過。
5. 精量或多視角擬合故障 MH5 的固定關節姿態，並補量桌架與 ZED 安裝支架接觸尺寸，再取代 provisional collision geometry；ZED 外觀尺寸本身已由官方 mesh bounds 固定。
6. 建立 ZED object-pose observation adapter，先跑 read-only shadow mode，記錄視覺 pose、policy action、拒絕原因與 latency；`shadow` live state、real adapter 選型、人工 dry-run、急停與第一個實機動作都必須等操作員清醒在場後逐項完成，不能由 unattended agent 代簽或代測。

### 2026-09-06/07 Dataset v3、GRU 泛化與 synthetic ZED position baseline

Dataset v3 保留 nominal 12、±0.010 corrective 144 與 ±0.020 corrective 144，共 300 episodes／87,767 transitions，SHA-256 `c00c527afacb34ed0a5cb04b83f41ec6b45492e6319e57d48274e0d86cc374f3`；v0/v1/v2 均未覆寫。incremental-action GRU 使用完整 episode、nominal 0.50／corrective 0.50（return 0.30、四個 downstream 各 0.05）的分層 loss 與既定 normalization floors。正式 checkpoint `mh5_bc_recurrent_v3_inc_v1.pt` SHA-256 `6f83ca3cbb57737c858f0f2e2d22f6d29b7ec1dd251707007bd24fe988bf8da0`。

嚴格 simulation-only closed-loop：nominal held-out 3/4、既有 ±0.010 recovery 144/144、既有 ±0.020 recovery 143/144；完全未入資料集的 dynamics seeds 與新擾動為 nominal 8/12、±0.010 26/36、±0.020 22/36，合計 56/84。0/376 以 joint-limit violation 終止；六軸 projection 最大 `2.38e-6 rad`，有量級的 projection 仍是 finger_joint。失敗包含禁止碰撞與 600-step truncated。由於 fully unseen generalization 未通過，本項**不勾選**，不做 PPO、不進實機。另確認 v3 的 288 筆 corrective replay 全在 dynamics seed 24180，只有 source trajectory 分布於三個 seeds，這是目前最重要的 dynamics diversity 限制。

2026-09-07 新增 synthetic ZED RGB-D position baseline：estimator 只取 RGB、depth、內參與既有 eye-on-base transform；TargetCube ground-truth transform 在估測完成後才讀取且只用於評分。near/far/left/right × 兩個未見 seed labels 共 8/8 valid，base_link 3-D position error mean `1.324 mm`、median `1.320 mm`、max `1.698 mm`（門檻 20 mm），ROS 2 Bridge false。總表 SHA-256 `85ac263bd4af2550f454ae7132c2c3db8aeacff127fb58e7aa4c83d74e598181`，保存在 Isaac PC `mh5_isaaclab_reach_lift_v0/zed_pose_v1/summary.json`。

27-D adapter 已離線驗證 dims 14:17 `gripper_to_object`、17:20 `object_local`，8-case 與 ground-truth fields 最大差 `1.320 mm`；invalid depth、>250 ms stale、低信心、non-finite、workspace 外五類均 fail-closed，且 `actions_emitted: false`。adapter 報告 SHA-256 `88a9be657c82b4b7845dde33a887adade9d3a6dae12558992d612ea8e9aa4084`。這只是 observation-contract 離線 A/B，尚未完成每 control tick 的 camera-driven closed-loop grasp；orientation 亦仍具方柱對稱歧義。因此 ZED observation adapter／camera-driven grasp **不得勾選為完成**，也沒有啟用實體 ZED 或實機 control。詳細附件為 Isaac PC `MH5_PROGRESS_2026-09-06.md` 與 `MH5_PROGRESS_2026-09-07.md`。

### 2026-09-07 中午 camera-driven closed-loop 抓取評估（未通過，找到結構性 blocker）

在同一顆已驗證 checkpoint（`mh5_bc_recurrent_v3_inc_v1.pt`，hash 與 bc_eval_v3 相同、未重訓）與同一 `combined.usd` 上，建立每個 control tick 都用即時相機估測更新 27-D observation 的 simulation-only 閉迴路（`run_mh5_bc_policy_eval_camera_closed_loop.py`）。建置過程中修正三個真實 bug：相機朝向少了 180°-X 校正、`gripper_to_object` 誤用含 TCP offset 的參考點（憑空塞入約 168 mm 常數偏移）、以及 RTX 相機 render product 會凍結但 age 偵測不到的隱藏 stale 漏洞（新增逐位元組比對偵測 `frozen_render_frame`）。

near/far/left/right × seed 24182（held-out）／seed 24190（全新未見）共 8 case 全數以 `camera_fail_closed_abort` 終止（8/8 皆非成功，也**沒有一次**以碰撞或 joint-limit 終止）：`right` 位置從 home pose 起就被手臂本體擋住相機視線（0/4 tick 有效）；`near/far/left` 於 reach 階段追蹤準確（mean 2.7–10.4 mm）但夾爪靠近抓取時遮住目標，confidence 跌破門檻。延長連續-rejection 容忍度到 60（診斷用，非驗收放寬）證實這是持久遮蔽，不會自行恢復。所有 rejected tick 的動作均為 hold（zero action），沒有把未驗證資料送進 policy。

結論：這是目前這顆**固定外部 ZED 相機安裝視角**在最終抓取階段的結構性遮蔽，不是 policy 或碰撞/關節限位問題；因此本項**不勾選**、不放寬任何門檻、不做 PPO、不進實機。詳細逐 case 結果、command、hash 見 Isaac PC `mh5_isaaclab_reach_lift_v0/camera_closed_loop_v1/summary.json` 與 `MH5_PROGRESS_2026-09-07.md`。下一步建議：改用 eye-in-hand 相機或加第二視角覆蓋 `right` 側與最終抓取角度。


### 2026-09-07 下午 實體 ZED read-only 檢查（未偵測到裝置，於診斷步驟 fail closed）

依「只讀感測、不得控制實體 MH5」任務啟動，全程 `ros2_bridge_enabled: false`、`real_interfaces_enabled: false`、`command_authority: none`，未搜尋或啟動任何手臂 driver/controller/MoveIt 啟動檔，未向實體 MH5 送出任何 joint/action/trajectory command。

`lsusb` 全列表與 `lsusb -d 2b03:`（Stereolabs vendor ID）均確認**沒有實體 ZED 裝置連接**（`/dev/video*` 亦不存在，非權限問題，帳號已在 `video`/`zed` 群組）。官方 ZED SDK 3.8.2 唯讀診斷工具 `ZED_Diagnostic -c` 佐證：`USB Camera Diagnostic: Failed — No Camera detected`（`ZEDs`/`ZEDDetected` 陣列皆空）。ZED SDK 另回報 `CUDA 13 not detected`／`Graphics Card` 偵測失敗，經 `nvidia-smi` 核實為 SDK 診斷工具與目前環境（RTX 5080、driver 580.126.20 實際正常運作，只是未裝 CUDA toolkit/`nvcc`）的版本落差，屬次要且獨立的環境缺口，不影響「相機未接上」這個主要且已足夠 fail-closed 的結論。`pyzed` 在本機 Python 環境亦尚未安裝。

依規則裝置不在時必須停在診斷、不得以模擬資料替代：本節**沒有**實際 RGB/depth/point cloud 擷取、**沒有**GUI 顯示、**沒有**物體位置估測，因此本項**不勾選**。證據與 hash 保存在 Isaac PC `mh5_isaaclab_reach_lift_v0/physical_zed_readonly_v1/`（`ZED_Diagnostic_Results_20260907_121459.json` SHA-256 `e3749cc460d62fc6bb17a33a1448e7ac31083795b0be5321294eef4579d6ffbd`；`zed_diagnostic_stdout_20260907_121459.log` SHA-256 `28217e10953a0ca4f1b936320e54f0a7368c00dfcca002944b08e952ee9182fe`；`environment_evidence_20260907_121459.log` SHA-256 `9fb25c2636088a3a2ea6dada0db35c67b8aa93fad6e885292d3ed1fc720842f0`）。此節與同日稍早的 synthetic ZED position baseline（已通過）完全分開記錄，不得混淆為同一結果。

下一步建議（尚未執行）：實體檢查 ZED 連接線與供電確認插入本機後，重跑同一組唯讀指令複驗；再視情況安裝 `pyzed`（`get_python_api.py`，唯讀安裝）與確認是否需要 CUDA toolkit，之後才依序做 read-only 擷取、GUI 顯示、camera-frame 物體位置估測（無可信外參前不得宣稱 robot-base frame pose）。


---

## 更正：實體 ZED 位於 cyc，v2 唯讀實證（2026-09-07 13:31 +08:00）

上一輪在 Isaac 電腦做的「未偵測到 ZED」是錯誤主機診斷；原節與證據保留。本次 SSH `cyc-server` 實測 `lsusb` 的 `2b03:f582 STEREOLABS ZED camera`，serial `24180`、型號 `ZED`、SDK `3.8.2`，`/dev/video0`/`video1` 與 pyzed 權限正常。cyc ROS graph 雖有 fake/demo MoveIt、ros2_control 與控制相關 topics，本次未接觸或呼叫任何 ROS/DDS、driver、MoveIt execution、controller、joint/action/trajectory topic/service/action。

純 `pyzed.sl` SDK read-only capture：HD1080 `1920x1080`、30 FPS、ULTRA、IMAGE/meter，30 warmup + 1 frame；實測有效 depth/XYZ 點 `1,470,452/2,073,600`，PLY 為 binary XYZRGBA。cyc 原件：`data/zed_rgbd_pointcloud_readonly_v2/20260907T133138+0800/`；Isaac 副本：`physical_zed_readonly_v2_from_cyc/20260907T133138+0800/`；兩端逐檔 SHA-256 一致，完整 hash 見 `SHA256SUMS.txt`。

XYZ 僅為 ZED left optical/IMAGE camera-frame；ZED-to-MH5-base 外參未驗證，沒有宣稱 robot-base pose，未做 segmentation/SVO/PPO。Isaac 已用只讀 VTK viewer 顯示實際 PLY。安全開關維持 `command_authority=none`、`ros2_bridge_enabled=false`、`real_interfaces_enabled=false`；舊 dataset/checkpoint/證據未覆寫或刪除。

### 2026-09-07 Codex physical ZED PLY Isaac Sim view-only import

- Isaac Sim 5.1 GUI viewport was actually opened with the MH5/workcell USD and the physical ZED PLY from data/zed_rgbd_pointcloud_readonly_v2/20260907T133138+0800/zed_pointcloud_xyzrgba.ply.
- Result is UNREGISTERED camera-frame display only. No ZED-to-MH5-base/world transform was applied; no registered/fused claim is allowed.
- Source PLY SHA-256: c1c434d17ebeb89fdcfa3b23ffbd8185290bd2eb6e23b8c8a9824579ca9989ea; finite source points: 1470452; Isaac display sample: 98031; RGB retained.
- Isaac evidence is on the Isaac PC under /home/vincent/Desktop/mh5_isaaclab_reach_lift_v0/physical_zed_isaac_view_v1/: isaac_gui_view.png SHA-256 4a42b7a4119b29e8ee0892c0c911d30668b669723259a9baee6eab55b0363815, import_report.json SHA-256 7f11634c5b670ff48ee4a3941c85ef280b97eab5555980c085b4ecd8a190eeed.
- Available ArUco projection evidence (2026-08-20) reports RMSE 16.101 px, max 20.216 px, but board X/Y/yaw were fitted from the same image and board height/table-to-base assumptions were operator-provided; it is not independent absolute eye-on-base validation. Original easy_handeye2 artifact was not retrieved. Keep registration gate closed.
- Safety: simulation/view-only; ROS 2 Bridge=false; real interfaces=false; no driver/MoveIt/controller/topic/service/action/policy/PPO; no physical MH5 movement.

### 2026-09-07 傍晚 ZED live 點雲串流、housing visual 持久化、外參 sensitivity（Isaac PC 主導，本節唯讀 append）

Isaac PC 側完成三項修正（詳見 Isaac PC `MH5_PROGRESS_2026-09-07.md` 傍晚班次章節）：
1. 已驗證的第一代 ZED housing/lens 外觀（`BodyBoundsVisual`/`LeftLensRing`/`LeftLensGlass`/`RightLensRing`/`RightLensGlass`）拆成獨立 USD overlay layer，組合進新場景 `mh5_physical_workcell_persistent_zed_visual/mh5_physical_workcell.usda`，不修改 clean robot USD、不修改既有 `mh5_physical_workcell/mh5_physical_workcell.usda`（hash 確認未變）；`render_zed_physical_model.py` 驗證 passed=true。
2. 新增 read-only 即時點雲串流：cyc 端 `zed_live_stream_cyc_sender.py`（純 `pyzed.sl`，只 bind `127.0.0.1`，未占用/未新增對外 port，未使用 ROS）；Isaac 端經獨立 SSH tunnel 接收，於 USD session layer 就地更新同一個 `UsdGeom.Points` prim。已用真實 cyc ZED（serial 24180）端到端驗證，GUI 截圖可見 MH5＋workcell＋live 點雲同時顯示。因 cyc 對外為 WiFi 上行，實測穩定頻寬約 0.9–1.3 MB/s，量到平均 FPS 約 1.5–2.6 Hz（低於 5–10 Hz 目標下限，誠實記錄非隱藏）。點雲 transform 仍用既有 candidate `eye_on_base`，狀態維持 `CANDIDATE_NOT_VALIDATED`。串流已於驗證後停止，未留長駐程序占用相機。
3. 對既有 4-marker ArUco 證據（2026-08-20，未重新從新影像擬合——新影像只偵測到 1/4 marker，證據不足）做 candidate eye_on_base 聯合精修＋敏感度分析：數學證實單一影像／單一 board 無法分離相機外參與 board 姿態（condition number ~1e16），6 個外參自由度全數標記需要獨立人工量測；**未**更新 `zed_sn24180_hd1080.yaml` 的 candidate 值，維持 `CANDIDATE_NOT_VALIDATED`。

另診斷 cyc 現有 ros2_control 硬體介面：`src/mh5_moveit_config/config/motoman_mh5.ros2_control.xacro` 的 `<plugin>` 是 `mock_components/GenericSystem`，目前並存的 4 組 `ros2_control_node`／`move_group`（2026-09-05 啟動）皆用同一份 mock 設定，且無 FS100/MotoROS driver 程序在跑；因此 `/joint_states` 不能視為實體 MH5 讀值，Isaac 端未做任何視覺姿態同步、保留 authored pose。

本節全程 `ros2_bridge_enabled=false`、`real_interfaces_enabled=false`、`command_authority=none`；未觸碰、啟動或停止任何 cyc 既有 fake/demo ROS 程序；唯一被啟停的是本節自建的 `zed_live_stream_cyc_sender.py`（已停止）。未勾選任何既有清單項目（本節屬於獨立於既有清單結構之外的 Isaac PC 進度 append，未驗證項目維持未勾選）。完整 hash 見 Isaac PC `isaac_pc_setup_zed_model_20260821/SHA256SUMS_20260907_zed_live_registration.txt`。

---

### 2026-09-10 夜間 視覺定位 → MoveIt → 實體 MH5 移動到物件上方，並可回原位

本節記錄第一次由 ZED 視覺決定目標、經 MoveIt 規劃、實際驅動實體 MH5 的運動。機器可讀證據：`docs/validation/mh5-staged-vision-pregrasp-physical-20260910.json`。

**已驗證可做到**：偵測桌上的藍色捲尺 → 產生 top-down 抓取分段 → 規劃 → 實體手臂移動到物件正上方 100 mm → 回到原本的休息姿態。實測 4 趟（pre_grasp ×2、home ×2，最後一趟為 2026-09-11 10:45），全部完成，`max_error_rad` 介於 2.34e-04 到 7.17e-04（約 0.013°–0.041°），每趟 26 點、約 7.77 秒。

**抓取幾何由 URDF 推導並用測試釘死**，非臆測：`grasp_joint` 套 Ry(-90°)，故 `grasp_link +X` 為接近方向；RG2-FT 手指沿 `grasp_link +Y` 分開（與 URDF 鏈實測點積 1.000000）。由此推得 yaw 等於物件長軸在 XY 平面的角度。另發現 `/compute_ik` 回的是「某一個」解而非最近解，原本 pre-grasp 需把手腕 T 軸轉 129.43°；因 top-down 抓取在半圈旋轉下等價，改為同時解 yaw 與 yaw+180° 並取手腕擺動最小者，降為 50.57°。

**新增獨立的分階段執行通道，未放寬原有閘限。** 原 one-shot 通道的 `MAX_SEGMENT_RAD`(0.035 rad) 同時限制「整條軌跡的總位移」，因此切密軌跡無效，抓取（pre-grasp 需 50.57°）在結構上無法通過。處理方式不是調大該值——那會默默削弱所有既有路徑——而是新增 `robot_integration/staged_envelope.py` 這條獨立通道，原 `COMMAND` 路徑逐行確認未更動。上限：速度 0.10 rad/s（沿用 `config/safety/mh5_gateway_core_v0.yaml` 已審查值）、單階段總位移 1.20 rad、單點步長 0.15 rad、起點誤差 0.02 rad。時間戳由伺服端依速度上限重算，客戶端給的時間完全丟棄（非驗證而是丟棄），故客戶端無法要求更快。arm 時發一次性 nonce，publish 後立即 disarm 回 MIRROR。該模組不含 ROS，可離機單元測試（34 項）。

**尺規驗證的意義界定**：量測 `[0.38, 0.28, 0.04]` 對偵測 `[0.3777, 0.2776, 0.0422]`，差 2.3/2.4/-2.2 mm。但量測值僅到 1 cm 解析度，**故此檢查證實的是「在量測解析度內吻合」，不是毫米級準確**；它驗證的是 optical-to-base 轉換（以物件為探針），不是某個特定擺放位置。捲尺其後被不慎碰移，下一次偵測（seq 72008）追到 28 mm 外的新位置而 z 與叢集品質不變——同一擺放下三幀重現性在 1 mm 內，故「橫向跳動但 z 不變」應判讀為物件被移動而非偵測漂移。

**測試過程抓到一個真 bug**：`arm` 與 `staged_trajectory` 原本走兩條 TCP 連線，而 gateway 在客戶端斷線時會 disarm，故 `run_staged_grasp.py` 原本永遠不可能執行成功（只會回 `not armed`）。已改為三個指令共用同一條連線。此問題僅在實打線上 gateway 時浮現，單元測試看不到。

**本節未做、不得宣稱**：`grasp` 與 `lift` 階段從未執行；**夾爪從未被下過任何命令**（RG2-FT gateway 在跑、Modbus 可達，但未曾 open/close）；Isaac 端為 gravity=0 的運動學預覽，只播放規劃路徑，**未模擬接觸**，不構成物理夾取驗證。偵測到的 extent 是降採樣俯視「上表面 footprint」而非真實外框，且 z 裁切下界會截斷物件下緣。規劃的抓取單邊僅約 6 mm 餘裕（物件窄邊約 30 mm，夾爪開約 42 mm）。`config/safety/mh5_gateway_core_v0.yaml` 四個 precondition 仍全為 false，本路徑未消費它們。

**安全**：預設維持 MIRROR / DISARMED；每一次實體階段皆經操作者明確確認且人在 E-stop 旁；一次 arm 僅一階段；偵測報告超過 300 s 即拒絕使用；物件相關階段強制檢查 profile 的 `robot_motion_allowed`。`home` 階段刻意跳過偵測報告與 profile 旗標檢查——回原位是與物件無關的關節目標，若套用該兩道關卡，只會在報告過期時把手臂困在桌面上方。

**ZED 串流限制**：sender 一次只服務一個 client，panel 佔住 socket 時獨立偵測程式會靜默逾時（sender 不回錯誤）。故偵測必須跑在 Isaac panel 內、重用 `LiveZedOverlay` 已轉為 base_link 的點雲。

---

### 2026-09-11 MH5_VLA_PLAN Phase 0：MH5_CFG spawn 驗證（純模擬，FAIL，含兩個真發現）

機器可讀原始輸出：`docs/validation/mh5-isaaclab-spawn-phase0.json`。腳本
`integration_workspace/robot_integration/verify_mh5_isaaclab_spawn.py`，headless 執行，
未開啟任何 socket、未接觸 gateway、實體手臂全程未動。**整體結果 FAIL。**

注意：該次執行的行程最終以 exit code 143（SIGTERM）結束，是外層 `timeout 600` 在
Isaac Sim 關閉流程卡住時將其終止。報告已於此之前完整寫出（`result` 欄位是 `main()`
最後一個寫入的欄位，JSON 可完整解析、5 項檢查齊全），故結果有效；exit 143 不代表本次量測失敗。

**執行環境（本機 `isaaclab.sh` 是壞的，記下來免得重試）**：`/home/vincent/IsaacLab/isaaclab.sh`
報 `python: command not found`（找不到 `_isaac_sim/python.sh`）。可用組合是
**`/home/vincent/env_isaacsim/bin/python` + `PYTHONPATH=/home/vincent/IsaacLab/source/{isaaclab,isaaclab_assets,isaaclab_tasks,isaaclab_rl,isaaclab_mimic}`**。
原因：`Downloads/python.sh` 有 isaacsim 無 isaaclab；venv 有 isaacsim 且其 site-packages 的
`isaaclab` 是整包 repo 副本（真正 package 在 `.../isaaclab/source/isaaclab/isaaclab/`）但**不含 mh5.py**；
`mh5.py` 只存在於 `/home/vincent/IsaacLab/source/isaaclab_assets/`。另注意 IsaacLab 腳本必須先
`AppLauncher` 再 import `isaaclab.*`，否則 `pxr` 尚未就緒。

**PASS 項目**

- USD 關節名稱與 URDF 完全一致：12 個（6 手臂 + 6 夾爪），無缺漏、無多餘。
- `MH5_CFG` 設定引用的手臂與夾爪關節在 USD 中全部存在。
- 靜態比對：URDF `<mimic>` multiplier 與 `mh5.py` 的 `MH5_GRIPPER_CLOSE` 符號完全吻合
  （`right_inner_knuckle_joint` 為 +1，其餘四個從動為 -1）。
- 釐清一個易誤判處：URDF 裡另有 `right_inner_knuckle_to_finger_joint` 與
  `left_inner_knuckle_to_finger_joint`，但它們在 `<gazebo>` 區塊內、用 SDF 語法
  （`<pose>`、`<use_parent_model_frame>`），標準 URDF parser 不會當成關節，是 Gazebo 閉環約束。
  故真實可動關節數為 12，與 `MH5_GRIPPER_JOINT_NAMES` 一致。

**發現 A（確認為真，且推翻計畫的既定假設）**

USD 匯入**保留了 mimic**：`/World/mh5/joints/` 下有 5 個 `PhysxMimicJointAPI:rotX`，正好對應
5 個從動夾爪關節。但 `mh5.py` 的 docstring 寫「they are driven explicitly by
BinaryJointPositionActionCfg **instead of relying on mimic**」，此前提不成立。
明確驅動 6 個關節等於在 mimic 約束之上再加一組約束，兩者互相對抗：

| 關節 | 目標 | 到達 |
|---|---|---|
| `finger_joint` | +0.700 | +0.7008 |
| `right_inner_knuckle_joint` | +0.700 | **+2.8715** |
| `left_outer_knuckle_joint` | -0.700 | **+0.2305** |
| `left_inner_finger_joint` | -0.700 | **+0.1471** |

隨後下達開啟指令時數值發散，`finger_joint` 到達 **1628 rad**。
`MH5_VLA_PLAN.md` 風險表中的「RG2-FT 閉環連桿在 PhysX 不穩」成立，
**而所選緩解措施正是肇因**。修正方向：只驅動 `finger_joint`，讓 5 個 PhysX mimic 從動關節跟隨；
並據此修改 `mh5.py` 的 docstring 與 `BinaryJointPositionActionCfg` 的 `joint_names`。此修正尚未實施、尚未驗證。

**發現 B（判定為本次測試腳本的瑕疵，不得記為設定缺陷）**

init pose 未守住：指令 `[0, 0.5, 0, 0, -1.1, 0]`，安定後
`[0.000, 0.224, -0.130, 0.000, -0.018, 0.000]`，最大誤差 1.082 rad。
但各軸皆朝 0 收斂，研判是腳本在 settle 階段**未寫入關節位置目標**，致目標預設為 0，
而非 `MH5_CFG` 的初始姿態設定有誤。**需修腳本重跑才能定論**，本次不對此項下結論。

**發現 C（腳本缺漏）**

`tool0` 與 `grasp_link` 皆不在 articulation 的 16 個 body 之中，但腳本只記錄了 `body_count`、
**未保存 `body_names`**，故無法得知 USD 中末端 link 的實際命名。下次執行須一併輸出 `body_names`。
因此「init pose 是否讓末端位於桌面可作業範圍」本次**未能驗證**。

**結論**：Phase 0 未通過，不得進入 Phase 1。待辦為：修正腳本（寫入位置目標、輸出 body_names）、
改為只驅動 `finger_joint` 後重跑，確認夾爪開合穩定且末端位姿合理。

**安全**：純模擬；未載入任何 ROS/DDS、未連線 gateway、`command_authority=none`、實體 MH5 未移動。
