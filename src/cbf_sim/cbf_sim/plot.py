import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.path import Path
from matplotlib.patches import Polygon

# 1. CSV 파일 불러오기
filename = 'm6_1.csv'  # 실제 저장된 파일명으로 변경하세요
try:
    df = pd.read_csv(filename)
except FileNotFoundError:
    print(f"Error: '{filename}' 파일이 없습니다. 경로와 파일명을 확인해주세요.")
    exit()

if len(df) < 2:
    print("데이터가 너무 적습니다. 로봇이 충분히 주행한 후 다시 로깅해주세요.")
    exit()

# 2. 각도 정규화 함수
def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

# 3. 정량 지표 계산
df['yaw_diff'] = df['yaw'].diff().fillna(0)
df['yaw_diff_norm'] = df['yaw_diff'].apply(normalize_angle)
df['yaw_diff_abs'] = np.abs(df['yaw_diff_norm'])
total_heading_variation = df['yaw_diff_abs'].sum()

# ---------------------------------------------------------
# 코너 구간 설정 (다각형)
# ---------------------------------------------------------
corner_polygons = [
    np.array([
        [1.0, 1.25],
        [2.0, 1.25],
        [2.0, 4.05],
        [1.0, 4.05],
        [1.0, 3.3],
        [0.35, 3.3],
        [0.35, 2.0],
        [1.0, 2.0]
    ]),
    np.array([
        [-1.0, 4.15],
        [0.3, 4.15],
        [0.3, 4.8],
        [0.65, 4.8],
        [0.65, 5.8],
        [-1.0, 5.8]
    ])
]

# 각 코너별 색상/라벨 따로 지정
corner_colors = ['deepskyblue', 'purple']
corner_labels = ['Region 1', 'Region 2']

points = df[['x', 'y']].to_numpy()
corner_mask = pd.Series(False, index=df.index)

for poly in corner_polygons:
    path = Path(poly)
    mask = path.contains_points(points)
    corner_mask = corner_mask | pd.Series(mask, index=df.index)

corner_heading_variation = df.loc[corner_mask, 'yaw_diff_abs'].sum()

# ---------------------------------------------------------
# 미분을 통한 각/선형 가속도 및 저크 계산
# ---------------------------------------------------------
# 시간 변화량(dt) 계산 (0으로 나누기 방지)
df['dt'] = df['timestamp'].diff().bfill()
df['dt'] = df['dt'].replace(0, 0.001)

# [각방향 (Angular)] 미분
df['angular_accel'] = df['angular_w'].diff() / df['dt']
df['angular_accel'] = df['angular_accel'].fillna(0)

df['angular_jerk'] = df['angular_accel'].diff() / df['dt']
df['angular_jerk'] = df['angular_jerk'].fillna(0)

# [선방향 (Linear)] 미분
# CSV에 x, y 분해 속도가 있는지 확인 (Swerve Crab Walk 고려)
if 'linear_v_x' in df.columns and 'linear_v_y' in df.columns:
    df['linear_v_mag'] = np.sqrt(df['linear_v_x']**2 + df['linear_v_y']**2)
elif 'linear_v' in df.columns:  # 구버전 로그 파일 호환용
    df['linear_v_mag'] = df['linear_v'].abs()
else:
    df['linear_v_mag'] = 0.0

df['linear_accel'] = df['linear_v_mag'].diff() / df['dt']
df['linear_accel'] = df['linear_accel'].fillna(0)

df['linear_jerk'] = df['linear_accel'].diff() / df['dt']
df['linear_jerk'] = df['linear_jerk'].fillna(0)

# 수치 미분 노이즈 제거를 위한 스무딩(Moving Average) 적용
window_size = 10
df['accel_smooth'] = df['angular_accel'].rolling(
    window=window_size, min_periods=1, center=True
).mean()
df['jerk_smooth'] = df['angular_jerk'].rolling(
    window=window_size, min_periods=1, center=True
).mean()

# 선형 저크도 스무딩 처리
df['linear_jerk_smooth'] = df['linear_jerk'].rolling(
    window=window_size, min_periods=1, center=True
).mean()

# ---------------------------------------------------------
# 콘솔 출력
# ---------------------------------------------------------
print("=" * 40)
print(f"[{filename}] 정량 평가 결과")
print(f"- Total Heading Variation : {total_heading_variation:.4f} rad")
print(f"- Corner Heading Variation: {corner_heading_variation:.4f} rad")
print("-" * 40)
print("[Angular (회전 안정성)]")
print(f"- Max Angular Accel (Abs) : {df['accel_smooth'].abs().max():.4f} rad/s^2")
print(f"- Avg Angular Accel (Abs) : {df['accel_smooth'].abs().mean():.4f} rad/s^2")
print(f"- Max Angular Jerk (Abs)  : {df['jerk_smooth'].abs().max():.4f} rad/s^3")
print(f"- Avg Angular Jerk (Abs)  : {df['jerk_smooth'].abs().mean():.4f} rad/s^3")
print("-" * 40)
print("[Linear (병진 안정성 - 급가감속/충격)]")
print(f"- Max Linear Jerk (Abs)   : {df['linear_jerk_smooth'].abs().max():.4f} m/s^3")
print(f"- Avg Linear Jerk (Abs)   : {df['linear_jerk_smooth'].abs().mean():.4f} m/s^3")
print("=" * 40)

# 4. 논문용 그래프(Figure) 그리기
fig = plt.figure(figsize=(16, 12))
gs = gridspec.GridSpec(2, 2, width_ratios=[1, 2.5], height_ratios=[1, 1])

ax0 = fig.add_subplot(gs[:, 0])  # 좌측: X-Y Trajectory
ax1 = fig.add_subplot(gs[0, 1])  # 우측 1: Heading
ax2 = fig.add_subplot(gs[1, 1])  # 우측 2: Angular Velocity

# (1) 좌측: X-Y Trajectory
ax0.plot(
    df['x'].to_numpy(),
    df['y'].to_numpy(),
    label='Robot Path',
    color='blue',
    linewidth=2.5
)
ax0.scatter(df['x'].iloc[0], df['y'].iloc[0], color='green', label='Start', s=80, zorder=5)
ax0.scatter(df['x'].iloc[-1], df['y'].iloc[-1], color='red', label='End', s=80, zorder=5)

# 코너 2개 각각 다른 색 적용
for poly, color, label in zip(corner_polygons, corner_colors, corner_labels):
    polygon_patch = Polygon(
        poly,
        closed=True,
        facecolor=color,
        edgecolor=color,
        alpha=0.18,
        linewidth=1.5,
        label=label
    )
    ax0.add_patch(polygon_patch)

ax0.set_title('Robot Trajectory (Spatial)', fontsize=15, fontweight='bold')
ax0.set_xlabel('X Position (m)', fontsize=12)
ax0.set_ylabel('Y Position (m)', fontsize=12)
ax0.legend(loc='lower left')
ax0.grid(True, linestyle='--', alpha=0.7)
ax0.axis('equal')

# (2) 우측 1: Heading (Yaw)
unwrapped_yaw = np.unwrap(df['yaw'].to_numpy())
ax1.plot(
    df['timestamp'].to_numpy(),
    unwrapped_yaw,
    label='Yaw Angle',
    color='purple',
    linewidth=2
)
ax1.set_title('Heading (Yaw)', fontsize=14, fontweight='bold')
ax1.set_ylabel('Yaw (rad)', fontsize=12)
ax1.legend(loc='upper right')
ax1.grid(True, linestyle='--', alpha=0.7)

# (3) 우측 2: Angular Velocity
ax2.plot(
    df['timestamp'].to_numpy(),
    df['angular_w'].to_numpy(),
    label='Angular Velocity',
    color='red',
    linewidth=2
)
ax2.set_title('Angular Velocity', fontsize=14, fontweight='bold')
ax2.set_xlabel('Time (s)', fontsize=12)
ax2.set_ylabel('Vel (rad/s)', fontsize=12)
ax2.legend(loc='upper right')
ax2.grid(True, linestyle='--', alpha=0.7)

plt.tight_layout()
plt.show()