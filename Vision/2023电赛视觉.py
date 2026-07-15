 # my_23e.py
# 庐山派 K230 — 同心矩形检测 + 阈值调节 (触摸屏版)
#
# ── 框架来源 ──
#   硬件初始化 / FLAG 状态机 / 触摸UI  → 移植自 main.py
#   （庐山派 K230 已验证的框架）
#
# ── 算法来源 ──
#   同心矩形检测 + 红色激光跟踪 + 阈值调节预览  → 移植自 23e.py
#   （仅移植 problem_id==3|4 的逻辑，移除 UART 串口部分）
#
# ── FLAG 状态机 ──
#   flag=0  主菜单         右侧两个按钮: 开始 / 阈值调节
#   flag=1  同心矩形检测   实时检测内外矩形 + 红色激光，右侧退出按钮
#   flag=2  阈值调节       触屏调参 + 实时二值化预览，右侧 参数+/参数-/切换/退出

import time, utime, os, sys, math, struct
from media.sensor import *
from media.display import *
from media.media import *
from machine import FPIOA, TOUCH, UART


# ============================================================
#  工具函数  (移植自 23e.py)
# ============================================================

def sort_corners_clockwise(points):
    """将4个点按逆时针排序，保证一致的方向"""
    if len(points) != 4:
        return points
    cx = sum(p[0] for p in points) / 4
    cy = sum(p[1] for p in points) / 4
    # 按极角排序得到逆时针顺序
    return sorted(points, key=lambda p: math.atan2(p[1] - cy, p[0] - cx), reverse=False)


def draw_rect(img, corners, color=(0, 255, 0), draw_dots=True, thick=2):
    """绘制矩形边 + 可选角点圆"""
    for i in range(4):
        a, b = corners[i], corners[(i + 1) % 4]
        img.draw_line(a[0], a[1], b[0], b[1], color, thick)
    if draw_dots:
        for x, y in corners:
            img.draw_circle(x, y, 5, (0, 0, 255), thick, fill=False)


def is_rectangular(corners, tol=30):
    """检查四个角是否都接近90°（tol为容许偏差度），过滤畸形四边形"""
    for i in range(4):
        a = corners[i]
        b = corners[(i + 1) % 4]
        c = corners[(i + 2) % 4]
        v1 = (a[0] - b[0], a[1] - b[1])
        v2 = (c[0] - b[0], c[1] - b[1])
        dot = v1[0] * v2[0] + v1[1] * v2[1]
        m1 = math.sqrt(v1[0]**2 + v1[1]**2)
        m2 = math.sqrt(v2[0]**2 + v2[1]**2)
        if m1 < 2 or m2 < 2:
            return False
        cos_a = max(-1, min(1, dot / (m1 * m2)))
        if abs(math.degrees(math.acos(cos_a)) - 90) > tol:
            return False
    return True


def _lab_t_idx(param_idx, color_mode):
    """将 LAB 参数索引 (0-5) 映射到 T[] 实际索引
       color_mode: 0=红色(T[6-11]), 1=绿色(T[12-17])"""
    return (6 if color_mode == 0 else 12) + param_idx


# ============================================================
#  庐山派 K230 硬件初始化  (移植自 main.py)
# ============================================================

fpioa = FPIOA()
fpioa.set_function(11, FPIOA.UART2_TXD)
fpioa.set_function(12, FPIOA.UART2_RXD)
fpioa.set_function(2,  FPIOA.GPIO2)

sensor = Sensor(id=2)
sensor.reset()
sensor.set_framesize(width=640, height=360, chn=CAM_CHN_ID_0)
sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

Display.init(Display.ST7701, width=800, height=480, to_ide=False)
MediaManager.init()
sensor.run()

tp = TOUCH(0)                       # 触屏

# 串口初始化
uart = None
try:
    uart = UART(2, baudrate=115200)
    print("UART2 打开成功")
except Exception as e:
    print("UART2 打开失败:", e)


# ============================================================
#  全局变量
# ============================================================

# 画面尺寸
LCD_W, LCD_H = 640, 360
DISP_W, DISP_H = 800, 480

# ROI
_roi_w, _roi_h = LCD_W // 2, LCD_H // 2
_roi_x = (LCD_W - _roi_w) // 2
_roi_y = (LCD_H - _roi_h) // 2
DEFAULT_ROI        = (_roi_x, _roi_y, _roi_w, _roi_h)
DEFAULT_ROI_PTS    = [(_roi_x, _roi_y),
                      (_roi_x + _roi_w, _roi_y),
                      (_roi_x + _roi_w, _roi_y + _roi_h),
                      (_roi_x, _roi_y + _roi_h)]
ROI_CENTER         = (LCD_W // 2, LCD_H // 2)
cur_roi            = DEFAULT_ROI
cur_roi_pts        = DEFAULT_ROI_PTS[:]

# 阈值参数  (同 23e.py)
T = [10000,                         # 0  rect_th
     5000,  100000,                   # 1~2 area_min, area_max
     100, 255,                      # 3~4 bin_Lmin, bin_Lmax
     1.4,                           # 5  roi_scale
     9, 100,                        # 6~7 red_Lmin, red_Lmax
     15, 98,                        # 8~9 red_Amin, red_Amax
     -13, 77,                       # 10~11 red_Bmin, red_Bmax
     0, 255,                       # 12~13 green_Lmin, green_Lmax
     -13, -128,                     # 14~15 green_Amin, green_Amax
     -30, 50]                       # 16~17 green_Bmin, green_Bmax

PNAMES   = ["rect_th", "area_min", "area_max", "bin_Lmin", "bin_Lmax",
            "roi_s", "R_Lmin", "R_Lmax", "R_Amin", "R_Amax", "R_Bmin", "R_Bmax",
            "G_Lmin", "G_Lmax", "G_Amin", "G_Amax", "G_Bmin", "G_Bmax"]
PSTEP    = [1000, 500, 1000, 5, 5, 0.1, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5]
PLIMIT   = [(0,30000), (0,100000), (0,100000), (0,255), (0,255),
            (0.5,2), (0,100), (0,100), (-128,127), (-128,127), (-128,127), (-128,127),
            (0,255), (0,255), (-128,127), (-128,127), (-128,127), (-128,127)]
LAB_PARAM_NAMES = ["Lmin", "Lmax", "Amin", "Amax", "Bmin", "Bmax"]
p_idx    = 0                        # 当前选中的参数索引 (flag==3:LAB参数0-5)

# 检测结果
rect_coords  = []
red_coords   = []
green_coords = []
func_mode    = 1        # 1=功能1(矩形+红激光)  2=功能2(红+绿激光)
lab_color    = 0        # 0=红色  1=绿色 (用于 flag==3 LAB调节模式)

# FLAG 状态机
flag = 0        # 0=主菜单  1=检测  2=阈值调节

# 触摸防抖
last_touch = 0
DEBOUNCE   = 300

# 帧率
f_cnt = 0
f_last = utime.ticks_ms()
fps   = 0

# 路径插值与串口发送
PATH_N = 30               # 每条边中间插值点数
PATH_INTERVAL = 100        # 路径点切换间隔(ms)
path_idx = 0
last_path_move = 0
path_points = []

# 合成缓冲区
img_buf = image.Image(DISP_W, DISP_H, image.RGB565)


# ============================================================
#  UI 绘制  (移植自 main.py 的右侧按钮风格)
# ============================================================

def _btn(y, txt, color, h=120):
    """在右侧 640~800 区域绘制一个填充按钮"""
    img_buf.draw_rectangle(640, y, 160, h, color=color, thickness=2, fill=True)
    tw = len(txt) * 18
    img_buf.draw_string_advanced(640 + (160 - tw) // 2,
                                 y + (h - 30) // 2,
                                 30, txt, color=(0, 0, 0))

def ui_main():
    _btn(0,   "开始",     (0, 255, 0))
    _btn(120, "常用调节",  (255, 0, 0))
    _btn(240, "红绿LAB",  (0, 0, 255))
    mode_txt = "功能1" if func_mode == 1 else "功能2"
    _btn(360, mode_txt,   (0, 255, 255))

def ui_detect():
    _btn(360, "退出",     (0, 255, 255))

def ui_threshold():
    """常用调节：右侧 L_min/L_max，底部 roi_s"""
    # 右侧面板
    _btn(0,   "L_min+", (0, 255, 0),    h=96)
    _btn(96,  "L_min-", (255, 0, 0),    h=96)
    _btn(192, "L_max+", (0, 0, 255),    h=96)
    _btn(288, "L_max-", (0, 255, 255),  h=96)
    _btn(384, "退出",   (128, 128, 128), h=96)
    # 底部区域 roi_s 按钮 (y=360-480)
    img_buf.draw_string_advanced(10, LCD_H+5, 24,
        "roi_s={:.2f}".format(T[5]), (255, 255, 0))
    img_buf.draw_rectangle(0, LCD_H+36, 320, 84,
        color=(0, 255, 0), thickness=2, fill=True)
    img_buf.draw_string_advanced(80, LCD_H+50, 30, "roi_s-", (0, 0, 0))
    img_buf.draw_rectangle(320, LCD_H+36, 320, 84,
        color=(0, 255, 0), thickness=2, fill=True)
    img_buf.draw_string_advanced(400, LCD_H+50, 30, "roi_s+", (0, 0, 0))

def ui_lab_adjust():
    """LAB 调节 UI：右侧面板 + 底部参数选择条"""
    # 右侧面板
    _btn(0,   "红绿切换", (255, 128, 0), h=120)
    _btn(120, "+",          (0, 255, 0),  h=120)
    _btn(240, "-",          (255, 0, 0),  h=120)
    _btn(360, "退出",       (128, 128, 128), h=120)
    # 底部参数选择条 (0-640, 360-480)
    n = len(LAB_PARAM_NAMES)
    bw = 640 // n
    for i in range(n):
        x = i * bw
        if i == p_idx:
            img_buf.draw_rectangle(x, LCD_H, bw, 120,
                color=(0, 255, 255), thickness=2, fill=True)
        else:
            img_buf.draw_rectangle(x, LCD_H, bw, 120,
                color=(80, 80, 80), thickness=2)
        img_buf.draw_string_advanced(x + bw//2 - 20, LCD_H + 40, 24,
            LAB_PARAM_NAMES[i], (255, 255, 255))


# ============================================================
#  触摸分发  (移植自 main.py 的 flag 跳转系统)
# ============================================================

def on_touch_main(tx, ty):
    """主菜单触摸"""
    global flag, func_mode
    if not (640 < tx < 800): return
    if       0 < ty < 120:
        flag = 1 if func_mode == 1 else 4
    elif   120 < ty < 240:    flag = 2
    elif   240 < ty < 360:    flag = 3
    elif   360 < ty < 480:    func_mode = 2 if func_mode == 1 else 1

def on_touch_detect(tx, ty):
    """检测模式触摸"""
    global flag
    if 640 < tx < 800 and 360 < ty < 480:
        flag = 0

def on_touch_threshold(tx, ty):
    """简化阈值调节（仅处理退出，L_min/L_max在防抖外连续执行）"""
    global flag
    if 640 < tx < 800 and 384 < ty < 480:
        flag = 0

def on_touch_lab_adjust(tx, ty):
    """LAB 调节模式触摸：切换/退出/参数选择（防抖）"""
    global flag, p_idx, lab_color
    # 右侧面板：切换/退出（debounced）
    if 640 < tx < 800:
        if   0 < ty < 120:              # 切换红⇄绿
            lab_color = 1 - lab_color
            p_idx = 0
        elif 360 < ty < 480:            # 退出
            flag = 0
    # 底部参数选择条（debounced）
    if 0 < tx < 640 and 360 < ty < 480:
        n = len(LAB_PARAM_NAMES)
        bw = 640 // n
        i = int(tx // bw)
        if 0 <= i < n:
            p_idx = i


# ============================================================
#  串口发送  (移植自 画黑框.py send_combined_data)
# ============================================================

def send_combined_data(x1, y1, x2, y2, z=0):
    """
    整合发送数据包
    格式：帧头(0xAA) + x1(2B) + y1(2B) + x2(2B) + y2(2B) + z(2B) + 帧尾(0x55)
    共 12 字节，大端序 signed short
    """
    if uart is None:
        return
    header = 0xAA
    tail = 0x55
    try:
        def clamp_s16(v):
            return max(-32768, min(int(v), 32768))
        vals = [clamp_s16(v) for v in (x1, y1, x2, y2, z)]
        packet = struct.pack(">BhhhhhB", header, *vals, tail)
        uart.write(packet)
    except Exception:
        pass


def generate_path(pts, n):
    """在4个角点之间线性插值生成路径点（含角点），共 4*(n+1) 个点"""
    path = []
    for i in range(4):
        p1 = pts[i]
        p2 = pts[(i + 1) % 4]
        path.append(p1)
        for j in range(1, n + 1):
            alpha = j / (n + 1)
            x = int(p1[0] + (p2[0] - p1[0]) * alpha)
            y = int(p1[1] + (p2[1] - p1[1]) * alpha)
            path.append((x, y))
    return path


# ============================================================
#  功能: 同心矩形检测  (移植自 23e.py 题3/4)
# ============================================================

def run_detect(img):
    """在 img 上绘制检测结果（修改原图）"""
    global rect_coords, red_coords, cur_roi, cur_roi_pts, path_points, path_idx

    # 更新 ROI
    s = T[5]
    cx, cy = ROI_CENTER
    cur_roi_pts = [(int(cx + (x-cx)*s), int(cy + (y-cy)*s)) for (x,y) in DEFAULT_ROI_PTS]
    xs = [p[0] for p in cur_roi_pts]
    ys = [p[1] for p in cur_roi_pts]
    cur_roi = (min(xs), min(ys), max(xs)-min(xs), max(ys)-min(ys))

    # 先创建灰度拷贝（确保不含后续绘制的图形干扰）
    g = img.to_grayscale(copy=True)

    # 检测红色激光
    blobs = img.find_blobs([(T[6], T[7], T[8], T[9], T[10], T[11])], False, roi=cur_roi,
                           x_stride=2, y_stride=2, pixels_threshold=1,
                           merge=True, margin=True)
    if blobs:
        best = max(blobs, key=lambda b: (b.area(), -b.y()))
        red_coords = (best.cx(), best.cy())
        img.draw_cross(best.cx(), best.cy(), (0, 255, 0), 20, 3)

    # 在干净的灰度图上做二值化 + 膨胀
    g.binary([(T[3], T[4])]).dilate(1)

    rects = g.find_rects(threshold=T[0], roi=cur_roi)
    # 面积筛选
    rects = [r for r in rects if T[1] <= r.rect()[2] * r.rect()[3] <= T[2]]
    rects.sort(key=lambda r: r.rect()[2] * r.rect()[3], reverse=True)

    if len(rects) >= 2:
        outer = sort_corners_clockwise(rects[0].corners())
        inner = sort_corners_clockwise(rects[1].corners())
        # 对齐内外矩形的角点顺序（保证 outer[i] 和 inner[i] 是同一个角）
        dists = [(inner[j][0]-outer[0][0])**2 + (inner[j][1]-outer[0][1])**2 for j in range(4)]
        start = dists.index(min(dists))
        inner = inner[start:] + inner[:start]
        # 角度校验，过滤畸形四边形
        if is_rectangular(outer) and is_rectangular(inner):
            draw_rect(img, outer)
            draw_rect(img, inner)
            avg_pts = []
            for i in range(4):
                avg_pts.append((
                    (outer[i][0] + inner[i][0]) // 2,
                    (outer[i][1] + inner[i][1]) // 2
                ))
            rect_coords = avg_pts
            path_points = generate_path(avg_pts, PATH_N)
            if path_idx >= len(path_points):
                path_idx = 0
            for x, y in avg_pts:
                img.draw_circle(x, y, 3, (255, 255, 0), 2, fill=True)

    # ROI 框
    draw_rect(img, cur_roi_pts, (255, 255, 255), False, 2)
    del g


# ============================================================
#  功能: 红+绿激光检测 (功能2)
# ============================================================

def run_func2(img):
    """功能2：同时检测红色激光和绿色激光"""
    global red_coords, green_coords

    # 红色激光
    red_blobs = img.find_blobs([(T[6], T[7], T[8], T[9], T[10], T[11])], False,
                               x_stride=2, y_stride=2, pixels_threshold=1,
                               merge=True, margin=True)
    if red_blobs:
        best = max(red_blobs, key=lambda b: (b.area(), -b.y()))
        red_coords = (best.cx(), best.cy())
        img.draw_cross(best.cx(), best.cy(), (0, 255, 0), 20, 3)
        img.draw_string_advanced(best.cx()+15, best.cy()-15, 20, "R", (0, 255, 0))
    else:
        red_coords = []

    # 绿色激光
    green_blobs = img.find_blobs([(T[12], T[13], T[14], T[15], T[16], T[17])], False,
                                 x_stride=2, y_stride=2, pixels_threshold=1,
                                 merge=True, margin=True)
    if green_blobs:
        best = max(green_blobs, key=lambda b: (b.area(), -b.y()))
        green_coords = (best.cx(), best.cy())
        img.draw_cross(best.cx(), best.cy(), (255, 0, 255), 20, 3)
        img.draw_string_advanced(best.cx()+15, best.cy()-15, 20, "G", (255, 0, 255))
    else:
        green_coords = []


# ============================================================
#  功能: 阈值调节预览  (移植自 23e.py adjust_threshold_mode)
# ============================================================

def run_threshold_preview(img):
    """返回一张处理后的图像（二值化 + 矩形 + 激光），用于显示"""
    global cur_roi, cur_roi_pts

    s = T[5]
    cx, cy = ROI_CENTER
    cur_roi_pts = [(int(cx + (x-cx)*s), int(cy + (y-cy)*s)) for (x,y) in DEFAULT_ROI_PTS]
    xs = [p[0] for p in cur_roi_pts]
    ys = [p[1] for p in cur_roi_pts]
    cur_roi = (min(xs), min(ys), max(xs)-min(xs), max(ys)-min(ys))

    # 检测红色激光（在原图上）
    blobs = img.find_blobs([(T[6], T[7], T[8], T[9], T[10], T[11])], False, roi=cur_roi,
                           x_stride=2, y_stride=2, pixels_threshold=1,
                           merge=True, margin=True)

    # 二值化预览
    g = img.to_grayscale(copy=True).binary([(T[3], T[4])]).dilate(1)

    rects = g.find_rects(threshold=T[0], roi=cur_roi)
    # 面积筛选
    rects = [r for r in rects if T[1] <= r.rect()[2] * r.rect()[3] <= T[2]]
    if rects:
        for r in rects:
            pts = sort_corners_clockwise(r.corners())
            if is_rectangular(pts):
                draw_rect(g, pts)

    # 激光标记画到二值图上
    if blobs:
        best = max(blobs, key=lambda b: (b.area(), -b.y()))
        g.draw_cross(best.cx(), best.cy(), (0, 255, 255), 25, 3)

    draw_rect(g, cur_roi_pts, (255, 0, 0), False, 2)

    # 参数信息
    info1 = "L_min={}  L_max={}".format(T[3], T[4])
    g.draw_string_advanced(10, LCD_H - 55, 20, info1, color=(255, 255, 0))
    info2 = "roi_s={:.2f}".format(T[5])
    g.draw_string_advanced(10, LCD_H - 30, 20, info2, color=(255, 255, 0))
    return g


def run_lab_preview(img):
    """LAB 阈值二值化预览：逐像素 LAB 筛选，匹配=白，不匹配=黑"""
    global lab_color, cur_roi, cur_roi_pts

    # 更新 ROI
    s = T[5]
    cx, cy = ROI_CENTER
    cur_roi_pts = [(int(cx + (x-cx)*s), int(cy + (y-cy)*s)) for (x,y) in DEFAULT_ROI_PTS]
    xs = [p[0] for p in cur_roi_pts]
    ys = [p[1] for p in cur_roi_pts]
    cur_roi = (min(xs), min(ys), max(xs)-min(xs), max(ys)-min(ys))

    # 当前颜色 LAB 阈值
    lo = 6 if lab_color == 0 else 12
    lab_th = [(T[lo], T[lo+1], T[lo+2], T[lo+3], T[lo+4], T[lo+5])]

    # 创建彩色副本 → 逐像素 LAB 二值化
    g = image.Image(LCD_W, LCD_H, image.RGB565)
    g.draw_image(img, 0, 0)           # 复制原图
    g.binary(lab_th)                  # 每个像素 LAB 匹配→白, 不匹配→黑
    g = g.to_grayscale()              # 转灰度显示

    # ROI 框
    draw_rect(g, cur_roi_pts, (100, 100, 100), False, 2)

    # 参数信息
    mode_str = "RED" if lab_color == 0 else "GREEN"
    actual_idx = _lab_t_idx(p_idx, lab_color)
    info = "{}: {}={}".format(
        mode_str, LAB_PARAM_NAMES[p_idx], T[actual_idx])
    g.draw_string_advanced(10, LCD_H - 55, 22, info, color=(200, 200, 200))
    info2 = "{:>4} {:>4} {:>4} {:>4} {:>4} {:>4}".format(
        T[lo], T[lo+1], T[lo+2], T[lo+3], T[lo+4], T[lo+5])
    g.draw_string_advanced(10, LCD_H - 30, 18, info2, color=(150, 150, 150))
    return g


# ============================================================
#  主渲染  (移植自 main.py 的 run_function / flag 分发)
# ============================================================

def render(img):
    global f_cnt, fps, f_last

    # FPS
    f_cnt += 1
    now = utime.ticks_ms()
    dt = utime.ticks_diff(now, f_last)
    if dt > 1000:
        fps = f_cnt * 1000 / dt
        f_cnt = 0
        f_last = now
    fps_txt = "FPS: {:.1f}".format(fps) if fps > 0 else "FPS: --"

    if flag == 0:
        # ── 主菜单 ──
        img_buf.clear(roi=(0, 0, LCD_W, DISP_H))
        img_buf.draw_image(img, 0, 0)
        ui_main()
        img_buf.draw_string_advanced(10, 10, 24, fps_txt, (255, 255, 255))

    elif flag == 1:
        # ── 同心矩形检测 ──
        run_detect(img)                         # 直接在 img 上绘制
        img_buf.clear(roi=(0, 0, LCD_W, DISP_H))
        img_buf.draw_image(img, 0, 0)
        # 绘制当前串口发送的路径点（红色大圆）
        if path_points:
            px, py = path_points[path_idx]
            img_buf.draw_circle(px, py, 8, (0, 0, 255), thick=2, fill=True)
        ui_detect()
        img_buf.draw_string_advanced(10, 10, 24, fps_txt, (255, 255, 255))
        # 底部状态
        if rect_coords:
            img_buf.draw_string_advanced(10, LCD_H+5, 20,
                "rect OK  avg:{}".format(rect_coords), (0, 255, 0))
        else:
            img_buf.draw_string_advanced(10, LCD_H+5, 20,
                "rect: --", (255, 0, 0))
        if red_coords:
            img_buf.draw_string_advanced(10, LCD_H+30, 20,
                "laser:{}".format(red_coords), (255, 255, 0))

    elif flag == 2:
        # ── 常用调节：L_min/L_max + roi_s ──
        preview = run_threshold_preview(img)
        img_buf.clear(roi=(0, 0, LCD_W, DISP_H))
        img_buf.draw_image(preview, 0, 0)
        ui_threshold()
        img_buf.draw_string_advanced(10, 10, 24, fps_txt, (255, 255, 255))
        del preview

    elif flag == 3:
        # ── 红绿激光 LAB 阈值调节 ──
        preview = run_lab_preview(img)
        img_buf.clear(roi=(0, 0, LCD_W, DISP_H))
        img_buf.draw_image(preview, 0, 0)
        ui_lab_adjust()
        img_buf.draw_string_advanced(10, 10, 24, fps_txt, (255, 255, 255))
        del preview

    elif flag == 4:
        # ── 功能2：红+绿激光检测 ──
        run_func2(img)
        img_buf.clear(roi=(0, 0, LCD_W, DISP_H))
        img_buf.draw_image(img, 0, 0)
        ui_detect()
        img_buf.draw_string_advanced(10, 10, 24, fps_txt, (255, 255, 255))
        # 底部状态
        if red_coords:
            img_buf.draw_string_advanced(10, LCD_H+5, 20,
                "Red:{}".format(red_coords), (0, 255, 0))
        else:
            img_buf.draw_string_advanced(10, LCD_H+5, 20,
                "Red: --", (255, 0, 0))
        if green_coords:
            img_buf.draw_string_advanced(10, LCD_H+30, 20,
                "Green:{}".format(green_coords), (255, 255, 0))
        else:
            img_buf.draw_string_advanced(10, LCD_H+30, 20,
                "Green: --", (255, 0, 0))


# ============================================================
#  主循环  (移植自 main.py, 融入 FLAG 状态机 + 触摸 + 合成显示)
# ============================================================

try:
    while True:
        os.exitpoint()
        now = utime.ticks_ms()
        img = sensor.snapshot(chn=CAM_CHN_ID_0)     # 640x360

        # 触摸处理
        p = tp.read()
        if p:
            tx, ty = p[0].x, p[0].y

            # ── 连续调节 (无防抖) ──
            if flag == 2 and 640 < tx < 800:
                if       0 < ty < 96:      T[3] += PSTEP[3]           # L_min+
                elif   96 < ty < 192:      T[3] -= PSTEP[3]           # L_min-
                elif  192 < ty < 288:      T[4] += PSTEP[4]           # L_max+
                elif  288 < ty < 384:      T[4] -= PSTEP[4]           # L_max-
            if flag == 2 and 0 < tx < 640 and 360 < ty < 480:
                # 底部 roi_s 调节
                if tx < 320:
                    T[5] -= PSTEP[5]       # roi_s-
                else:
                    T[5] += PSTEP[5]       # roi_s+
            if flag == 3 and 640 < tx < 800:
                if 120 < ty < 240:          # +
                    idx = _lab_t_idx(p_idx, lab_color)
                    T[idx] += PSTEP[idx]
                elif 240 < ty < 360:        # -
                    idx = _lab_t_idx(p_idx, lab_color)
                    T[idx] -= PSTEP[idx]
            if flag == 2 or flag == 3:
                for i in range(len(T)):
                    lo, hi = PLIMIT[i]
                    T[i] = max(lo, min(T[i], hi))

            # ── 状态切换 (带防抖) ──
            if utime.ticks_diff(now, last_touch) > DEBOUNCE:
                if   flag == 0:   on_touch_main(tx, ty)
                elif flag == 1:   on_touch_detect(tx, ty)
                elif flag == 2:   on_touch_threshold(tx, ty)   # 仅退出
                elif flag == 3:   on_touch_lab_adjust(tx, ty)  # 切换/参数选择/退出
                elif flag == 4:   on_touch_detect(tx, ty)      # 退出
                # 触摸反馈
                img_buf.draw_cross(tx, ty, (255, 0, 0), 10, 3)
                last_touch = now

        render(img)
        Display.show_image(img_buf)

        # 串口发送（仅检测模式）
        if flag == 1:
            if path_points:
                if utime.ticks_diff(now, last_path_move) > PATH_INTERVAL:
                    path_idx = (path_idx + 1) % len(path_points)
                    last_path_move = now
            x1, y1 = path_points[path_idx] if path_points else (0, 0)
            x2, y2 = red_coords if red_coords else (x1, y1)
            send_combined_data(x1, y1, x2, y2)

        elif flag == 4:
            # 至少一个识别到时发送，缺失的点复用另一个的坐标（偏差0→舵机不动）
            if red_coords or green_coords:
                x1, y1 = red_coords if red_coords else green_coords
                x2, y2 = green_coords if green_coords else (x1, y1)
                send_combined_data(x1, y1, x2, y2)

        del img
        import gc
        gc.collect()
        time.sleep_ms(5)

except KeyboardInterrupt as e:
    print("用户停止:", e)
except BaseException as e:
    print("异常:", e)
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
