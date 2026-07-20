# ============================================================
# K230 红色方框检测 + 按键锁定 + UART 串口发送系统
# ============================================================

import time
import json
import image

try:
    import struct
except ImportError:
    import ustruct as struct

from machine import Pin, UART
try:
    from machine import FPIOA
    HAS_FPIOA = True
except ImportError:
    HAS_FPIOA = False

from media.sensor import *
from media.display import *
from media.media import *

# =========================
# 基础配置
# =========================

IMG_W = 800
IMG_H = 480
LCD_W = 800
LCD_H = 480
PROC_W = 400
PROC_H = 240

LCD_ONLY = False
DEBUG_PRINT_INTERVAL_MS = 500
DRAW_STATUS_TEXT = True
RED_THRESHOLD = (23, 54, 17, 48, -33, 22)
# 黑白棋子色块 LAB 阈值
BLACK_THRESHOLD = (0, 30, -15, 15, -15, 15)
WHITE_THRESHOLD = (66, 98, -40, -4, -19, 27)
# 棋子识别面积阈值（在 400x240 处理小图下的像素个数），用于过滤阴影与杂色干扰
BLACK_PIXELS_THRESHOLD = 250  # 只有黑色像素数大于此值才识别为黑棋
WHITE_PIXELS_THRESHOLD = 150  # 只有白色像素数大于此值才识别为白棋

# 红色棋盘外框追踪参数：优先保证抗遮挡、防误触发，再兼顾帧率
BOARD_MIN_W = 80
BOARD_MIN_H = 80
BOARD_ASPECT_MIN = 0.75
BOARD_ASPECT_MAX = 1.30
BOARD_MIN_AREA_RATIO = 0.04
BOARD_EDGE_BALANCE_MAX = 2.60
BOARD_MAX_JUMP_RATIO = 0.25
BOARD_STABLE_FRAMES = 2
MAX_BOARD_LOST_FRAMES = 30
DEBUG_DRAW_DETAIL = False


# 屏幕与摄像头画面反转/镜像配置
HMIRROR       = True       # 摄像头水平镜像（若摄像头左右颠倒设为 True）
VFLIP         = True       # 摄像头垂直翻转（若摄像头上下颠倒设为 True）
ROTATE_SCREEN = True        # 屏幕显示物理反转 180 度（若整个屏幕倒置安装，设为 True）

# 串口通信配置
UART_BAUDRATE = 115200
PACKET_HEADER = 0xAA            # 棋盘四边形数据包帧头
PACKET_HEADER_BLACK_OUT = 0xCC  # 棋盘外黑棋数据包帧头
PACKET_HEADER_BLACK_IN  = 0xEE  # 棋盘内黑棋数据包帧头
PACKET_HEADER_WHITE_OUT = 0xDD  # 棋盘外白棋数据包帧头
PACKET_HEADER_WHITE_IN  = 0xEF  # 棋盘内白棋数据包帧头
PACKET_TAIL   = 0x55            # 数据包帧尾

resize_warned = False

# =========================
# 图像缩小
# =========================

def make_process_image(img):
    global resize_warned
    x_scale = PROC_W / IMG_W
    y_scale = PROC_H / IMG_H

    try:
        proc_img = img.copy(x_scale=x_scale, y_scale=y_scale)
        proc_w = proc_img.width()
        proc_h = proc_img.height()
        sx = IMG_W / proc_w
        sy = IMG_H / proc_h
        return proc_img, proc_w, proc_h, sx, sy
    except Exception:
        if not resize_warned:
            print("图像缩放失败，降级使用原图检测")
            resize_warned = True

    return img, IMG_W, IMG_H, 1.0, 1.0


def sort_quad_corners(corners):
    """对四边形的 4 个顶点进行排序，返回顺序为: [左上, 右上, 右下, 左下]"""
    # 按照 Y 坐标排序，分出上方两个点和下方两个点
    pts = sorted(corners, key=lambda p: p[1])
    top = sorted(pts[0:2], key=lambda p: p[0])      # 上方的两个点按 X 排序，得到左上、右上
    bottom = sorted(pts[2:4], key=lambda p: p[0])   # 下方的两个点按 X 排序，得到左下、右下

    tl = top[0]
    tr = top[1]
    br = bottom[1]
    bl = bottom[0]
    return tl, tr, br, bl

def get_quad_point(tl, tr, br, bl, u, v):
    """利用双线性插值计算四边形在参数 (u, v) 处的投影坐标"""
    # 顶部和底部水平插值
    tx = (1 - u) * tl[0] + u * tr[0]
    ty = (1 - u) * tl[1] + u * tr[1]

    bx = (1 - u) * bl[0] + u * br[0]
    by = (1 - u) * bl[1] + u * br[1]

    # 垂直插值
    px = int((1 - v) * tx + v * bx)
    py = int((1 - v) * ty + v * by)
    return (px, py)

def is_point_in_quad(p, quad):
    """判断点 p 是否在凸四边形 quad (由 4 个顶点组成的列表) 内部 (使用向量叉乘法)"""
    px, py = p[0], p[1]
    signs = []
    for i in range(4):
        p1 = quad[i]
        p2 = quad[(i + 1) % 4]
        # 计算向量 (p1->p2) 与 (p1->p) 的 2D 叉乘
        cross = (p2[0] - p1[0]) * (py - p1[1]) - (p2[1] - p1[1]) * (px - p1[0])
        if cross > 0:
            signs.append(1)
        elif cross < 0:
            signs.append(-1)
        else:
            signs.append(0)
    # 如果乘积符号既有正又有负，说明在四边形外部；否则在内部（或边界上）
    has_pos = any(s > 0 for s in signs)
    has_neg = any(s < 0 for s in signs)
    return not (has_pos and has_neg)

def is_inside_board(p, locked_rects):
    """判断点 p 是否在当前的任何一个锁定网格线框内"""
    if not locked_rects:
        return False
    for quad in locked_rects:
        if is_point_in_quad(p, quad):
            return True
    return False


def dist2(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return dx * dx + dy * dy


def quad_bbox(corners):
    min_x = min(p[0] for p in corners)
    max_x = max(p[0] for p in corners)
    min_y = min(p[1] for p in corners)
    max_y = max(p[1] for p in corners)
    return min_x, min_y, max_x, max_y


def quad_area(corners):
    area = 0
    for i in range(4):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) % 4]
        area += x1 * y2 - x2 * y1
    if area < 0:
        area = -area
    return area / 2.0


def quad_center(corners):
    sx = 0
    sy = 0
    for p in corners:
        sx += p[0]
        sy += p[1]
    return (int(sx / 4), int(sy / 4))


def corners_motion_ratio(prev_corners, curr_corners):
    if prev_corners is None or curr_corners is None:
        return 0.0

    min_x, min_y, max_x, max_y = quad_bbox(prev_corners)
    diag2 = (max_x - min_x) * (max_x - min_x) + (max_y - min_y) * (max_y - min_y)
    if diag2 <= 0:
        return 1.0

    max_move2 = 0
    for i in range(4):
        d2 = dist2(prev_corners[i], curr_corners[i])
        if d2 > max_move2:
            max_move2 = d2
    return (max_move2 / diag2) ** 0.5


def validate_board_corners(corners, min_w, min_h):
    if corners is None or len(corners) != 4:
        return False

    min_x, min_y, max_x, max_y = quad_bbox(corners)
    w = max_x - min_x
    h = max_y - min_y
    if w < min_w or h < min_h:
        return False

    ratio = w / max(h, 1)
    if ratio < BOARD_ASPECT_MIN or ratio > BOARD_ASPECT_MAX:
        return False

    area = quad_area(corners)
    if area < IMG_W * IMG_H * BOARD_MIN_AREA_RATIO:
        return False

    # 四条边长度不能严重失衡，避免细长红色干扰物或异常角点被当作棋盘
    edges = []
    for i in range(4):
        d = dist2(corners[i], corners[(i + 1) % 4])
        if d <= 0:
            return False
        edges.append(d)
    shortest = min(edges)
    longest = max(edges)
    if longest / max(shortest, 1) > BOARD_EDGE_BALANCE_MAX * BOARD_EDGE_BALANCE_MAX:
        return False

    # 允许角点略出界，但过度异常的候选直接拒绝
    margin = 20
    for x, y in corners:
        if x < -margin or x > IMG_W + margin or y < -margin or y > IMG_H + margin:
            return False

    return True


def score_board_candidate(corners, prev_corners):
    min_x, min_y, max_x, max_y = quad_bbox(corners)
    bbox_area = (max_x - min_x) * (max_y - min_y)
    area = quad_area(corners)
    score = area + bbox_area * 0.2

    ratio = (max_x - min_x) / max((max_y - min_y), 1)
    ratio_error = abs(ratio - 1.0)
    score -= ratio_error * 3000

    if prev_corners is not None:
        motion = corners_motion_ratio(prev_corners, corners)
        if motion <= BOARD_MAX_JUMP_RATIO:
            score += 8000
        else:
            score -= motion * 12000

    return score


def select_best_board_rect(rects, sx, sy, prev_corners):
    if not rects:
        return None

    best_corners = None
    best_score = -999999999
    min_w = BOARD_MIN_W * sx
    min_h = BOARD_MIN_H * sy

    for r in rects:
        try:
            if r.w() <= BOARD_MIN_W or r.h() <= BOARD_MIN_H:
                continue
            corners = r.corners()
        except Exception:
            continue

        scaled_corners = [(int(p[0] * sx), int(p[1] * sy)) for p in corners]
        sorted_corners = sort_quad_corners(scaled_corners)
        if not validate_board_corners(sorted_corners, min_w, min_h):
            continue

        score = score_board_candidate(sorted_corners, prev_corners)
        if score > best_score:
            best_score = score
            best_corners = sorted_corners

    return best_corners


# =========================
# 人机博弈 (三子棋/井字棋) 算法及串口指令
# =========================

def check_win(board, player):
    """判断指定玩家是否在当前棋盘上获胜"""
    # 获胜线定义（横、竖、斜）
    win_lines = [
        [0, 1, 2], [3, 4, 5], [6, 7, 8], # 横行
        [0, 3, 6], [1, 4, 7], [2, 5, 8], # 竖列
        [0, 4, 8], [2, 4, 6]             # 对角线
    ]
    for line in win_lines:
        if board[line[0]] == player and board[line[1]] == player and board[line[2]] == player:
            return True
    return False

def get_ai_move(board):
    """人机博弈三子棋 AI（落子优先级：能赢直赢 -> 能防死防 -> 占中 -> 占角 -> 占边）"""
    # 1. 尝试直接赢得游戏
    for i in range(9):
        if board[i] is None:
            temp_board = list(board)
            temp_board[i] = 'W'
            if check_win(temp_board, 'W'):
                return i

    # 2. 尝试防守，阻止人类获胜
    for i in range(9):
        if board[i] is None:
            temp_board = list(board)
            temp_board[i] = 'B'
            if check_win(temp_board, 'B'):
                return i

    # 3. 抢占中心点
    if board[4] is None:
        return 4

    # 4. 抢占角点
    corners = [0, 2, 6, 8]
    empty_corners = [c for c in corners if board[c] is None]
    if empty_corners:
        return empty_corners[0]

    # 5. 抢占剩余边缘点
    empty_cells = [i for i in range(9) if board[i] is None]
    if empty_cells:
        return empty_cells[0]

    return -1

def send_ai_move(uart, cell_idx, target_center):
    """通过串口2发送AI的下子目标中心点和索引
    数据包格式: [帧头 0xFF] [格子索引 cell_idx] [cx_H cx_L cy_H cy_L] [校验和] [帧尾 0x55]
    校验和 = (cell_idx + cx_H + cx_L + cy_H + cy_L) & 0xFF
    """
    cx, cy = target_center
    xh = (cx >> 8) & 0xFF
    xl = cx & 0xFF
    yh = (cy >> 8) & 0xFF
    yl = cy & 0xFF

    buf = bytearray()
    buf.append(0xFF)            # 帧头 0xFF：AI落子指令
    buf.append(cell_idx & 0xFF) # 落子单元格索引 (0-8)
    buf.append(xh)
    buf.append(xl)
    buf.append(yh)
    buf.append(yl)

    checksum = (cell_idx + xh + xl + yh + yl) & 0xFF
    buf.append(checksum)
    buf.append(0x55)            # 帧尾 0x55

    uart.write(buf)
    print(f"[UART2] 发送 AI 下子指令 -> 格子 {cell_idx}, 坐标 ({cx}, {cy})")



# =========================
# 串口数据包发送
# =========================

def send_rect_data(uart, rect_centers):
    """将所有矩形中心点坐标以数据包格式通过串口发送
    数据包格式: [帧头 0xAA] [矩形数量 N] [x1_H x1_L y1_H y1_L] ... [xN_H xN_L yN_H yN_L] [校验和] [帧尾 0x55]
    校验和 = (N + 所有坐标字节) & 0xFF
    """
    n = len(rect_centers)
    if n == 0:
        return

    # 构建数据包
    buf = bytearray()
    buf.append(PACKET_HEADER)       # 帧头
    buf.append(n & 0xFF)            # 矩形数量

    checksum = n
    for (cx, cy) in rect_centers:
        xh = (cx >> 8) & 0xFF
        xl = cx & 0xFF
        yh = (cy >> 8) & 0xFF
        yl = cy & 0xFF
        buf.append(xh)
        buf.append(xl)
        buf.append(yh)
        buf.append(yl)
        checksum += xh + xl + yh + yl

    buf.append(checksum & 0xFF)     # 校验和
    buf.append(PACKET_TAIL)         # 帧尾

    uart.write(buf)
    print(f"[UART1] 已发送 {n} 个矩形中心: {rect_centers}")


last_uart2_print_ms = 0

def send_blob_data(uart, header, blob_centers):
    """将色块的中心点坐标以指定帧头的数据包格式通过串口2发送
    数据包格式: [帧头] [色块数量 N] [x1_H x1_L y1_H y1_L] ... [校验和] [帧尾 0x55]
    """
    global last_uart2_print_ms
    n = len(blob_centers)
    buf = bytearray()
    buf.append(header)              # 传入的自定义帧头
    buf.append(n & 0xFF)            # 数量

    checksum = n
    for (cx, cy) in blob_centers:
        xh = (cx >> 8) & 0xFF
        xl = cx & 0xFF
        yh = (cy >> 8) & 0xFF
        yl = cy & 0xFF
        buf.append(xh)
        buf.append(xl)
        buf.append(yh)
        buf.append(yl)
        checksum += xh + xl + yh + yl

    buf.append(checksum & 0xFF)     # 校验和
    buf.append(PACKET_TAIL)         # 帧尾

    uart.write(buf)

    # 节流控制台打印，每500ms打印一次，避免数据洪流导致IDE卡顿
    now = time.ticks_ms()
    if time.ticks_diff(now, last_uart2_print_ms) > 500:
        if header == PACKET_HEADER_WHITE_IN:
            last_uart2_print_ms = now

        if header == PACKET_HEADER_BLACK_OUT:
            color_str = "黑棋(外)"
        elif header == PACKET_HEADER_BLACK_IN:
            color_str = "黑棋(内)"
        elif header == PACKET_HEADER_WHITE_OUT:
            color_str = "白棋(外)"
        else:
            color_str = "白棋(内)"
        print(f"[UART2] 实时已发送 {n} 个{color_str}中心: {blob_centers}")


# =========================
# 绘图与标注
# =========================

def draw_lcd_status(img, has_target, b_out, b_in, w_out, w_in, fps, proc_w, proc_h,
                    game_mode=1, game_winner=None, ai_next_move=-1, board_source="LOST", board_lost_frames=0):
    if not DRAW_STATUS_TEXT:
        return

    if board_source == "LIVE":
        img.draw_string_advanced(5, 5, 18, "BOARD: LIVE", color=(0, 255, 0))
    elif board_source == "HISTORY":
        img.draw_string_advanced(5, 5, 18, f"BOARD: HISTORY {board_lost_frames}", color=(255, 255, 0))
    elif board_source == "LOCKED" and has_target:
        img.draw_string_advanced(5, 5, 18, "BOARD: LOCKED", color=(255, 128, 0))
    else:
        img.draw_string_advanced(5, 5, 18, "BOARD: LOST", color=(255, 0, 0))

    if game_mode == 1:
        img.draw_string_advanced(5, 25, 16, "MODE: 1 (TRACKING)", color=(0, 255, 255))
        img.draw_string_advanced(5, 45, 16, f"BLACK: OUT={b_out} IN={b_in}", color=(255, 255, 255))
        img.draw_string_advanced(5, 65, 16, f"WHITE: OUT={w_out} IN={w_in}", color=(255, 255, 255))
    else:
        img.draw_string_advanced(5, 25, 16, "MODE: 2 (GAME PLAY)", color=(255, 255, 0))
        # 显示博弈状态与输赢结果
        if game_winner == 'B':
            winner_text = "HUMAN (BLACK) WINS!"
            txt_color = (0, 255, 0)
        elif game_winner == 'W':
            winner_text = "AI (WHITE) WINS!"
            txt_color = (255, 0, 0)
        elif game_winner == 'Draw':
            winner_text = "DRAW GAME!"
            txt_color = (255, 255, 255)
        else:
            winner_text = "YOUR TURN (BLACK)"
            txt_color = (0, 255, 0)

        img.draw_string_advanced(5, 45, 16, winner_text, color=txt_color)

        if ai_next_move != -1:
            img.draw_string_advanced(5, 65, 16, f"AI MOVE: CELL {ai_next_move}", color=(255, 128, 0))
        else:
            img.draw_string_advanced(5, 65, 16, "WAITING FOR MOVE...", color=(255, 255, 255))

    img.draw_string_advanced(5, 85, 18, f"FPS: {fps:.1f}", color=(0, 255, 255))
    img.draw_string_advanced(5, 105, 18, f"PROC: {proc_w}x{proc_h}", color=(255, 255, 255))


# =========================
# 主程序
# =========================

def main():
    # 将变量提前声明到外部，确保 finally 块在任何时候都能获取并释放它们
    sensor = None
    media_inited = False
    display_inited = False

    print("========== K230 红色方框检测 + 按键锁定系统已启动 ==========")

    try:
        # 1. 硬件初始化

        # --- FPIOA 引脚功能映射 ---
        if HAS_FPIOA:
            try:
                fpioa = FPIOA()
                fpioa.set_function(53, FPIOA.GPIO53)      # GPIO53 → 按键输入
                fpioa.set_function(3,  FPIOA.UART1_TXD)   # GPIO3  → UART1 TX
                fpioa.set_function(4,  FPIOA.UART1_RXD)   # GPIO4  → UART1 RX
                fpioa.set_function(5,  FPIOA.UART2_TXD)   # GPIO 5 (对应板载排针物理 11 号脚) → UART2 TX
                fpioa.set_function(6,  FPIOA.UART2_RXD)   # GPIO 6 (对应板载排针物理 13 号脚) → UART2 RX
            except Exception as e:
                print(f"[警告] FPIOA 初始化/设置引脚映射失败: {e}，将尝试直接使用 Pin/UART")
        else:
            print("[信息] 当前固件无 FPIOA 模块，跳过引脚重映射，直接进行硬件初始化")

        # --- 按键初始化：GPIO53，下拉输入，按下为高电平 ---
        button = Pin(53, Pin.IN, Pin.PULL_DOWN)

        # --- 串口1初始化：UART1, 115200, 8N1 ---
        uart_id = 1
        if hasattr(UART, 'UART1'):
            uart_id = UART.UART1

        try:
            uart1 = UART(uart_id, baudrate=UART_BAUDRATE, bits=8, parity=None, stop=1)
        except Exception:
            # 简化版构造函数以兼容部分固件
            uart1 = UART(uart_id, baudrate=UART_BAUDRATE)

        # --- 串口2初始化：UART2, 115200, 8N1 ---
        uart2_id = 2
        if hasattr(UART, 'UART2'):
            uart2_id = UART.UART2

        try:
            uart2 = UART(uart2_id, baudrate=UART_BAUDRATE, bits=8, parity=None, stop=1)
        except Exception:
            # 简化版构造函数以兼容部分固件
            uart2 = UART(uart2_id, baudrate=UART_BAUDRATE)

        print(f"[初始化] GPIO53 按键就绪 | UART1 串口就绪 | UART2(排针物理11,13脚) 串口就绪")

        # --- 摄像头与显示初始化 ---
        sensor = Sensor(width=IMG_W, height=IMG_H)
        sensor.reset()
        sensor.set_hmirror(HMIRROR)
        sensor.set_vflip(VFLIP)
        sensor.set_framesize(width=IMG_W, height=IMG_H)
        sensor.set_pixformat(Sensor.RGB565)

        Display.init(Display.ST7701, width=LCD_W, height=LCD_H, to_ide=(not LCD_ONLY))
        display_inited = True

        MediaManager.init()
        media_inited = True

        sensor.run()

        clock = time.clock()
        last_print_ms = time.ticks_ms()
        proc_w_show = PROC_W
        proc_h_show = PROC_H

        # --- 按键锁定状态 ---
        frozen = False           # 是否处于冻结状态
        last_btn_state = 0       # 上一次按键状态（用于边沿检测）

        # 存储锁定/当前检测的四边形信息
        locked_rects = []        # 存放四边形的顶点集 [[(x1,y1), ...], ...]
        locked_centers = []      # 存放对应的中心点 [(cx, cy), ...]

        # --- 棋盘线框追踪与记忆机制参数 ---
        last_good_board_corners = None  # 上一次成功检测的棋盘角点在大图下的位置 [(x,y), ...]
        board_lost_frames = 0           # 连续丢失检测的帧数计数器
        pending_board_corners = None    # 跳变较大的候选棋盘，需连续稳定后才接受
        pending_board_count = 0
        board_source = "LOST"           # LIVE / HISTORY / LOST，用于屏幕显示与调试

        # --- 游戏博弈模式参数 ---
        game_mode = 1                  # 1 = 正常数据回传模式, 2 = 人机博弈模式
        board_state = [None] * 9       # 棋盘格状态
        game_winner = None             # 获胜者: 'B', 'W', 'Draw', None
        ai_next_move = -1              # AI 下一步落子格索引 (0-8)
        cell_stable_counters = [0] * 9  # 落子消抖计数器，用于防止手指阴影误触发

        # 2. 主循环
        while True:
            clock.tick()

            # ===== 按键边沿检测（支持长按切换模式，短按冻结） =====
            btn_state = button.value()
            btn_pressed = (btn_state == 1 and last_btn_state == 0)  # 上升沿
            last_btn_state = btn_state

            if btn_pressed:
                time.sleep_ms(20)  # 消抖
                if button.value() == 1:
                    # 长按检测：按住超过 1.5 秒视为长按
                    press_time = time.ticks_ms()
                    is_long_press = False
                    while button.value() == 1:
                        if time.ticks_diff(time.ticks_ms(), press_time) > 1500:
                            is_long_press = True
                            break
                        time.sleep_ms(10)

                    if is_long_press:
                        # 触发长按：切换博弈模式
                        game_mode = 2 if game_mode == 1 else 1
                        print(f"[按键] ▶ 长按检测触发！模式切换为: 模式 {game_mode}")
                        # 重置博弈状态
                        board_state = [None] * 9
                        game_winner = None
                        ai_next_move = -1
                        # 等待按键松开，防止连击
                        while button.value() == 1:
                            time.sleep_ms(10)
                    else:
                        # 触发短按：切换锁定状态
                        frozen = not frozen
                        if frozen:
                            print("[按键] ▶ 棋盘线框已锁定，发送数据...")
                            if locked_centers:
                                send_rect_data(uart1, locked_centers)
                            else:
                                print("[UART] 未检测到任何线框，无数据发送")
                        else:
                            print("[按键] ▶ 棋盘线框已解锁，恢复实时跟踪")

            # ===== 实时获取画面 =====
            img = sensor.snapshot()
            if img is None:
                continue

            # 处理小图
            proc_img, proc_w, proc_h, sx, sy = make_process_image(img)
            proc_w_show = proc_w
            proc_h_show = proc_h

            # ===== 【1】实时棋子检测（实时更新，包含黑/白双色） =====
            # 1. 黑棋检测
            black_blobs = proc_img.find_blobs(
                [BLACK_THRESHOLD],
                pixels_threshold=BLACK_PIXELS_THRESHOLD,
                area_threshold=BLACK_PIXELS_THRESHOLD,
                merge=True
            )
            black_count_out = 0
            black_count_in = 0
            black_centers_out = []
            black_centers_in = []
            if black_blobs:
                for b in black_blobs:
                    bw, bh = b[2], b[3]
                    area = bw * bh
                    if area == 0: continue
                    density = b[4] / area
                    # 黑棋是实心圆形盘，密度应该较高，调高阈值以过滤边缘模糊的指缝手影
                    if density < 0.55: continue
                    aspect = max(bw, bh) / max(min(bw, bh), 1)
                    # 圆形棋子宽高比接近 1.0，收紧至 1.35，过滤掉细长条状的手指阴影
                    if aspect > 1.35: continue
                    if b[4] < BLACK_PIXELS_THRESHOLD: continue

                    b_cx = int(b[5] * sx)
                    b_cy = int(b[6] * sy)
                    b_x  = int(b[0] * sx)
                    b_y  = int(b[1] * sy)
                    b_w  = int(bw * sx)
                    b_h  = int(bh * sy)

                    # 判断是在棋盘内还是棋盘外
                    if is_inside_board((b_cx, b_cy), locked_rects):
                        black_count_in += 1
                        black_centers_in.append((b_cx, b_cy))
                        # 绘制框：用蓝色，但标注为 B_IN
                        img.draw_rectangle(b_x, b_y, b_w, b_h, color=(0, 0, 255), thickness=2)
                        img.draw_cross(b_cx, b_cy, color=(0, 0, 255), size=8, thickness=2)
                        if DEBUG_DRAW_DETAIL:
                            img.draw_string_advanced(b_cx + 6, b_cy - 10, 14, f"B_IN({b_cx},{b_cy})", color=(0, 0, 255))
                    else:
                        black_count_out += 1
                        black_centers_out.append((b_cx, b_cy))
                        # 绘制框：用蓝色，标注为 B_OUT
                        img.draw_rectangle(b_x, b_y, b_w, b_h, color=(0, 0, 255), thickness=2)
                        img.draw_cross(b_cx, b_cy, color=(0, 0, 255), size=8, thickness=2)
                        if DEBUG_DRAW_DETAIL:
                            img.draw_string_advanced(b_cx + 6, b_cy - 10, 14, f"B_OUT({b_cx},{b_cy})", color=(0, 0, 255))

            # 2. 白棋检测
            white_blobs = proc_img.find_blobs(
                [WHITE_THRESHOLD],
                pixels_threshold=WHITE_PIXELS_THRESHOLD,
                area_threshold=WHITE_PIXELS_THRESHOLD,
                merge=True
            )
            white_count_out = 0
            white_count_in = 0
            white_centers_out = []
            white_centers_in = []
            if white_blobs:
                for b in white_blobs:
                    bw, bh = b[2], b[3]
                    area = bw * bh
                    if area == 0: continue
                    density = b[4] / area
                    if density < 0.4: continue
                    aspect = max(bw, bh) / max(min(bw, bh), 1)
                    if aspect > 4.0: continue
                    if b[4] < WHITE_PIXELS_THRESHOLD: continue

                    w_cx = int(b[5] * sx)
                    w_cy = int(b[6] * sy)
                    w_x  = int(b[0] * sx)
                    w_y  = int(b[1] * sy)
                    w_w  = int(bw * sx)
                    w_h  = int(bh * sy)

                    # 判断是在棋盘内还是棋盘外
                    if is_inside_board((w_cx, w_cy), locked_rects):
                        white_count_in += 1
                        white_centers_in.append((w_cx, w_cy))
                        # 绘制框：用橙色，标注为 W_IN
                        img.draw_rectangle(w_x, w_y, w_w, w_h, color=(255, 128, 0), thickness=2)
                        img.draw_cross(w_cx, w_cy, color=(255, 128, 0), size=8, thickness=2)
                        if DEBUG_DRAW_DETAIL:
                            img.draw_string_advanced(w_cx + 6, w_cy - 10, 14, f"W_IN({w_cx},{w_cy})", color=(255, 128, 0))
                    else:
                        white_count_out += 1
                        white_centers_out.append((w_cx, w_cy))
                        # 绘制框：用橙色，标注为 W_OUT
                        img.draw_rectangle(w_x, w_y, w_w, w_h, color=(255, 128, 0), thickness=2)
                        img.draw_cross(w_cx, w_cy, color=(255, 128, 0), size=8, thickness=2)
                        if DEBUG_DRAW_DETAIL:
                            img.draw_string_advanced(w_cx + 6, w_cy - 10, 14, f"W_OUT({w_cx},{w_cy})", color=(255, 128, 0))

            # --- 对黑棋（内/外）分别进行由上至下、由左至右排序 ---
            if black_centers_out:
                black_centers_out.sort(key=lambda p: p[1])
                black_rows = []
                current_row = [black_centers_out[0]]
                for p in black_centers_out[1:]:
                    if p[1] - current_row[0][1] < 40:
                        current_row.append(p)
                    else:
                        black_rows.append(current_row)
                        current_row = [p]
                black_rows.append(current_row)

                sorted_black_centers = []
                for row in black_rows:
                    row.sort(key=lambda p: p[0])
                    sorted_black_centers.extend(row)
                black_centers_out = sorted_black_centers

            if black_centers_in:
                black_centers_in.sort(key=lambda p: p[1])
                black_rows = []
                current_row = [black_centers_in[0]]
                for p in black_centers_in[1:]:
                    if p[1] - current_row[0][1] < 40:
                        current_row.append(p)
                    else:
                        black_rows.append(current_row)
                        current_row = [p]
                black_rows.append(current_row)

                sorted_black_centers = []
                for row in black_rows:
                    row.sort(key=lambda p: p[0])
                    sorted_black_centers.extend(row)
                black_centers_in = sorted_black_centers

            # --- 对白棋（内/外）分别进行由上至下、由左至右排序 ---
            if white_centers_out:
                white_centers_out.sort(key=lambda p: p[1])
                white_rows = []
                current_row = [white_centers_out[0]]
                for p in white_centers_out[1:]:
                    if p[1] - current_row[0][1] < 40:
                        current_row.append(p)
                    else:
                        white_rows.append(current_row)
                        current_row = [p]
                white_rows.append(current_row)

                sorted_white_centers = []
                for row in white_rows:
                    row.sort(key=lambda p: p[0])
                    sorted_white_centers.extend(row)
                white_centers_out = sorted_white_centers

            if white_centers_in:
                white_centers_in.sort(key=lambda p: p[1])
                white_rows = []
                current_row = [white_centers_in[0]]
                for p in white_centers_in[1:]:
                    if p[1] - current_row[0][1] < 40:
                        current_row.append(p)
                    else:
                        white_rows.append(current_row)
                        current_row = [p]
                white_rows.append(current_row)

                sorted_white_centers = []
                for row in white_rows:
                    row.sort(key=lambda p: p[0])
                    sorted_white_centers.extend(row)
                white_centers_in = sorted_white_centers

            # ===== 【2】红色四边形（棋盘外框）检测与 9 等分细分 =====
            proc_img.binary([RED_THRESHOLD])
            proc_img.dilate(2)
            rects = proc_img.find_rects(threshold=2000)

            live_rects = []
            live_centers = []
            board_corners = None
            board_source = "LOST"

            candidate_corners = select_best_board_rect(rects, sx, sy, last_good_board_corners)
            if candidate_corners is not None:
                accept_candidate = False
                if last_good_board_corners is None:
                    accept_candidate = True
                else:
                    motion = corners_motion_ratio(last_good_board_corners, candidate_corners)
                    if motion <= BOARD_MAX_JUMP_RATIO:
                        accept_candidate = True
                    else:
                        if pending_board_corners is not None and \
                           corners_motion_ratio(pending_board_corners, candidate_corners) <= BOARD_MAX_JUMP_RATIO:
                            pending_board_count += 1
                        else:
                            pending_board_corners = candidate_corners
                            pending_board_count = 1

                        if pending_board_count >= BOARD_STABLE_FRAMES:
                            accept_candidate = True

                if accept_candidate:
                    board_corners = candidate_corners
                    last_good_board_corners = candidate_corners
                    pending_board_corners = None
                    pending_board_count = 0
                    board_lost_frames = 0
                    board_source = "LIVE"
                elif last_good_board_corners is not None and board_lost_frames < MAX_BOARD_LOST_FRAMES:
                    board_lost_frames += 1
                    board_corners = last_good_board_corners
                    board_source = "HISTORY"
            elif last_good_board_corners is not None and board_lost_frames < MAX_BOARD_LOST_FRAMES:
                board_lost_frames += 1
                board_corners = last_good_board_corners
                board_source = "HISTORY"
            else:
                pending_board_corners = None
                pending_board_count = 0

            # 如果成功获取到了棋盘角点（无论是正常识别、历史外推，还是色块保底）
            if board_corners is not None:
                tl, tr, br, bl = board_corners

                # 双线性插值 3x3 共 9 个网格
                temp_rect_datas = []
                for i in range(3):       # 行 (从上到下)
                    for j in range(3):   # 列 (从左到右)
                        u0, u1 = j / 3.0, (j + 1) / 3.0
                        v0, v1 = i / 3.0, (i + 1) / 3.0

                        c0 = get_quad_point(tl, tr, br, bl, u0, v0)
                        c1 = get_quad_point(tl, tr, br, bl, u1, v0)
                        c2 = get_quad_point(tl, tr, br, bl, u1, v1)
                        c3 = get_quad_point(tl, tr, br, bl, u0, v1)

                        uc, vc = (j + 0.5) / 3.0, (i + 0.5) / 3.0
                        cx, cy = get_quad_point(tl, tr, br, bl, uc, vc)

                        temp_rect_datas.append({
                            "corners": [c0, c1, c2, c3],
                            "center": (cx, cy)
                        })

                live_rects = [d["corners"] for d in temp_rect_datas]
                live_centers = [d["center"] for d in temp_rect_datas]

            # 实时通过 UART2 发送实时黑棋和白棋（棋盘外、棋盘内）的中心坐标信息
            send_blob_data(uart2, PACKET_HEADER_BLACK_OUT, black_centers_out)
            send_blob_data(uart2, PACKET_HEADER_BLACK_IN, black_centers_in)
            send_blob_data(uart2, PACKET_HEADER_WHITE_OUT, white_centers_out)
            send_blob_data(uart2, PACKET_HEADER_WHITE_IN, white_centers_in)

            # 如果未锁定，将可靠棋盘坐标同步更新；短时遮挡时允许使用历史坐标，避免网格闪烁或跳变
            if not frozen:
                if live_rects and board_source != "LOST":
                    locked_rects = live_rects
                    locked_centers = live_centers

            # ===== 如果是模式 2 (博弈模式)，执行人机博弈逻辑 =====
            if game_mode == 2:
                if len(locked_centers) == 9:
                    # 1. 扫描当前物理棋盘状态
                    detected_board = [None] * 9
                    for i in range(9):
                        cell_quad = locked_rects[i]
                        # 检查是否有黑棋落入
                        has_black = False
                        for b_pt in black_centers_in:
                            if is_point_in_quad(b_pt, cell_quad):
                                has_black = True
                                break
                        if has_black:
                            detected_board[i] = 'B'
                            continue

                        # 检查是否有白棋落入
                        has_white = False
                        for w_pt in white_centers_in:
                            if is_point_in_quad(w_pt, cell_quad):
                                has_white = True
                                break
                        if has_white:
                            detected_board[i] = 'W'

                    # 2. 如果检测到整个物理盘全空，则自动清空整个博弈盘，实现无感重置
                    total_pieces_in = sum(1 for cell in detected_board if cell is not None)
                    if total_pieces_in == 0:
                        if any(cell is not None for cell in board_state) or game_winner is not None:
                            board_state = [None] * 9
                            game_winner = None
                            ai_next_move = -1
                            print("[博弈] 棋盘上没有检测到任何棋子，游戏自动复位")
                    else:
                        # 部分清除：同步清除被拿走的棋子
                        for i in range(9):
                            if board_state[i] is not None and detected_board[i] is None:
                                board_state[i] = None
                                if i == ai_next_move:
                                    ai_next_move = -1
                                print(f"[博弈] 检测到格子 {i} 上的棋子被移走，清除该格子状态")

                    # 3. 博弈进行时落子判定
                    if game_winner is None:
                        new_black_idx = -1
                        for i in range(9):
                            # 人执黑棋，当检测到该格子有黑棋且未落子时，累加稳定计数（防指缝手影抖动误触发）
                            if board_state[i] is None and detected_board[i] == 'B':
                                cell_stable_counters[i] += 1
                                if cell_stable_counters[i] >= 8:  # 连续 8 帧均检测到黑棋（约 0.6 秒）视为稳定落子
                                    new_black_idx = i
                                    break
                            else:
                                cell_stable_counters[i] = 0

                        if new_black_idx != -1:
                            # 确定落子后，重置所有的消抖计数器
                            for k in range(9):
                                cell_stable_counters[k] = 0
                            board_state[new_black_idx] = 'B'
                            print(f"[博弈] 稳定落子确认：人类执黑落子于格子 {new_black_idx}")

                            # 判定人类是否获胜
                            if check_win(board_state, 'B'):
                                game_winner = 'B'
                                print("[博弈] 恭喜！人类获胜！")
                            elif all(cell is not None for cell in board_state):
                                game_winner = 'Draw'
                                print("[博弈] 棋盘已满，平局！")
                            else:
                                # 人类落子完成，轮到 AI 落子
                                ai_idx = get_ai_move(board_state)
                                if ai_idx != -1:
                                    board_state[ai_idx] = 'W'
                                    ai_next_move = ai_idx
                                    print(f"[博弈] AI (白棋) 落子于格子 {ai_idx}")

                                    # 立即将落子指令和坐标发送给下位机，下位机控制机械臂执行抓取和放置
                                    send_ai_move(uart2, ai_idx, locked_centers[ai_idx])

                                    # 判定 AI 是否获胜
                                    if check_win(board_state, 'W'):
                                        game_winner = 'W'
                                        print("[博弈] AI 获胜！")
                                    elif all(cell is not None for cell in board_state):
                                        game_winner = 'Draw'
                                        print("[博弈] 棋盘已满，平局！")
                else:
                    # 强提醒：模式 2 必须配合按键锁定棋盘使用以固定 9 个格子的物理坐标
                    img.draw_string_advanced(180, 220, 20, "PLEASE LOCK BOARD FIRST!", color=(255, 0, 0))

            # ===== 绘制棋盘线框（未锁定时绘制当前更新的，锁定时绘制保持不变的） =====
            has_target = len(locked_centers) > 0
            for idx, (corners, (cx, cy)) in enumerate(zip(locked_rects, locked_centers)):
                # 绘制四边形的四条边（红色）
                for i in range(4):
                    p0 = corners[i]
                    p1 = corners[(i + 1) % 4]
                    img.draw_line(p0[0], p0[1], p1[0], p1[1], color=(255, 0, 0), thickness=2)

                # 绘制绿色中心十字与中心点坐标标注
                img.draw_cross(cx, cy, color=(0, 255, 0), size=8, thickness=2)
                img.draw_string_advanced(cx + 6, cy - 10, 14, f"({cx},{cy})", color=(255, 255, 0))

                # 绘制绿色顶点
                if DEBUG_DRAW_DETAIL:
                    for p in corners:
                        img.draw_circle(p[0], p[1], 4, color=(0, 255, 0), fill=True)

                # 如果是模式 2，绘制 X/O 符号到对应的格子中心以供人眼直观观察
                if game_mode == 2:
                    if board_state[idx] == 'B':
                        img.draw_string_advanced(cx - 8, cy - 12, 22, "X", color=(0, 128, 255))
                    elif board_state[idx] == 'W':
                        img.draw_string_advanced(cx - 8, cy - 12, 22, "O", color=(255, 0, 0))

            # 屏幕锁定状态叠加显示
            if frozen:
                img.draw_string_advanced(5, 125, 18, "GRID STATE: LOCKED", color=(255, 128, 0))
            else:
                img.draw_string_advanced(5, 125, 18, "GRID STATE: LIVE", color=(0, 255, 0))

            fps = clock.fps()
            status_board_source = "LOCKED" if frozen else board_source
            draw_lcd_status(img, has_target, black_count_out, black_count_in, white_count_out, white_count_in,
                            fps, proc_w_show, proc_h_show, game_mode, game_winner, ai_next_move,
                            status_board_source, board_lost_frames)

            # 打印控制台节流
            if DEBUG_PRINT_INTERVAL_MS > 0:
                now = time.ticks_ms()
                if time.ticks_diff(now, last_print_ms) > DEBUG_PRINT_INTERVAL_MS:
                    last_print_ms = now

            # 如果开启了屏幕物理反转，在送显前整体旋转 180 度（通过高效的 C 语言级 replace 镜像与翻转快速实现）
            if ROTATE_SCREEN:
                img.replace(img, hmirror=True, vflip=True)

            # 显示图像
            Display.show_image(img, x=0, y=0)

            # 给系统软中断留出反应间隙
            time.sleep_ms(1)

    except KeyboardInterrupt:
        print("\n[用户中断] 检测到强行停止信号...")
    except BaseException as e:
        print(f"\n[程序异常] 触发错误: {e}")
    finally:
                # ============================================================
                # 🚨 完美顺序防御锁（专门解决卡在 LCD 去初始化的死锁问题）
                # ============================================================
                print("\n[系统守护] 正在执行底层硬件资源强制释放...")

                # 1. 优先安全关闭摄像头
                if sensor is not None:
                    try:
                        sensor.stop()
                        print("- 摄像头已安全关闭")
                    except Exception as e:
                        print(f"- 关闭摄像头失败: {e}")

                # 2. 【关键改动】必须在 MediaManager 活着的时候先去初始化 Display
                if display_inited:
                    try:
                        print("- 正在尝试解绑 LCD 屏...")
                        Display.deinit()
                        print("- LCD 显示屏已成功去初始化")
                    except Exception as e:
                        print(f"- 释放显示屏失败（已跳过防止死锁）: {e}")

                # 3. 最后再释放多媒体管理器（去掉 IMMEDIATE=True 暴力掐断，让它自然回收）
                if media_inited:
                    try:
                        MediaManager.deinit()
                        print("- 多媒体管理器已去初始化")
                    except Exception as e:
                        print(f"- 释放多媒体管理器失败: {e}")

                print("[系统守护] 硬件释放完毕，环境恢复干净，可立即重新运行！\n")

if __name__ == "__main__":
    main()
