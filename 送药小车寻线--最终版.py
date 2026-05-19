import sensor, image, time, pyb

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_vflip(True)              # 镜头垂直翻转

# 串口 OpenMV H7 专用 UART3
uart = pyb.UART(3, 115200, timeout_char=1000)

# ===================== 寻线参数 =====================
# 红色LAB阈值
RED_THRESHOLD = (0, 62, 7, 38, 1, 26)
# 黑色LAB阈值
BLACK_THRESHOLD = (0, 31, -22, -1, -46, 17)
# 梯形（低视角寻线专用：上窄下宽）
TOP_WIDTH = 25       # 顶部宽度
BOTTOM_WIDTH = 65    # 底部宽度（越靠近镜头越宽）
CENTER_X = 160       # 屏幕中心
TOP_Y = 40          # 梯形顶部开始位置
BOTTOM_Y = 240      # 梯形底部位置

# 目标中心点
TARGET_X = 160

# ===================== 黑色检测矩形框参数 =====================
BLACK_RECT_X = 80     # 矩形框左上角X坐标
BLACK_RECT_Y = 189     # 矩形框左上角Y坐标
BLACK_RECT_W = 170    # 矩形框宽度
BLACK_RECT_H = 50     # 矩形框高度
# 黑色色块检测阈值（过滤小噪点）
BLACK_AREA_THRESHOLD = 50
BLACK_PIXELS_THRESHOLD = 50

# ===================== 新增：转向检测框参数（左上+右上） =====================
# 左上角红色转向检测框
LEFT_TURN_RECT_X = 10       # 左上角X
LEFT_TURN_RECT_Y = 10       # 左上角Y
LEFT_TURN_RECT_W = 60       # 宽度
LEFT_TURN_RECT_H = 60       # 高度

# 右上角红色转向检测框
RIGHT_TURN_RECT_X = 250      # 右上角X（320-60-10）
RIGHT_TURN_RECT_Y = 10       # 右上角Y
RIGHT_TURN_RECT_W = 60       # 宽度
RIGHT_TURN_RECT_H = 60       # 高度

# 转向红色色块阈值（改为与寻线阈值一致）
TURN_RED_AREA = 100          # 原寻线area_threshold=100
TURN_RED_PIXELS = 100        # 原寻线pixels_threshold=100

# ===================== PID 控制参数（核心！） =====================
# 比例系数、积分系数、微分系数
Kp = 0.5   # 比例：调大=反应快，过大抖动
Ki = 0.2  # 积分：调大=消除静差，过大过冲
Kd = 0.8  # 微分：调大=抑制抖动，过大震荡

# PID 变量
last_error = 0    # 上一次误差
integral = 0      # 积分累加值
pid_max = 100     # PID输出限幅（防止过大）
pid_min = -100

# ======================================================
# 统一冷却时间：发送任意标志后，3秒内不再发送标志
COOLDOWN_TIME = 3000  # 3秒，单位毫秒
# 记录上一次发送标志的时间（统一管理）
last_send_time = 0

clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()
    current_time = pyb.millis()
    # 统一判断：是否在冷却期内（冷却期禁止发送任何标志）
    in_cooldown = (current_time - last_send_time) < COOLDOWN_TIME

    # ================= 画梯形寻线框（绿色） =================
    img.draw_line(CENTER_X-TOP_WIDTH, TOP_Y, CENTER_X+TOP_WIDTH, TOP_Y, color=(0,255,0))
    img.draw_line(CENTER_X-BOTTOM_WIDTH, BOTTOM_Y, CENTER_X+BOTTOM_WIDTH, BOTTOM_Y, color=(0,255,0))
    img.draw_line(CENTER_X-TOP_WIDTH, TOP_Y, CENTER_X-BOTTOM_WIDTH, BOTTOM_Y, color=(0,255,0))
    img.draw_line(CENTER_X+TOP_WIDTH, TOP_Y, CENTER_X+BOTTOM_WIDTH, BOTTOM_Y, color=(0,255,0))

    # ================= 画黑色检测矩形框（蓝色） =================
    img.draw_rectangle(BLACK_RECT_X, BLACK_RECT_Y, BLACK_RECT_W, BLACK_RECT_H, color=(0,0,255))
    img.draw_string(BLACK_RECT_X, BLACK_RECT_Y-10, "Black Detect", color=(0,0,255), scale=1)

    # ================= 画转向检测框（黄色） =================
    # 左上角转向框
    img.draw_rectangle(LEFT_TURN_RECT_X, LEFT_TURN_RECT_Y, LEFT_TURN_RECT_W, LEFT_TURN_RECT_H, color=(255,255,0))
    img.draw_string(LEFT_TURN_RECT_X, LEFT_TURN_RECT_Y-10, "Turn L", color=(255,255,0), scale=1)
    # 右上角转向框
    img.draw_rectangle(RIGHT_TURN_RECT_X, RIGHT_TURN_RECT_Y, RIGHT_TURN_RECT_W, RIGHT_TURN_RECT_H, color=(255,255,0))
    img.draw_string(RIGHT_TURN_RECT_X, RIGHT_TURN_RECT_Y-10, "Turn R", color=(255,255,0), scale=1)

    # ================= 检测黑色色块 =================
    black_blobs = img.find_blobs([BLACK_THRESHOLD],
                                roi=(BLACK_RECT_X, BLACK_RECT_Y, BLACK_RECT_W, BLACK_RECT_H),
                                merge=True,
                                area_threshold=BLACK_AREA_THRESHOLD,
                                pixels_threshold=BLACK_PIXELS_THRESHOLD)

    # 检测到黑色色块 → 停车
    if black_blobs:
        max_black_b = max(black_blobs, key=lambda b: b.pixels())
        img.draw_rectangle(max_black_b.rect(), color=(255,255,0))
        img.draw_cross(max_black_b.cx(), max_black_b.cy(), size=8, color=(255,255,0))

        # 不在冷却期才发送
        if not in_cooldown:
            uart.write("stop\n")
            print("检测到黑色色块，发送stop")
            last_send_time = current_time  # 更新最后发送时间

        img.draw_string(10, 50, "Status: STOP", color=(255,0,0), scale=2)
        integral = 0  # 清空积分，防止累积
        last_error = 0

    else:
        # 未检测到黑色 → 检测左右上角红色转向
        # 检测左上角红色
        left_turn_blobs = img.find_blobs([RED_THRESHOLD],
                                       roi=(LEFT_TURN_RECT_X, LEFT_TURN_RECT_Y, LEFT_TURN_RECT_W, LEFT_TURN_RECT_H),
                                       merge=True,
                                       area_threshold=TURN_RED_AREA,
                                       pixels_threshold=TURN_RED_PIXELS)

        # 检测右上角红色
        right_turn_blobs = img.find_blobs([RED_THRESHOLD],
                                       roi=(RIGHT_TURN_RECT_X, RIGHT_TURN_RECT_Y, RIGHT_TURN_RECT_W, RIGHT_TURN_RECT_H),
                                       merge=True,
                                       area_threshold=TURN_RED_AREA,
                                       pixels_threshold=TURN_RED_PIXELS)

        # 检测到任意转向红色 + 不在冷却期 → 发送turn
        if (left_turn_blobs or right_turn_blobs) and not in_cooldown:
            # 绘制检测到的色块
            if left_turn_blobs:
                max_turn_b = max(left_turn_blobs, key=lambda b: b.pixels())
                img.draw_rectangle(max_turn_b.rect(), color=(255,0,255))
                img.draw_cross(max_turn_b.cx(), max_turn_b.cy(), size=8, color=(255,0,255))
            if right_turn_blobs:
                max_turn_b = max(right_turn_blobs, key=lambda b: b.pixels())
                img.draw_rectangle(max_turn_b.rect(), color=(255,0,255))
                img.draw_cross(max_turn_b.cx(), max_turn_b.cy(), size=8, color=(255,0,255))

            uart.write("turn\n")
            print("检测到转向红色，发送turn")
            last_send_time = current_time  # 更新最后发送时间

        # 正常循迹 + PID计算
        blobs = img.find_blobs([RED_THRESHOLD],
                               roi=(CENTER_X-BOTTOM_WIDTH, TOP_Y, BOTTOM_WIDTH*2, BOTTOM_Y-TOP_Y),
                               merge=True,
                               area_threshold=100,
                               pixels_threshold=100)

        if blobs:
            max_b = max(blobs, key=lambda b: b.pixels())
            cx = max_b.cx()
            cy = max_b.cy()
            img.draw_rectangle(max_b.rect(), color=(255,0,0))
            img.draw_cross(cx, cy, size=8, color=(0,0,255))
            error = cx - TARGET_X
        else:
            error = 0  # 无线时误差为0

        # ===================== PID 计算 =====================
        # 积分限幅，防止积分饱和
        integral += error
        if integral > 10: integral = 10
        if integral < -10: integral = -10

        # PID公式
        pid_output = Kp * error + Ki * integral + Kd * (error - last_error)

        # 输出限幅
        if pid_output > pid_max:
            pid_output = pid_max
        if pid_output < pid_min:
            pid_output = pid_min

        # 保存上一次误差
        last_error = error

        # ================= 屏幕显示 =================
        img.draw_string(10, 80, f"Err: {error}", color=(255,255,255), scale=2)
        img.draw_string(10, 105, f"PID: {int(pid_output)}", color=(0,255,255), scale=2)
        if in_cooldown:
            img.draw_string(10, 130, "Status: COOLDOWN", color=(255,0,255), scale=2)
        elif left_turn_blobs or right_turn_blobs:
            img.draw_string(10, 130, "Status: TURN", color=(255,0,255), scale=2)

        # 发送 PID 输出给 STM32
        uart.write(f"{int(pid_output)}\n")
        print(f"误差: {error}  PID输出: {int(pid_output)}")

    time.sleep_ms(10)
