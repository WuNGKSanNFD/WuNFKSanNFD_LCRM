# -*- coding: utf-8 -*-
"""
生成「嵌入式控制设计方案」任务关系框图。
排版要点：cmdTask 居中于三机构之上；输入源等距分布于其上方；
母线到方框留足间距；标签统一置于对应连线的上方左侧。
"""

import os
from PIL import Image, ImageDraw, ImageFont, ImageOps

HERE = os.path.dirname(os.path.abspath(__file__))

FONT_REG = r"C:\Windows\Fonts\msyh.ttc"
FONT_BOLD = r"C:\Windows\Fonts\msyhbd.ttc"

FILL = "#E9EAF6"
EDGE = "#2B2B45"
TXT = "#1F1F33"
GREY = "#5a5a72"
RULE = "#d5d5e2"
BG = "#ffffff"


def font(size, bold=False):
    try:
        return ImageFont.truetype(FONT_BOLD if bold else FONT_REG, size, index=0)
    except Exception:
        return ImageFont.truetype(r"C:\Windows\Fonts\simhei.ttf", size)


def tw(d, s, f):
    b = d.textbbox((0, 0), s, font=f)
    return b[2] - b[0]


def varrow(d, x, y0, y1):
    d.line([(x, y0), (x, y1 - 9)], fill=EDGE, width=2)
    d.polygon([(x - 5, y1 - 9), (x + 5, y1 - 9), (x, y1)], fill=EDGE)


def harrow(d, y, x0, x1):
    sgn = 1 if x1 > x0 else -1
    d.line([(x0, y), (x1 - 9 * sgn, y)], fill=EDGE, width=2)
    d.polygon([(x1 - 9 * sgn, y - 5), (x1 - 9 * sgn, y + 5), (x1, y)], fill=EDGE)


def line(d, x0, y0, x1, y1):
    d.line([(x0, y0), (x1, y1)], fill=EDGE, width=2)


def box(d, cx, cy, w, h, title, sub, duty, f_t, f_s):
    x0, y0 = cx - w / 2, cy - h / 2
    d.rectangle([x0, y0, x0 + w, y0 + h], fill=FILL, outline=EDGE, width=2)
    for i, (t, f, c) in enumerate([(title, f_t, TXT), (sub, f_s, GREY), (duty, f_s, GREY)]):
        if not t:
            continue
        y = y0 + 10 + i * 22
        d.text((cx - tw(d, t, f) / 2, y), t, font=f, fill=c)


def build(path):
    W, H = 1100, 800
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)

    f_title = font(24, True)
    f_note = font(13)
    f_t = font(15, True)
    f_s = font(11)
    f_lbl = font(11)
    f_ft = font(12)

    # ---------- 纵向基准
    Y0, Y1, Y2, Y3 = 120, 290, 460, 630
    W_TK, H_TK = 190, 82
    TOP0, BOT0 = Y0 - H_TK / 2, Y0 + H_TK / 2
    TOP1, BOT1 = Y1 - H_TK / 2, Y1 + H_TK / 2
    TOP2, BOT2 = Y2 - H_TK / 2, Y2 + H_TK / 2
    TOP3 = Y3 - H_TK / 2

    # ---------- 横向基准：三机构等距，cmd 居中
    GIM_X, CMD_X, SHO_X = 430, 670, 910
    CHA_X = CMD_X
    IN_XS = (GIM_X, CMD_X, SHO_X)      # 三个输入源
    IMU_X = 170
    MOT_X = CMD_X
    CAN_X = 950

    BUS_IN = 190
    BUS_RET = 224
    BUS_FAN = 380
    BUS_JOIN = 545

    d.text((75, 24), "嵌入式控制设计方案", font=f_title, fill=TXT)
    d.text((75, 56), "任务之间的关系：箭头表示数据流向，标注为传递的内容",
           font=f_note, fill=GREY)

    # ---------- 输入源
    box(d, IN_XS[0], Y0, W_TK, H_TK, "遥控 DR16", "USART3 · 100 ms", "摇杆 / 拨杆 / 拨轮", f_t, f_s)
    box(d, IN_XS[1], Y0, W_TK, H_TK, "视觉上位机", "USB CDC · 1 s", "自瞄指令 cmd_id + 5 float", f_t, f_s)
    box(d, IN_XS[2], Y0, W_TK, H_TK, "deamonTask", "100 Hz", "模块掉线检测", f_t, f_s)

    # ---------- 输入 → cmd
    for sx in IN_XS:
        line(d, sx, BOT0, sx, BUS_IN)
    line(d, IN_XS[0], BUS_IN, IN_XS[2], BUS_IN)
    varrow(d, CMD_X, BUS_IN, TOP1)
    d.text((IN_XS[0] + 10, BUS_IN - 18), "遥控 / 视觉指令", font=f_lbl, fill=GREY)
    d.text((IN_XS[2] - 88, BUS_IN - 18), "掉线 → 全车失能", font=f_lbl, fill=GREY)

    # ---------- 任务框
    box(d, CMD_X, Y1, W_TK, H_TK, "cmdTask", "250 Hz", "输入解析·模式决策·限幅", f_t, f_s)
    box(d, IMU_X, Y2, W_TK, H_TK, "imuTask", "1 kHz", "位姿解算·恒温·校准", f_t, f_s)
    box(d, GIM_X, Y2, W_TK, H_TK, "gimbalTask", "500 Hz", "三轴目标·外部反馈", f_t, f_s)
    box(d, CHA_X, Y2, W_TK, H_TK, "chassisTask", "200 Hz", "逆解·四轮目标", f_t, f_s)
    box(d, SHO_X, Y2, W_TK, H_TK, "shootTask", "200 Hz", "摩擦轮·拨盘", f_t, f_s)
    box(d, MOT_X, Y3, W_TK, H_TK, "motorTask", "500 Hz", "双环计算·CAN 发帧", f_t, f_s)
    box(d, CAN_X, Y3, 150, 82, "CAN 执行器", "电机 · 电调", None, f_t, f_s)

    # ---------- cmd → 三机构
    line(d, CMD_X, BOT1, CMD_X, BUS_FAN)
    line(d, GIM_X, BUS_FAN, SHO_X, BUS_FAN)
    for cx in (GIM_X, CMD_X, SHO_X):
        varrow(d, cx, BUS_FAN, TOP2)
    d.text((CMD_X + 12, BUS_FAN - 18), "三份指令：云台角 / 底盘速度 / 射频",
           font=f_lbl, fill=GREY)

    # ---------- 三机构 → motor
    for cx in (GIM_X, CMD_X, SHO_X):
        line(d, cx, BOT2, cx, BUS_JOIN)
    line(d, GIM_X, BUS_JOIN, SHO_X, BUS_JOIN)
    varrow(d, MOT_X, BUS_JOIN, TOP3)
    d.text((CMD_X + 12, BUS_JOIN - 18), "目标值：角度 rad / 转速 rpm",
           font=f_lbl, fill=GREY)

    # ---------- motor → CAN
    harrow(d, Y3, MOT_X + W_TK / 2, CAN_X - 75)
    d.text((MOT_X + W_TK / 2 + 10, Y3 - 22), "CAN 控制帧", font=f_lbl, fill=GREY)

    # ---------- imu → gimbal
    harrow(d, Y2, IMU_X + W_TK / 2, GIM_X - W_TK / 2)
    d.text((IMU_X + W_TK / 2 + 8, Y2 - 24), "位置反馈", font=f_lbl, fill=GREY)

    # ---------- imu → cmd（姿态回传）：上行一次后直接进入 cmd 左边
    line(d, IMU_X, TOP2, IMU_X, Y1)
    harrow(d, Y1, IMU_X, CMD_X - W_TK / 2)
    d.text((300, Y1 - 19), "姿态回传", font=f_lbl, fill=GREY)

    # ---------- 分隔线与说明
    d.line([(75, 706), (CAN_X + 75, 706)], fill=RULE, width=1)
    for i, s in enumerate([
        "旁路观测：serialTask（200 Hz）只读取各模块快照做调试输出，不参与控制，图中未连线。",
        "上电校准：在 imuTask 内推进 —— 加热至 40 ℃ → 稳定 500 拍 → 采样 1000 拍 → 校准完成。",
        "守护：deamonTask 超时（遥控 100 ms / 视觉 1 s）即通知 cmdTask 全车失能，收到下一有效帧自动恢复。",
    ]):
        d.text((75, 722 + i * 22), s, font=f_ft, fill=GREY)

    inv = ImageOps.invert(img.convert("L"))
    bb = inv.getbbox()
    if bb:
        img = img.crop((max(0, bb[0] - 34), max(0, bb[1] - 34),
                        min(W, bb[2] + 34), min(H, bb[3] + 34)))

    img.save(path, "PNG")
    print("saved", path, img.size)


if __name__ == "__main__":
    build(os.path.join(HERE, "嵌入式控制设计方案.png"))
