#!/usr/bin/env python3
"""板端画面中文叠加。无中文字体时退回 ASCII。

优先用文泉驿 / Noto CJK / 微软雅黑等 TrueType，经 PIL 绘制；
找不到字体时把常见中文 HUD 词替换为英文短词，再用 OpenCV ``putText``。
"""
import os

import cv2
import numpy as np

FONT = cv2.FONT_HERSHEY_SIMPLEX
_CN_FONT_PATHS = (
    '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc',
    '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/truetype/arphic/uming.ttc',
    '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf',
    'C:/Windows/Fonts/msyh.ttc',
    'C:/Windows/Fonts/simhei.ttf',
)
_CN_FONT_CACHE = {}
# 长词在前，避免短词先替换把句子拆坏
_CN_ASCII = (
    ('等待 Stereonet 深度数据', 'Waiting Stereonet'),
    ('等待 Stereonet 深度图', 'Waiting Stereonet depth'),
    ('等待 Stereonet / 行人检测', 'Waiting Stereonet / person'),
    ('等待 Stereonet（GS130W / 例程8）', 'Waiting Stereonet'),
    ('单目USB普通摄像头，不用深度相机', 'USB camera'),
    ('等待 USB 摄像头', 'Waiting USB camera'),
    ('等待相机画面', 'Waiting camera'),
    ('等待 MIPI 拼接图', 'Waiting MIPI combine'),
    ('等待模拟深度', 'wait sim depth'),
    ('等待深度图', 'wait depth'),
    ('等待深度对齐', 'wait depth'),
    ('等待台架位姿', 'Waiting pose'),
    ('等待解锁', 'WAIT ARM'),
    ('等待起飞', 'WAIT TAKEOFF'),
    ('起飞中', 'TAKEOFF'),
    ('悬停，规划航点', 'HOVER PLAN'),
    ('悬停搜索', 'HOVER SEARCH'),
    ('对准 H 标', 'ALIGN H'),
    ('已对准，降落', 'ALIGNED LAND'),
    ('已完成，降落', 'DONE LAND'),
    ('完成，降落', 'DONE LAND'),
    ('画面丢失', 'NO IMAGE'),
    ('MIPI 画面中断', 'MIPI frame lost'),
    ('未发现 H 标', 'NO H MARK'),
    ('模型未加载', 'NO MODEL'),
    ('YOLO 未加载', 'YOLO not loaded'),
    ('未加载', 'NO'),
    ('推理失败', 'INFER FAIL'),
    ('无目标', 'NO TARGET'),
    ('检测', 'det'),
    ('亮度', 'luma'),
    ('画面偏暗已增强', 'boosted'),
    ('已居中', 'CENTERED'),
    ('相对H', 'vs H'),
    ('请先启动对应深度相机驱动', 'start depth camera first'),
    ('导航中', 'NAV'),
    ('避障中', 'AVOID'),
    ('阶段', 'phase'),
    ('相对高', 'rel_alt'),
    ('前方深', 'front'),
    ('轨迹点', 'trail'),
    ('高度', 'ALT'),
    ('航点', 'WP'),
    ('位移', 'MOVE'),
    ('障碍', 'OBS'),
    ('安全', 'CLEAR'),
    ('急停', 'STOP'),
    ('避障', 'AVOID'),
    ('刹前', 'SLOW'),
    ('上升', 'UP'),
    ('下降', 'DOWN'),
    ('悬停', 'HOVER'),
    ('升', 'U'),
    ('降', 'D'),
    ('前', 'FWD'),
    ('后', 'BACK'),
    ('左', 'LEFT'),
    ('右', 'RIGHT'),
)


def _cn_font(size):
    """按字号缓存并返回 PIL TrueType 字体；找不到则返回 None。"""
    key = int(size)
    if key in _CN_FONT_CACHE:
        return _CN_FONT_CACHE[key]
    try:
        from PIL import ImageFont
    except ImportError:
        _CN_FONT_CACHE[key] = None
        return None
    for path in _CN_FONT_PATHS:
        if not os.path.isfile(path):
            continue
        try:
            font = ImageFont.truetype(path, key)
            _CN_FONT_CACHE[key] = font
            return font
        except Exception:
            continue
    _CN_FONT_CACHE[key] = None
    return None


def _ascii_hud(text):
    """无中文字体时：按词表替换常见中文，再去掉剩余非 ASCII。"""
    out = text
    for cn, en in _CN_ASCII:
        out = out.replace(cn, en)
    out = ''.join(ch if ord(ch) < 128 else '' for ch in out)
    return out.strip() or '?'


def put_cn_lines(img, lines, origin=(10, 8), size=22):
    """在 BGR 图上叠加多行中文（或 ASCII 回退）。

    Args:
        img: OpenCV BGR 图，就地修改。
        lines: 每项为 ``(text, color_bgr)`` 或 ``(text, color_bgr, (x,y))``；
            未给坐标时从 ``origin`` 起按行高自动排布。
        origin: 自动排布时的左上起点。
        size: 字号（像素，传给 TrueType）。
    """
    if not lines:
        return
    font = _cn_font(size)
    x0, y0 = origin
    positioned = []
    y = y0
    for item in lines:
        if len(item) == 3:
            text, color, xy = item
        else:
            text, color = item
            xy = (x0, y)
            y += size + 8
        positioned.append((text, color, xy))
    if font is None:
        for text, color, xy in positioned:
            cv2.putText(img, _ascii_hud(text), (xy[0], xy[1] + 18),
                        FONT, 0.62, color, 2)
        return
    from PIL import Image, ImageDraw
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    pil = Image.fromarray(rgb)
    draw = ImageDraw.Draw(pil)
    for text, color, xy in positioned:
        # OpenCV BGR → PIL RGB；先画黑描边再画正文，提升可读性
        rgb_c = (int(color[2]), int(color[1]), int(color[0]))
        x, yy = xy
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            draw.text((x + dx, yy + dy), text, font=font, fill=(0, 0, 0))
        draw.text(xy, text, font=font, fill=rgb_c)
    img[:] = cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)
