"""Record actual MuJoCo frames: solved -> scramble -> inverse recovery.

MUJOCO_GL=egl uv run python record_rubiks_cube.py
"""
from pathlib import Path
import argparse
import json
import os
import shutil
import subprocess

os.environ.setdefault('MUJOCO_GL', 'egl')

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from rubiks_cube import RubiksCube, SCRAMBLE, ROOT, inverse_moves, parse_move

FPS = 30
WIDTH, HEIGHT = 1280, 720
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
FACE_NAMES = {'R': '右面', 'L': '左面', 'U': '上面', 'D': '下 面', 'F': '前面', 'B': '后面'}


class Recording:
    def __init__(self, cube, output):
        self.cube = cube
        self.output = output
        self.frames = 0
        self.next_frame = 0.
        self.phase = '初始状态'
        self.detail = '每个色块都固定在独立刚体上'
        self.move = ''
        self.sequence = SCRAMBLE
        self.index = -1
        self.accent = '#6ee7c7'
        self.camera = mujoco.MjvCamera()
        self.camera.lookat[:] = [0, 0, .077]
        self.camera.distance = .24
        self.camera.elevation = -25
        self.camera.azimuth = 135
        self.orbit_start = 0.
        self.orbit_speed = 0.
        self.base_azimuth = 135.
        self.fonts = {size: ImageFont.truetype(FONT, size) for size in [16, 18, 20, 24, 36, 62]}
        self.renderer = mujoco.Renderer(cube.model, height=HEIGHT, width=900)
        self.process = subprocess.Popen([
            shutil.which('ffmpeg'), '-y', '-loglevel', 'error', '-f', 'rawvideo',
            '-pix_fmt', 'rgb24', '-s', f'{WIDTH}x{HEIGHT}', '-r', str(FPS),
            '-i', '-', '-an', '-c:v', 'libx264', '-preset', 'fast', '-crf', '19',
            '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output/'demo.mp4'),
        ], stdin=subprocess.PIPE)

    def image(self):
        cube = self.cube
        self.camera.azimuth = self.base_azimuth + self.orbit_speed*(cube.data.time-self.orbit_start)
        self.renderer.update_scene(cube.data, camera=self.camera)
        image = Image.new('RGB', (WIDTH, HEIGHT), '#101725')
        image.paste(Image.fromarray(self.renderer.render()), (380, 0))
        draw = ImageDraw.Draw(image)

        def text(x, y, value, size=20, color='#e9eef7'):
            draw.text((x, y), value, font=self.fonts[size], fill=color)

        draw.rectangle((0, 0, WIDTH, 83), fill='#101725')
        draw.line((28, 83, WIDTH-28, 83), fill='#2c3748')
        draw.ellipse((30, 32, 41, 43), fill=self.accent)
        text(54, 21, 'MUJOCO  /  可打乱、可还原的魔方', 24)
        text(1065, 29, f'{cube.data.time:05.1f} s  /  30 fps', 18, '#9bacc1')
        text(30, 114, '3 × 3 × 3  /  RIGID BODY SIMULATION', 16, '#95a5bc')
        text(28, 151, self.phase, 36)
        text(30, 216, self.detail, 18, '#aebdd0')

        draw.rounded_rectangle((28, 270, 350, 430), radius=14, fill='#1b2637')
        text(46, 284, '当前转面' if self.move else '状态', 18, '#aebdd0')
        text(44, 313, self.move if self.move else 'READY' if self.phase == '初始状态'
             else 'SOLVED' if self.phase == '还原完成' else 'MIXED',
             62 if self.move else 36, self.accent)
        if self.move:
            face, count = parse_move(self.move)
            text(135, 343, f'{FACE_NAMES[face]} · ' + ('180°' if count == 2 else
                 '逆时针 90°' if count == -1 else '顺时针 90°'), 18)
            draw.rounded_rectangle((46, 410, 332, 414), radius=2, fill='#354154')
            draw.rounded_rectangle((46, 410, 46+max(1, 286*cube.progress), 414),
                                   radius=2, fill=self.accent)
        else:
            text(46, 386, '六面复原' if self.phase in ('初始状态', '还原完成')
                 else '位置与朝向均已改变', 18, '#aebdd0')

        text(30, 452, '逆序还原序列' if self.phase in ('逆序还原', '还原完成')
             else '打乱序列', 18, '#aebdd0')
        for i, move in enumerate(self.sequence):
            x, y = 30 + (i % 5)*64, 490 + (i//5)*47
            active = i == self.index
            draw.rounded_rectangle((x, y, x+53, y+36), radius=6,
                                   fill=self.accent if active else '#1b2637')
            text(x+12, y+1, move, 20, '#101725' if active else '#d7e2ef')
        text(30, 598, '6 个 hinge · 20 个独立角块 / 棱块', 16, '#aebdd0')
        text(30, 626, '电机驱动 + 可切换 weld 约束', 16, '#aebdd0')

        # Short result panel is based on live physical body poses.
        if self.phase == '还原完成':
            position, angle = cube.solved_pose_error()
            draw.rounded_rectangle((720, 555, 1225, 641), radius=12, fill='#142c2c')
            text(744, 565, '✓ 位置与朝向验证通过', 24, self.accent)
            text(744, 603, f'位置误差 {position*1000:.4f} mm   /   朝向误差 {np.degrees(angle):.4f}°',
                 16, '#b9d9d0')
        draw.rectangle((0, 674, WIDTH, HEIGHT), fill='#101725')
        text(30, 684, '固定核心 · 程序控制层间锁定 · 还原采用已知打乱序列的逆序操作', 16, '#95a5bc')
        return image

    def capture(self, cube):
        if cube.data.time + 1e-8 < self.next_frame:
            return
        image = self.image()
        self.process.stdin.write(image.tobytes())
        self.frames += 1
        self.next_frame = self.frames/FPS

    def hold(self, seconds, orbit_speed=0):
        self.base_azimuth = self.camera.azimuth
        self.orbit_start = self.cube.data.time
        self.orbit_speed = orbit_speed
        self.cube.advance(seconds, self.capture)
        self.base_azimuth = self.camera.azimuth
        self.orbit_speed = 0

    def close(self):
        self.process.stdin.close()
        code = self.process.wait(timeout=60)
        self.renderer.close()
        if code:
            raise RuntimeError(f'FFmpeg exited with status {code}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'output/rubiks_cube')
    args = parser.parse_args()
    if not shutil.which('ffmpeg'):
        parser.error('ffmpeg is required to encode the recording')
    if not Path(FONT).exists():
        parser.error(f'Chinese subtitle font not found: {FONT}')
    args.output.mkdir(parents=True, exist_ok=True)
    cube = RubiksCube()
    model_path = ROOT/'assets/rubiks_cube.xml'
    model_path.parent.mkdir(parents=True, exist_ok=True)
    model_path.write_text(cube.xml)
    recording = Recording(cube, args.output)
    checks = []
    try:
        recording.capture(cube)
        recording.hold(2, orbit_speed=5)
        recording.image().save(args.output/'01_solved.png')
        recording.phase = '连续打乱'
        recording.detail = '跨轴转面，小块重新分配到各层'
        recording.accent = '#f6ba73'
        for i, move in enumerate(SCRAMBLE):
            recording.index, recording.move = i, move
            cube.turn(move, recording.capture)
            checks.append({'move': move, 'pose_error': cube.pose_error()})
            print(f'Scramble {i+1}/{len(SCRAMBLE)}: {move}', flush=True)
        if cube.is_solved() or cube.solved_pose_error()[0] < .01:
            raise RuntimeError('The scramble did not change the physical cube')
        recording.phase, recording.move, recording.index = '已打乱', '', -1
        recording.detail = '接下来执行相反顺序、相反方向'
        recording.hold(2.5, orbit_speed=30)
        recording.image().save(args.output/'02_scrambled.png')
        scramble_report = cube.report()
        recording.phase = '逆序还原'
        recording.detail = '保留同一仿真状态，逐步逆向转动'
        recording.accent = '#6ee7c7'
        recording.sequence = inverse_moves(SCRAMBLE)
        for i, move in enumerate(recording.sequence):
            recording.index, recording.move = i, move
            cube.turn(move, recording.capture)
            checks.append({'move': move, 'pose_error': cube.pose_error()})
            print(f'Restore {i+1}/{len(SCRAMBLE)}: {move}', flush=True)
        position, angle = cube.solved_pose_error()
        if not cube.is_solved() or position > 2e-5 or angle > .002 or cube.data.warning.number.any():
            raise RuntimeError(f'Restoration failed: {cube.report()}')
        recording.phase, recording.move, recording.index = '还原完成', '', -1
        recording.detail = '已检查全部 20 块的位置和朝向'
        recording.hold(4, orbit_speed=25)
        recording.image().save(args.output/'03_restored.png')
        report = cube.report()
        report.update({'scramble': SCRAMBLE, 'scrambled_state': scramble_report,
                       'per_move_checks': checks, 'frames': recording.frames, 'fps': FPS,
                       'video_duration_s': recording.frames/FPS,
                       'model': str(model_path), 'video': str(args.output/'demo.mp4'),
                       'method': 'MuJoCo motor-driven hinges + switched welds; no qpos animation',
                       'limitations': 'Fixed core, no internal cubie collisions, inverse known scramble; not a general solver'})
        (args.output/'verification.json').write_text(json.dumps(report, indent=2))
    finally:
        recording.close()
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
