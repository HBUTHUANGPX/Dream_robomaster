"""Render a solver-generated robot solve and export executable action tables.

MUJOCO_GL=egl .venv-cube/bin/python record_dual_gripper_cube.py
"""
import argparse
import csv
import json
import os
from pathlib import Path
import shutil
import subprocess

os.environ.setdefault('MUJOCO_GL', 'egl')

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from cube_solver import facelets_from_cube, solve_facelets, SOLVED
from dual_gripper_cube import DualGripperCube
from gripper_plan import compile_moves, optimize_plan
from rubiks_cube import ROOT, inverse_moves

SCRAMBLE = "R U2 F' D L2 B R2 F U' L D2 B' U F2 R' D B2 L' U2 F".split()
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
FPS, SPEED = 30, 4


def action_description(action):
    kind, mode = action.get('kind'), action.get('mode')
    if kind == 'yaw':
        return {'whole': '整颗魔方换向', 'face': '单层转动', 'empty': '张开后原地回位'}[mode]
    if kind == 'jaw':
        return '张开夹爪' if action['target'] == 0 else '夹紧 / 等待接触'
    return {'release': '结束夹持状态', 'grasp': '确认双侧接触',
            'layer_unlock': '解锁待转层', 'layer_lock': '转面完成，锁定',
            'checkpoint': '核对小块位置与朝向'}.get(kind, '准备执行求解步骤')


class Recorder:
    def __init__(self, cube, solution, output, raw_count, playback=SPEED):
        self.cube, self.solution, self.output = cube, solution, output
        self.raw_count = raw_count
        self.playback = playback
        self.failure = None
        self.started = float(cube.data.time)
        self.frames = 0
        self.next_frame = self.started
        self.finished = False
        self.fonts = {n: ImageFont.truetype(FONT, n) for n in (16, 18, 20, 24, 28)}
        self.renderer = mujoco.Renderer(cube.model, height=720, width=950)
        self.closeup = mujoco.Renderer(cube.model, height=230, width=290)
        self.camera = mujoco.MjvCamera()
        self.camera.lookat[:] = [.025, -.025, .195]
        self.camera.distance = .69
        self.camera.azimuth = 135
        self.camera.elevation = -30
        self.detail_camera = mujoco.MjvCamera()
        self.detail_camera.lookat[:] = [0, 0, .22]
        self.detail_camera.distance = .155
        self.detail_camera.azimuth = 135
        self.detail_camera.elevation = -30
        self.process = subprocess.Popen([
            shutil.which('ffmpeg'), '-y', '-loglevel', 'error', '-f', 'rawvideo',
            '-pix_fmt', 'rgb24', '-s', '1280x720', '-r', str(FPS), '-i', '-',
            '-an', '-c:v', 'libx264', '-preset', 'fast', '-crf', '19',
            '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output/'demo.mp4'),
        ], stdin=subprocess.PIPE)
        self.cross_hand_contacts = 0
        self.max_cross_hand_penetration = 0.

    def frame(self):
        cube = self.cube
        self.renderer.update_scene(cube.data, camera=self.camera)
        self.closeup.update_scene(cube.data, camera=self.detail_camera)
        frame = Image.new('RGB', (1280, 720), '#101725')
        frame.paste(Image.fromarray(self.renderer.render()), (330, 0))
        frame.paste(Image.fromarray(self.closeup.render()), (20, 330))
        draw = ImageDraw.Draw(frame)

        def text(x, y, value, size=18, color='#dce6f3'):
            draw.text((x, y), value, font=self.fonts[size], fill=color)

        draw.rectangle((0, 0, 1280, 84), fill='#101725')
        text(28, 19, f'双夹爪魔方还原 / 腕部 {cube.wrist_speed:g}× · 开合 {cube.jaw_speed:g}×', 28)
        text(1060, 28, f'{self.playback:g}×  /  30 fps', 20, '#9eb2c9')
        draw.line((25, 84, 1255, 84), fill='#2d3b50')
        text(25, 108, '当前动作', 16, '#9eb2c9')
        action = cube.current_action or {}
        description = '安全检查停止执行' if self.failure else '魔方已还原' if self.finished else action_description(action)
        text(24, 143, description, 24, '#72e4c3' if self.finished else '#eaf1fa')
        hand = action.get('hand', '')
        target = action.get('target', 0)
        detail = ''
        if action.get('kind') == 'yaw':
            detail = f'夹爪 {hand}  →  {np.degrees(target):+.0f}°'
        elif action.get('kind') == 'jaw':
            detail = f'夹爪 {hand}  →  {target:.0f} / 255'
        elif hand:
            detail = f'夹爪 {hand}'
        text(25, 193, detail, 20, '#9eb2c9')
        if action.get('kind') == 'yaw' and action.get('mode') in ('empty', 'whole'):
            open_hand = hand if action['mode'] == 'empty' else ('B' if hand == 'A' else 'A')
            text(25, 220, f'{open_hand} 开口 {cube.open_gap(open_hand)*1000:.1f} mm ≥ 89.1 mm',
                 16, '#72e4c3')
        for i, h in enumerate(['A', 'B']):
            color = '#76b8f5' if h == 'A' else '#f2ab6a'
            mode = cube.grasped[h]
            text(25, 243+i*28, f'{h}：' + ('支撑整块' if mode == 'core' else
                 '夹持转动层' if mode == 'face' else '已释放'), 18, color)
        text(25, 303, '魔方局部视图', 16, '#9eb2c9')
        draw.rectangle((19, 329, 311, 561), outline='#36465a')
        completed = len(cube.history)
        text(25, 583, f'求解转面：{completed} / {len(self.solution)}', 20)
        index, total = action.get('index', -1)+1, action.get('total', 0)
        text(25, 620, f'动作 {index}/{total} · 原始 {self.raw_count}', 18, '#9eb2c9')
        text(385, 108, 'B：−Y 旋转轴', 18, '#f2ab6a')
        text(1060, 108, 'A：+X 旋转轴', 18, '#76b8f5')
        if self.finished:
            draw.rounded_rectangle((565, 155, 1035, 222), radius=12, fill='#15342e')
            text(596, 169, '54 个色块与实际位姿：验证通过', 20, '#72e4c3')
        for i, move in enumerate(self.solution):
            x, y = 357+(i % 10)*90, 579+(i//10)*39
            done = i < completed
            active = i == completed and not self.finished
            fill = '#72e4c3' if active else '#19332e' if done else '#1b2637'
            draw.rounded_rectangle((x, y, x+79, y+32), radius=5, fill=fill)
            text(x+25, y+1, move, 18, '#101725' if active else '#dce6f3')
        draw.rectangle((0, 674, 1280, 720), fill='#101725')
        text(25, 685, '固定安装，无进退轴 · 张开到安全间隙后原地回位 · 转换后优化 · 执行器驱动 · 纯接触摩擦夹持',
             16, '#9eb2c9')
        return frame

    def capture(self, cube):
        if cube.data.time+1e-8 < self.next_frame:
            return
        # Record any actual gripper-gripper contact for the verification report.
        for contact in cube.data.contact:
            names = [cube.model.body(cube.model.geom_bodyid[g]).name for g in contact.geom]
            if ((names[0].startswith('A_') and names[1].startswith('B_')) or
                    (names[1].startswith('A_') and names[0].startswith('B_'))):
                self.cross_hand_contacts += 1
                self.max_cross_hand_penetration = max(self.max_cross_hand_penetration, -float(contact.dist))
        self.process.stdin.write(self.frame().tobytes())
        self.frames += 1
        self.next_frame = self.started+self.frames*self.playback/FPS

    def close(self):
        self.process.stdin.close()
        code = self.process.wait(timeout=60)
        self.renderer.close()
        self.closeup.close()
        if code:
            raise RuntimeError(f'FFmpeg exited with {code}')


def export_plan(path, state, solution, plan, raw_plan):
    metadata = {
        'solver': 'https://github.com/muodov/kociemba', 'solver_version': '1.2.1',
        'initial_facelets': state, 'solution': solution,
        'axes': {'A': '+X outward', 'B': '-Y outward'},
        'precondition': 'both gripping the core; wrists zero; mounts fixed; fixture off',
        'units': {'yaw': 'radians, right-hand positive', 'jaw': 'Robotiq 0..255'},
        'simulation_events': ['grasp', 'release', 'layer_unlock', 'layer_lock', 'checkpoint'],
        'note': 'On hardware, grasp/release mean force-confirmed state transitions; layer locks are cube-model events.',
        'actions': [a.as_dict() for a in plan],
    }
    (path/'plan.json').write_text(json.dumps(metadata, indent=2))
    (path/'plan_unoptimized.json').write_text(json.dumps({
        **metadata, 'actions': [a.as_dict() for a in raw_plan]}, indent=2))
    stats = {
        'raw_actions': len(raw_plan), 'optimized_actions': len(plan),
        'removed_actions': len(raw_plan)-len(plan),
        'raw_jaw_commands': sum(a.kind == 'jaw' for a in raw_plan),
        'optimized_jaw_commands': sum(a.kind == 'jaw' for a in plan),
        'wrist_rotations_unchanged': [(a.hand, a.target, a.mode) for a in raw_plan if a.kind == 'yaw'] ==
                                    [(a.hand, a.target, a.mode) for a in plan if a.kind == 'yaw'],
        'method': 'post-compilation removal of redundant close/grasp/release/open sandwiches and no-op jaw commands',
    }
    (path/'optimization.json').write_text(json.dumps(stats, indent=2))
    with (path/'steps.csv').open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['step', 'kind', 'gripper', 'target', 'unit', 'mode', 'cube_move'])
        for i, action in enumerate(plan, 1):
            unit = {'yaw': 'rad', 'jaw': '0..255'}.get(action.kind, 'event')
            writer.writerow([i, action.kind, action.hand, action.target, unit, action.mode, action.move])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'output/dual_gripper_friction')
    parser.add_argument('--speed', type=float, default=1., help='Physical motion reference multiplier')
    parser.add_argument('--wrist-speed', type=float, help='Independent wrist speed multiplier')
    parser.add_argument('--jaw-speed', type=float, help='Independent tuned finger speed multiplier')
    parser.add_argument('--playback', type=float, default=SPEED, help='Video playback multiplier; use 1 for real time')
    args = parser.parse_args()
    if not np.isfinite(args.playback) or args.playback <= 0:
        parser.error('--playback must be finite and positive')
    if not shutil.which('ffmpeg'):
        parser.error('FFmpeg is required')
    args.output.mkdir(parents=True, exist_ok=True)
    cube = DualGripperCube(speed=args.speed, wrist_speed=args.wrist_speed, jaw_speed=args.jaw_speed)
    for move in SCRAMBLE:
        cube.turn(move)
    state = facelets_from_cube(cube)
    cube.history.clear()
    solution = solve_facelets(state)
    raw_plan = compile_moves(solution)
    plan = optimize_plan(raw_plan)
    export_plan(args.output, state, solution, plan, raw_plan)
    (args.output/'scene.xml').write_text(cube.xml)
    (args.output/'controller_profile.json').write_text(json.dumps({
        'wrist_speed': cube.wrist_speed, 'jaw_speed': cube.jaw_speed,
        'fast_parallel_fingers': cube.fast_jaws, 'video_playback': args.playback,
        'physics_step_s': float(cube.model.opt.timestep),
        'finger_kp': 100*cube.jaw_speed**2 if cube.fast_jaws else 100,
        'finger_kv': 10*cube.jaw_speed if cube.fast_jaws else 10,
        'finger_free_effort_limit_nm': 40 if cube.fast_jaws else 5,
        'finger_contact_effort_limit_nm': 5,
        'finger_reflected_inertia_scale': .01 if cube.fast_jaws else 1,
        'grasp': 'friction only; no external grasp constraints',
        'jaw_commands': '0=open, 115=close request; controller generates actual servo controls',
        'note': 'Tuned simulation drive and parallel guides; not stock Robotiq performance' if cube.fast_jaws else 'Original drive'
    }, indent=2))
    print('Solver solution:', ' '.join(solution), flush=True)
    print('Actions:', len(raw_plan), '->', len(plan), flush=True)
    cube.initialize_grasps()
    if facelets_from_cube(cube) != state:
        raise RuntimeError('Loading changed the cube state')
    cube.current_action = {'kind': 'ready', 'index': -1, 'total': len(plan)}
    recorder = Recorder(cube, solution, args.output, len(raw_plan), playback=args.playback)
    try:
        recorder.capture(cube)
        cube.advance(args.playback, recorder.capture)
        recorder.frame().save(args.output/'01_scrambled.png')
        previous = -1

        def capture(current):
            nonlocal previous
            index = current.current_action.get('index', 0)
            if index//25 != previous:
                previous = index//25
                print(f'Action {index+1}/{len(plan)}; solved moves {len(current.history)}', flush=True)
            recorder.capture(current)

        execution_started = float(cube.data.time)
        cube.execute(plan, capture)
        execution_time = float(cube.data.time)-execution_started
        cube.advance(.5, recorder.capture)
        report = cube.robot_report()
        if report['facelets'] != SOLVED or not cube.is_solved():
            raise RuntimeError('Final physical cube is not solved')
        recorder.finished = True
        cube.advance(3*args.playback, recorder.capture)
        recorder.frame().save(args.output/'02_solved.png')
        report = cube.robot_report()
        report.update({'execution_time_s': execution_time, 'scramble': SCRAMBLE, 'input_facelets': state, 'solution': solution,
                       'raw_primitive_actions': len(raw_plan), 'optimized_primitive_actions': len(plan),
                       'different_from_inverse_scramble': solution != inverse_moves(SCRAMBLE),
                       'video_frames': recorder.frames, 'video_fps': FPS,
                       'video_speed': args.playback, 'video_duration_s': recorder.frames/FPS,
                       'sampled_cross_gripper_contacts': recorder.cross_hand_contacts,
                       'max_sampled_cross_gripper_penetration_m': recorder.max_cross_hand_penetration})
        (args.output/'verification.json').write_text(json.dumps(report, indent=2))
        (args.output/'grasp_checks.json').write_text(json.dumps(cube.grasp_checks, indent=2))
    except RuntimeError as error:
        recorder.failure = str(error)
        failure = {'success': False, 'error': str(error), 'requested_motion_speed': args.speed, 'wrist_speed': cube.wrist_speed, 'jaw_speed': cube.jaw_speed,
                   'video_playback_speed': args.playback, 'action': cube.current_action,
                   'completed_actions': len(cube.executed), 'completed_moves': len(cube.history),
                   'motion_checks': cube.motion_checks, 'clearance': cube.clearance_report}
        (args.output/'failure.json').write_text(json.dumps(failure, indent=2))
        recorder.frame().save(args.output/'failure.png')
        for _ in range(2*FPS):
            recorder.process.stdin.write(recorder.frame().tobytes())
            recorder.frames += 1
        raise
    finally:
        recorder.close()
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
