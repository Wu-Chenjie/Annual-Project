#!/usr/bin/env python3
"""Record native Gazebo + live RViz windows, then encode an explicitly sped-up demo."""
import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path


def command(args, **kwargs):
    return subprocess.run(args, check=True, **kwargs)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--timeout', type=float, default=3600.)
    parser.add_argument('--speed', type=float, default=6.)
    parser.add_argument('--map')
    args = parser.parse_args(); out = Path(args.output_dir).resolve()
    if out.exists():
        raise FileExistsError(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, DISPLAY=os.environ.get('DISPLAY', ':99'), LIBGL_ALWAYS_SOFTWARE='1', QT_X11_NO_MITSHM='1')
    servers = []
    display = env['DISPLAY']; number = display.split(':')[-1].split('.')[0]
    if not Path(f'/tmp/.X11-unix/X{number}').exists():
        servers.append(subprocess.Popen(['Xvfb', display, '-screen', '0', '1920x1080x24', '-ac'], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        for _ in range(50):
            if Path(f'/tmp/.X11-unix/X{number}').exists():
                break
            time.sleep(.1)
        servers.append(subprocess.Popen(['openbox'], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
    runner = Path(__file__).with_name('decentralized_smoke.py')
    cmd = ['python3', str(runner), '--output-dir', str(out), '--timeout', str(args.timeout), '--gui', '--record']
    if args.map:
        cmd.extend(['--map', args.map])
    start = time.time(); log_file = out.parent/(out.name+'-runner.log')
    with log_file.open('w') as log:
        proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=log, start_new_session=True)
        try:
            arranged = set(); deadline = time.monotonic()+90
            while len(arranged) < 2 and time.monotonic() < deadline and proc.poll() is None:
                for kind, selector, geometry in [('gazebo', ['--name', '^Gazebo Sim$'], [0, 40, 800, 1000]),
                                                  ('rviz', ['--class', 'rviz'], [800, 40, 1120, 1000])]:
                    if kind in arranged:
                        continue
                    result = subprocess.run(['xdotool', 'search', '--onlyvisible', *selector], env=env, capture_output=True, text=True)
                    if result.returncode == 0 and result.stdout.strip():
                        wid = result.stdout.splitlines()[-1]
                        command(['xdotool', 'windowmove', wid, *map(str, geometry[:2]), 'windowsize', wid, *map(str, geometry[2:])], env=env)
                        arranged.add(kind)
                time.sleep(.5)
            if len(arranged) != 2:
                raise RuntimeError('Gazebo and RViz windows did not both become visible')
            # Persist provenance before the potentially long mission.
            (out/'recording.json').write_text(json.dumps(dict(source='live X11 capture of native Gazebo and RViz',
                wall_start_unix=start, arranged_wall_unix=time.time(), screen=[1920, 1080], capture_fps=10,
                playback_speed=args.speed, speed_basis='wall clock recording; simulation clock remains visible in RViz',
                command=cmd), indent=2))
            if proc.wait() != 0:
                raise RuntimeError(f'Gazebo acceptance failed; retain recording and inspect {log_file}')
        finally:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
            for server in reversed(servers):
                server.terminate()
    raw = out/'gazebo-rviz-raw.mp4'; final = out/'decentralized-exploration.mp4'
    title = f'ACTUAL GAZEBO FLIGHT + LIVE ROS 2 PLANNING  |  {args.speed:g}x recording speed'
    filters = (f'setpts=PTS/{args.speed},drawbox=x=0:y=0:w=iw:h=38:color=0x121923:t=fill,'
               f"drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf:text='{title}':x=24:y=9:fontsize=22:fontcolor=white")
    command(['ffmpeg', '-y', '-loglevel', 'warning', '-i', str(raw), '-vf', filters, '-r', '30', '-c:v', 'libx264',
             '-preset', 'veryfast', '-crf', '21', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(final)])
    command(['ffmpeg', '-y', '-loglevel', 'error', '-sseof', '-4', '-i', str(final), '-frames:v', '1', str(out/'demo-preview.png')])
    probe = subprocess.check_output(['ffprobe', '-v', 'quiet', '-show_format', '-show_streams', '-of', 'json', str(final)], text=True)
    (out/'video-probe.json').write_text(probe)
    print(final)

if __name__ == '__main__':
    main()
