#!/usr/bin/env python3
"""dojo_map.py - send your robot's map to the Robotics Dojo judges.

Runs on Ubuntu 24.04 (ROS 2 Jazzy) on the Pi or PC. Needs only Python 3; ROS is
only used when grabbing a PGM from the /map topic.

  Get it:      curl -fsSL http://JUDGE_IP:8000/static/dojo_map.py -o dojo_map.py

  Answer the judges' current request (once):
      python3 dojo_map.py --server http://JUDGE_IP:8000 --team Alpha
  Stay running and answer every request automatically:
      python3 dojo_map.py --server http://JUDGE_IP:8000 --team Alpha --watch
  Send something right now without waiting for a request:
      python3 dojo_map.py --server ... --team Alpha --push pgm
      python3 dojo_map.py --server ... --team Alpha --push screenshot
  Send a file you already have (map.pgm, or an RViz screenshot png/jpg):
      python3 dojo_map.py --server ... --team Alpha --file ~/Pictures/rviz.png

PGM    : grabs the OccupancyGrid on --topic (default /map) and writes it the same
         way nav2 map_saver does. Source ROS first:  source /opt/ros/jazzy/setup.bash
Screenshot : uses whatever screenshot tool is installed (gnome-screenshot, grim,
         scrot, ImageMagick import, maim). It captures the screen, so keep RViz
         visible. If nothing works, take the screenshot yourself and use --file.

Settings can also come from env vars DOJO_SERVER, DOJO_TEAM, DOJO_TOKEN.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request

PNG_MAGIC = b'\x89PNG\r\n\x1a\n'


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def kind_of(data):
    if data[:2] in (b'P5', b'P2'):
        return 'pgm'
    if data[:8] == PNG_MAGIC or data[:3] == b'\xff\xd8\xff':
        return 'screenshot'
    return None


# ------------------------------------------------------------------ HTTP
class ServerError(Exception):
    pass


def call(args, method, path, params=None, body=None, content_type='application/json'):
    url = args.server.rstrip('/') + path
    if params:
        url += '?' + urllib.parse.urlencode({k: v for k, v in params.items() if v not in (None, '')})
    headers = {'X-Dojo-Token': args.token or ''}
    if body is not None:
        headers['Content-Type'] = content_type
    req = urllib.request.Request(url, data=body, method=method, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            return json.loads(resp.read().decode() or '{}')
    except urllib.error.HTTPError as e:
        try:
            detail = json.loads(e.read().decode()).get('detail', e.reason)
        except Exception:
            detail = e.reason
        raise ServerError(f'server said {e.code}: {detail}')
    except (urllib.error.URLError, OSError) as e:
        raise ServerError(f'cannot reach {args.server} ({getattr(e, "reason", e)})')


def get_pending(args):
    return call(args, 'GET', '/api/maps/pending', {'team': args.team})['requests']


def upload(args, data, filename, request_id=None, meta=None):
    params = {'team': args.team, 'request_id': request_id, 'filename': filename}
    params.update(meta or {})
    return call(args, 'POST', '/api/maps/upload', params, data, 'application/octet-stream')


def report_failure(args, rid, message):
    try:
        call(args, 'POST', f'/api/maps/request/{rid}/fail',
             body=json.dumps({'team': args.team, 'message': message}).encode())
    except ServerError as e:
        log(f'could not report failure: {e}')


# ------------------------------------------------------------ capture: PGM
def occupancy_to_pgm(width, height, data):
    """OccupancyGrid data (int8, row 0 = bottom) -> PGM bytes like nav2 map_saver
    (trinary mode: free 254, occupied 0, unknown 205; image row 0 = top)."""
    table = bytearray(256)
    for i in range(256):
        v = i - 256 if i > 127 else i          # int8 -> signed
        table[i] = 0 if v >= 65 else (254 if 0 <= v <= 25 else 205)
    raw = bytes(data).translate(bytes(table))
    rows = [raw[y * width:(y + 1) * width] for y in range(height - 1, -1, -1)]
    return b'P5\n# CREATOR: dojo_map.py\n%d %d\n255\n' % (width, height) + b''.join(rows)


def capture_pgm(topic, timeout):
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from nav_msgs.msg import OccupancyGrid
    except ImportError:
        raise RuntimeError('ROS 2 not found. Run: source /opt/ros/jazzy/setup.bash '
                           '(and your workspace), then try again')

    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    try:
        node = Node('dojo_map_capture', context=ctx)
        ex = SingleThreadedExecutor(context=ctx)
        ex.add_node(node)
        got = []
        # map publishers are usually TRANSIENT_LOCAL, some are VOLATILE: listen with both
        for dur in (DurabilityPolicy.TRANSIENT_LOCAL, DurabilityPolicy.VOLATILE):
            qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE, durability=dur)
            node.create_subscription(OccupancyGrid, topic, got.append, qos)
        end = time.monotonic() + timeout
        while not got and time.monotonic() < end:
            ex.spin_once(timeout_sec=0.2)
        if not got:
            raise RuntimeError(f'no map received on {topic} within {timeout:.0f}s '
                               f'(is SLAM / map_server running, same ROS_DOMAIN_ID?)')
        m = got[0]
        info = m.info
        pgm = occupancy_to_pgm(info.width, info.height, m.data)
        meta = {'resolution': round(info.resolution, 5),
                'origin_x': round(info.origin.position.x, 4),
                'origin_y': round(info.origin.position.y, 4)}
        return pgm, meta
    finally:
        rclpy.shutdown(context=ctx)


# ------------------------------------------------------ capture: screenshot
def capture_screenshot():
    path = os.path.join(tempfile.mkdtemp(prefix='dojo_'), 'rviz.png')
    wayland = os.environ.get('XDG_SESSION_TYPE') == 'wayland'
    attempts = []
    if not wayland and os.environ.get('DISPLAY') and shutil.which('xdotool') and shutil.which('import'):
        ids = subprocess.run(['xdotool', 'search', '--name', 'RViz'],
                             capture_output=True, text=True).stdout.split()
        if ids:
            attempts.append(['import', '-window', ids[-1], path])     # just the RViz window
    attempts += [['gnome-screenshot', '-f', path], ['grim', path], ['scrot', '-o', path],
                 ['import', '-window', 'root', path], ['maim', path]]
    for cmd in attempts:
        if not shutil.which(cmd[0]):
            continue
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
        if r.returncode == 0 and os.path.exists(path) and os.path.getsize(path) > 0:
            log(f'screenshot taken with {cmd[0]}')
            with open(path, 'rb') as f:
                return f.read()
    raise RuntimeError('no working screenshot tool. Install one (sudo apt install gnome-screenshot) '
                       'or take the RViz screenshot yourself and send it with --file')


# ------------------------------------------------------------------ actions
def produce(args, kind):
    """Capture one item. Returns (bytes, filename, meta)."""
    if kind == 'pgm':
        log(f'grabbing map from {args.topic} ...')
        data, meta = capture_pgm(args.topic, args.timeout)
        return data, 'map.pgm', meta
    return capture_screenshot(), 'rviz.png', {}


def send(args, data, filename, request_id=None, meta=None):
    res = upload(args, data, filename, request_id, meta)
    log(f"OK: {res.get('message')}")


def answer(args, req):
    rid, wanted = req['id'], req['format']
    log(f"judges requested: {wanted}" + (f" from {req['team']}" if req['team'] else '') + f" (request {rid})")
    kinds = ['pgm', 'screenshot'] if wanted == 'any' else [wanted]
    errors = []
    for kind in kinds:
        try:
            data, filename, meta = produce(args, kind)
            send(args, data, filename, rid, meta)
            return True
        except ServerError as e:
            log(f'upload failed: {e}')
            return False
        except Exception as e:                      # capture problem: try next kind
            errors.append(f'{kind}: {e}')
            log(f'could not capture {kind}: {e}')
    report_failure(args, rid, '; '.join(errors))
    return False


def main():
    ap = argparse.ArgumentParser(description='Send your map to the Robotics Dojo judges.')
    ap.add_argument('--server', default=os.environ.get('DOJO_SERVER'), help='e.g. http://192.168.1.10:8000')
    ap.add_argument('--team', default=os.environ.get('DOJO_TEAM', ''), help='your team name')
    ap.add_argument('--token', default=os.environ.get('DOJO_TOKEN', ''), help='only if the judges gave you one')
    ap.add_argument('--watch', action='store_true', help='keep running and answer every request')
    ap.add_argument('--push', choices=['pgm', 'screenshot'], help='send now, without a request')
    ap.add_argument('--file', help='send this .pgm/.png/.jpg instead of capturing')
    ap.add_argument('--topic', default='/map', help='OccupancyGrid topic (default /map)')
    ap.add_argument('--timeout', type=float, default=10.0, help='seconds to wait for the map topic')
    ap.add_argument('--interval', type=float, default=2.0, help='poll period in --watch mode')
    args = ap.parse_args()

    if not args.server:
        ap.error('--server is required (or set DOJO_SERVER)')
    if not args.team:
        ap.error('--team is required (or set DOJO_TEAM)')

    try:
        if args.file:
            with open(args.file, 'rb') as f:
                data = f.read()
            kind = kind_of(data)
            if kind is None:
                sys.exit('error: --file must be a .pgm, .png or .jpg')
            pend = get_pending(args)
            rid = next((r['id'] for r in pend if r['format'] in ('any', kind)), None)
            if rid:
                log('answering a pending request')
            elif pend:
                log(f"WARNING: judges asked for {pend[0]['format']}, but this file is a {kind}; "
                    f"sending it as unrequested - the request stays open")
            else:
                log('no pending request: sending unrequested')
            send(args, data, os.path.basename(args.file), rid)
            return
        if args.push:
            data, filename, meta = produce(args, args.push)
            rid = next((r['id'] for r in get_pending(args) if r['format'] in ('any', args.push)), None)
            send(args, data, filename, rid, meta)
            return

        if args.watch:
            log(f'watching {args.server} as team "{args.team}" (Ctrl+C to stop)')
            warned = False
            while True:
                try:
                    pend = get_pending(args)
                    warned = False
                    if pend:
                        answer(args, pend[0])
                except ServerError as e:
                    if not warned:
                        log(f'{e} - will keep retrying')
                        warned = True
                time.sleep(args.interval)

        pend = get_pending(args)
        if not pend:
            log('no pending request from the judges right now '
                '(use --watch to wait, or --push pgm|screenshot to send anyway)')
            return
        if not answer(args, pend[0]):
            sys.exit(1)
    except ServerError as e:
        sys.exit(f'error: {e}')
    except RuntimeError as e:
        sys.exit(f'error: {e}')
    except KeyboardInterrupt:
        log('stopped')


if __name__ == '__main__':
    main()