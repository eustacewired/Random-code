#!/usr/bin/env python3
"""dojo_rviz_stream.py - share your RViz window with the judges as a live MJPEG stream.

Run it on the machine that shows RViz (the robot, a Pi desktop or a laptop):

    pip install --break-system-packages mss pillow     # once
    sudo apt install xdotool                           # once (finds the RViz window)
    python3 dojo_rviz_stream.py --server http://<judges-laptop>:8000 --team Alpha --watch

--watch   wait for the judges to press "Request RViz live", then register the stream.
          Without it the stream is registered straight away.
--demo    stream a moving test pattern instead of RViz (check the network, no RViz needed).

How it works: it grabs the RViz window ~8 times a second, serves the frames as MJPEG at
http://<this-machine>:8082/stream and tells the judges' server that URL
(POST /api/maps/stream, kind "rviz"). The judges' browser then loads the stream directly
from this machine, so both must be on the same network.

Needs an X11 desktop (log in with "Ubuntu on Xorg", or run RViz on a virtual display and
pass --display :99). A pure Wayland session shows a black picture.
"""
import argparse
import io
import json
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BOUNDARY = 'dojoframe'
_OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))   # ignore http_proxy env


# --------------------------------------------------------------------- frames
class Latest:
    """Holds the newest JPEG; stream clients wait for the next one."""

    def __init__(self):
        self.cond = threading.Condition()
        self.jpeg = None
        self.seq = 0

    def put(self, jpeg):
        with self.cond:
            self.jpeg = jpeg
            self.seq += 1
            self.cond.notify_all()

    def wait(self, last_seq, timeout=5.0):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last_seq, timeout)
            return self.seq, self.jpeg


def find_window(name):
    """Return {'left','top','width','height'} of the first visible window whose title contains name."""
    if not shutil.which('xdotool'):
        return None
    try:
        ids = subprocess.run(['xdotool', 'search', '--onlyvisible', '--name', name],
                             capture_output=True, text=True, timeout=3).stdout.split()
        for wid in ids:
            out = subprocess.run(['xdotool', 'getwindowgeometry', '--shell', wid],
                                 capture_output=True, text=True, timeout=3).stdout
            g = dict(line.split('=') for line in out.split() if '=' in line)
            if int(g.get('WIDTH', 0)) > 100 and int(g.get('HEIGHT', 0)) > 100:
                return {'left': int(g['X']), 'top': int(g['Y']), 'width': int(g['WIDTH']), 'height': int(g['HEIGHT'])}
    except (OSError, subprocess.SubprocessError, ValueError, KeyError):
        pass
    return None


def capture_loop(latest, args, stop):
    from PIL import Image
    period = 1.0 / args.fps
    if args.demo:
        from PIL import ImageDraw
        n = 0
        while not stop.is_set():
            img = Image.new('RGB', (960, 600), (48, 48, 48))
            d = ImageDraw.Draw(img)
            for x in range(0, 960, 40):
                d.line([(x, 0), (x, 600)], fill=(70, 70, 70))
            for y in range(0, 600, 40):
                d.line([(0, y), (960, y)], fill=(70, 70, 70))
            x = 80 + (n * 6) % 800
            d.ellipse([x - 14, 300 - 14, x + 14, 300 + 14], fill=(0, 200, 120))
            d.text((20, 20), f'dojo_rviz_stream DEMO  team={args.team}  frame {n}  {time.strftime("%H:%M:%S")}', fill=(255, 255, 255))
            buf = io.BytesIO()
            img.save(buf, 'JPEG', quality=args.quality)
            latest.put(buf.getvalue())
            n += 1
            time.sleep(period)
        return

    import mss
    with mss.mss() as sct:
        region, looked = None, 0.0
        while not stop.is_set():
            t0 = time.time()
            if not args.fullscreen and t0 - looked > 1.0:       # follow the window if it moves
                region, looked = find_window(args.window), t0
            if region is None:
                region = sct.monitors[1]                          # window not found: whole screen
            try:
                shot = sct.grab(region)
                img = Image.frombytes('RGB', shot.size, shot.bgra, 'raw', 'BGRX')
                if img.width > args.max_width:
                    img = img.resize((args.max_width, round(img.height * args.max_width / img.width)))
                buf = io.BytesIO()
                img.save(buf, 'JPEG', quality=args.quality)
                latest.put(buf.getvalue())
            except Exception as e:                                # screen locked, window closed ...
                print('capture failed:', e, file=sys.stderr)
                region, looked = None, 0.0
                time.sleep(1)
            time.sleep(max(0.0, period - (time.time() - t0)))


# ----------------------------------------------------------------- http server
def make_handler(latest):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            path = self.path.split('?')[0]
            if path == '/snapshot.jpg':
                _, jpeg = latest.wait(-1, 3)
                if jpeg is None:
                    self.send_error(503, 'no frame yet')
                    return
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(jpeg)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(jpeg)
                return
            if path != '/stream':
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', f'multipart/x-mixed-replace; boundary={BOUNDARY}')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            seq = -1
            try:
                while True:
                    seq, jpeg = latest.wait(seq)
                    if jpeg is None:
                        continue
                    self.wfile.write(f'--{BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}\r\n\r\n'.encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b'\r\n')
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
    return Handler


# ------------------------------------------------------------------ judges' side
def call(args, method, path, body=None):
    headers = {'X-Dojo-Token': args.token}
    data = None
    if body is not None:
        data = json.dumps(body).encode()
        headers['Content-Type'] = 'application/json'
    req = urllib.request.Request(args.server.rstrip('/') + path, data=data, headers=headers, method=method)
    with _OPENER.open(req, timeout=5) as r:
        return json.loads(r.read() or b'{}')


def my_ip(server):
    """The address of this machine on the network that reaches the judges' server."""
    host = urllib.parse.urlparse(server).hostname or '8.8.8.8'
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect((host, 9))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return '127.0.0.1'


def register(args, url, request_id=''):
    try:
        r = call(args, 'POST', '/api/maps/stream',
                 {'team': args.team, 'url': url, 'kind': 'rviz', 'request_id': request_id})
        print(f'registered {url}: {r.get("message")}')
        return True
    except urllib.error.HTTPError as e:
        print(f'judges server said {e.code}: {e.read().decode()[:200]}', file=sys.stderr)
    except OSError as e:
        print(f'cannot reach {args.server}: {e}', file=sys.stderr)
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--server', required=True, help='judges server, e.g. http://192.168.1.10:8000')
    ap.add_argument('--team', required=True)
    ap.add_argument('--token', default='', help='X-Dojo-Token, if the judges set one')
    ap.add_argument('--watch', action='store_true', help='wait for "Request RViz live" from the judges')
    ap.add_argument('--demo', action='store_true', help='stream a test pattern instead of RViz')
    ap.add_argument('--port', type=int, default=8082)
    ap.add_argument('--host', default='', help='address the judges should use (default: auto-detected)')
    ap.add_argument('--window', default='RViz', help='window title to capture (default: RViz)')
    ap.add_argument('--fullscreen', action='store_true', help='capture the whole screen instead of the RViz window')
    ap.add_argument('--display', default='', help='X display to capture, e.g. :99')
    ap.add_argument('--fps', type=float, default=8.0)
    ap.add_argument('--quality', type=int, default=70, help='JPEG quality 1-95')
    ap.add_argument('--max-width', type=int, default=1280)
    args = ap.parse_args()
    if args.display:
        import os
        os.environ['DISPLAY'] = args.display

    if not args.demo:
        for mod, apt in (('PIL', 'python3-pil'), ('mss', 'python3-mss')):
            try:
                __import__(mod)
            except ImportError:
                sys.exit(f'missing Python module "{mod}". Online: pip install --break-system-packages mss pillow '
                         f'| offline: sudo apt install {apt}')
        if os.environ.get('XDG_SESSION_TYPE') == 'wayland' and not args.display:
            print('WARNING: Wayland session - the picture will probably be BLACK. Log out and log in with '
                  '"Ubuntu on Xorg", or send screenshots with dojo_map.py instead.', file=sys.stderr)
        if not shutil.which('xdotool'):
            print('note: xdotool not found - streaming the whole screen (keep RViz visible)', file=sys.stderr)

    latest, stop = Latest(), threading.Event()
    threading.Thread(target=capture_loop, args=(latest, args, stop), daemon=True).start()
    httpd = None
    for port in range(args.port, args.port + 10):         # port busy (old run still alive)? take the next one
        try:
            httpd = ThreadingHTTPServer(('0.0.0.0', port), make_handler(latest))
            args.port = port
            break
        except OSError:
            continue
    if httpd is None:
        sys.exit(f'no free port in {args.port}-{args.port + 9}')
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    ip = args.host or my_ip(args.server)
    if ip.startswith('127.'):
        print('WARNING: could not work out this machine\'s address - judges cannot reach 127.x. '
              'Use --host <your-ip> (tailscale ip -4 if on Tailscale)', file=sys.stderr)
    url = f'http://{ip}:{args.port}/stream'
    print(f'serving {url}  (Ctrl-C to stop)')

    try:
        if not args.watch:
            while True:                       # re-register: survives a late network, a server restart
                register(args, url)
                time.sleep(30)
        print('waiting for an RViz live request from the judges...')
        while True:
            try:
                reqs = call(args, 'GET', '/api/maps/pending?team=' + urllib.parse.quote(args.team)).get('requests', [])
                for r in reqs:
                    if r.get('format') == 'rviz_live':
                        register(args, url, r['id'])
                        break
            except (OSError, ValueError, KeyError, TypeError):
                pass
            time.sleep(2)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        try:
            call(args, 'POST', '/api/maps/stream', {'team': args.team, 'url': '', 'kind': 'rviz'})
        except (OSError, ValueError):
            pass
        httpd.shutdown()


if __name__ == '__main__':
    main()
