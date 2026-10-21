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

  Detection (the frame the robot classified + its label) - judges can request it,
  or the detector pushes it as soon as it decides:
      python3 dojo_map.py --server ... --team Alpha --push detection --label plastic --confidence 0.93
      python3 dojo_map.py --server ... --team Alpha --push detection --det-file /tmp/det.jpg --label metal
  Live camera (Pi Camera Module 3) for the judges, served at http://<robot-ip>:8081/stream:
      python3 dojo_map.py --server ... --team Alpha --watch --stream

PGM    : grabs the OccupancyGrid on --topic (default /map) and writes it the same
         way nav2 map_saver does. Source ROS first:  source /opt/ros/jazzy/setup.bash
Screenshot : uses whatever screenshot tool is installed (gnome-screenshot, grim,
         scrot, ImageMagick import, maim). It captures the screen, so keep RViz
         visible. If nothing works, take the screenshot yourself and use --file.
Detection : the image comes from (first that works) --det-file, the ROS topic
         --det-topic (sensor_msgs/CompressedImage, default /detection/image/compressed),
         or a fresh still from the camera. The label comes from --label, else the
         std_msgs/String topic --label-topic (default /detection/class), which may
         hold "plastic" or "plastic:0.93". Classes: organic, plastic, metal.
Stream : MJPEG from --stream-topic (CompressedImage jpeg, use this when your own
         camera node already owns the camera), else Picamera2, else rpicam-vid.
         The URL is registered with the judges automatically (--stream-url to override).

Settings can also come from env vars DOJO_SERVER, DOJO_TEAM, DOJO_TOKEN.
"""
import argparse
import io
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PNG_MAGIC = b'\x89PNG\r\n\x1a\n'
CLASSES = ('organic', 'plastic', 'metal')


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


def register_stream(args, url):
    call(args, 'POST', '/api/maps/stream', body=json.dumps({'team': args.team, 'url': url}).encode())


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


# ------------------------------------------------------------ camera frames
class FrameHub:
    """Latest JPEG frame from whichever camera source is running; HTTP clients wait on it."""
    def __init__(self):
        self.cond = threading.Condition()
        self.frame, self.count, self.source = None, 0, ''

    def put(self, jpg):
        with self.cond:
            self.frame, self.count = jpg, self.count + 1
            self.cond.notify_all()

    def wait(self, last, timeout=5.0):
        with self.cond:
            self.cond.wait_for(lambda: self.count != last, timeout)
            return self.frame, self.count


hub = FrameHub()


def _ros_frames(topic, stop):
    """Feed hub from a sensor_msgs/CompressedImage (jpeg) topic until stop is set."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    try:
        node = Node('dojo_stream', context=ctx)
        ex = SingleThreadedExecutor(context=ctx)
        ex.add_node(node)
        node.create_subscription(CompressedImage, topic, lambda m: hub.put(bytes(m.data)), qos_profile_sensor_data)
        while not stop.is_set():
            ex.spin_once(timeout_sec=0.2)
    finally:
        rclpy.shutdown(context=ctx)


def _picamera2_frames(size, stop):
    from picamera2 import Picamera2
    from picamera2.encoders import JpegEncoder
    from picamera2.outputs import FileOutput

    class Out(io.BufferedIOBase):
        def write(self, buf):
            hub.put(bytes(buf))
            return len(buf)

    cam = Picamera2()
    cam.configure(cam.create_video_configuration(main={'size': size}))
    cam.start_recording(JpegEncoder(q=70), FileOutput(Out()))
    try:
        stop.wait()
    finally:
        cam.stop_recording()
        cam.close()


def _rpicam_frames(size, stop):
    exe = shutil.which('rpicam-vid') or shutil.which('libcamera-vid')
    if not exe:
        raise RuntimeError('neither picamera2 nor rpicam-vid is installed')
    p = subprocess.Popen([exe, '-t', '0', '-n', '--codec', 'mjpeg', '--width', str(size[0]),
                          '--height', str(size[1]), '--framerate', '15', '-o', '-'],
                         stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    buf = b''
    try:
        while not stop.is_set():
            chunk = p.stdout.read(65536)
            if not chunk:
                raise RuntimeError(f'{os.path.basename(exe)} exited (is the camera used by another process?)')
            buf += chunk
            while True:                      # cut complete JPEGs out of the byte stream
                a = buf.find(b'\xff\xd8')
                b = buf.find(b'\xff\xd9', a + 2)
                if a < 0 or b < 0:
                    break
                hub.put(buf[a:b + 2])
                buf = buf[b + 2:]
            if len(buf) > 4 << 20:
                buf = b''
    finally:
        p.terminate()


def start_camera(args, stop):
    """Start the first camera source that works in a background thread."""
    size = tuple(int(v) for v in args.stream_size.lower().split('x'))
    sources = []
    if args.stream_topic:
        sources.append((f'ROS topic {args.stream_topic}', lambda: _ros_frames(args.stream_topic, stop)))
    sources += [('Picamera2', lambda: _picamera2_frames(size, stop)), ('rpicam-vid', lambda: _rpicam_frames(size, stop))]

    def runner():
        for name, fn in sources:
            start = hub.count
            hub.source = name
            try:
                log(f'camera: trying {name}')
                fn()
                return
            except Exception as e:
                if hub.count != start and not stop.is_set():
                    log(f'camera: {name} stopped: {e}')
                else:
                    log(f'camera: {name} unavailable ({e})')
        hub.source = ''
        log('camera: no source worked - stream disabled')
    threading.Thread(target=runner, daemon=True).start()


class StreamHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *a):
        pass

    def do_GET(self):
        path = self.path.split('?')[0]
        if path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            last = -1
            try:
                while True:
                    frame, last = hub.wait(last)
                    if frame is None:
                        continue
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n' % len(frame))
                    self.wfile.write(frame + b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                return
        elif path in ('/snapshot.jpg', '/snapshot'):
            frame, _ = hub.wait(-1, 3.0)
            if frame is None:
                self.send_error(503, 'no camera frame yet')
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(frame)))
            self.end_headers()
            self.wfile.write(frame)
        else:
            body = b'<html><body style="margin:0;background:#000"><img src="/stream" style="width:100%"></body></html>'
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)


def own_ip(server):
    """The address the judges' laptop can reach us on (the interface routing to it)."""
    host = urllib.parse.urlparse(server).hostname or '8.8.8.8'
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.connect((host, 80))
            return s.getsockname()[0]
        except OSError:
            return socket.gethostbyname(socket.gethostname())


def start_stream(args):
    stop = threading.Event()
    start_camera(args, stop)
    srv = ThreadingHTTPServer(('0.0.0.0', args.stream_port), StreamHandler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    url = args.stream_url or f'http://{own_ip(args.server)}:{args.stream_port}/stream'
    log(f'live camera at {url}')
    return url


# ------------------------------------------------------- capture: detection
def parse_label(text):
    """'plastic', 'Plastic 0.93', 'plastic:0.93' or '{"label": .., "confidence": ..}' -> (label, conf)."""
    text = (text or '').strip()
    try:
        d = json.loads(text)
        if isinstance(d, dict):
            return str(d.get('label', d.get('class', ''))).lower(), d.get('confidence')
    except ValueError:
        pass
    m = re.match(r'([A-Za-z]+)\W*([0-9.]+)?', text)
    if not m:
        return '', None
    label = m.group(1).lower()
    label = 'metal' if label == 'other' else label
    return label, float(m.group(2)) if m.group(2) else None


def _ros_detection(img_topic, label_topic, timeout):
    """Latest CompressedImage + String label from the robot's detector (None if no image)."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    try:
        node = Node('dojo_detection_capture', context=ctx)
        ex = SingleThreadedExecutor(context=ctx)
        ex.add_node(node)
        imgs, labels = [], []
        for dur in (DurabilityPolicy.TRANSIENT_LOCAL, DurabilityPolicy.VOLATILE):
            for rel in (ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT):
                qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=rel, durability=dur)
                node.create_subscription(CompressedImage, img_topic, imgs.append, qos)
                node.create_subscription(String, label_topic, labels.append, qos)
        end = time.monotonic() + timeout
        while not imgs and time.monotonic() < end:
            ex.spin_once(timeout_sec=0.2)
        end = time.monotonic() + 0.5          # the label usually arrives with the image
        while imgs and not labels and time.monotonic() < end:
            ex.spin_once(timeout_sec=0.1)
        if not imgs:
            return None
        return bytes(imgs[-1].data), (labels[-1].data if labels else '')
    finally:
        rclpy.shutdown(context=ctx)


def camera_still():
    if hub.frame is not None:                # stream already running: reuse its frame
        return hub.frame
    try:
        from picamera2 import Picamera2
        cam = Picamera2()
        cam.configure(cam.create_still_configuration(main={'size': (1280, 960)}))
        cam.start()
        time.sleep(1.0)                      # let auto-exposure settle
        out = io.BytesIO()
        cam.capture_file(out, format='jpeg')
        cam.close()
        return out.getvalue()
    except Exception:
        pass
    exe = shutil.which('rpicam-still') or shutil.which('libcamera-still')
    if exe:
        r = subprocess.run([exe, '-n', '-t', '1000', '--width', '1280', '--height', '960', '-o', '-'],
                           capture_output=True, timeout=20)
        if r.returncode == 0 and r.stdout[:3] == b'\xff\xd8\xff':
            return r.stdout
    raise RuntimeError('no detection image: give --det-file, publish --det-topic, or attach the Pi camera')


def capture_detection(args):
    data, label_text = None, ''
    if args.det_file:
        with open(args.det_file, 'rb') as f:
            data = f.read()
    else:
        try:
            got = _ros_detection(args.det_topic, args.label_topic, min(args.timeout, 3.0))
            if got:
                data, label_text = got
                log(f'detection image from {args.det_topic}')
        except ImportError:
            pass                              # no ROS sourced: fall back to the camera
    if data is None:
        log('taking a camera still for the detection')
        data = camera_still()
    label, conf = parse_label(args.label or label_text)
    if args.confidence is not None:
        conf = args.confidence
    if label and label not in CLASSES:
        log(f'WARNING: label "{label}" is not one of {", ".join(CLASSES)}')
    return data, {'kind': 'detection', 'label': label, 'confidence': conf}


# ------------------------------------------------------------------ actions
def produce(args, kind):
    """Capture one item. Returns (bytes, filename, meta)."""
    if kind == 'pgm':
        log(f'grabbing map from {args.topic} ...')
        data, meta = capture_pgm(args.topic, args.timeout)
        return data, 'map.pgm', meta
    if kind == 'detection':
        data, meta = capture_detection(args)
        log(f"detection: {meta['label'] or 'no label'}")
        return data, 'detection.png' if data[:8] == PNG_MAGIC else 'detection.jpg', meta
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
    ap.add_argument('--push', choices=['pgm', 'screenshot', 'detection'], help='send now, without a request')
    ap.add_argument('--file', help='send this .pgm/.png/.jpg instead of capturing')
    ap.add_argument('--topic', default='/map', help='OccupancyGrid topic (default /map)')
    ap.add_argument('--timeout', type=float, default=10.0, help='seconds to wait for the map topic')
    ap.add_argument('--interval', type=float, default=2.0, help='poll period in --watch mode')
    det = ap.add_argument_group('detection')
    det.add_argument('--label', default='', help='detected class: organic, plastic or metal')
    det.add_argument('--confidence', type=float, help='detector confidence 0..1')
    det.add_argument('--det-file', help='image the detector classified (.jpg/.png)')
    det.add_argument('--det-topic', default='/detection/image/compressed', help='CompressedImage topic with the classified frame')
    det.add_argument('--label-topic', default='/detection/class', help='String topic with the detected class')
    cam = ap.add_argument_group('live camera')
    cam.add_argument('--stream', action='store_true', help='serve the camera as MJPEG and register it with the judges')
    cam.add_argument('--stream-port', type=int, default=8081)
    cam.add_argument('--stream-size', default='640x480', help='WIDTHxHEIGHT (Picamera2 / rpicam-vid)')
    cam.add_argument('--stream-topic', default='', help='use this CompressedImage topic instead of opening the camera')
    cam.add_argument('--stream-url', default='', help='URL to give the judges (default http://<this-ip>:PORT/stream)')
    args = ap.parse_args()

    if not args.server:
        ap.error('--server is required (or set DOJO_SERVER)')
    if not args.team:
        ap.error('--team is required (or set DOJO_TEAM)')

    stream_url = None
    if args.stream:
        stream_url = start_stream(args)
        try:
            register_stream(args, stream_url)
            log('stream registered with the judges')
        except ServerError as e:
            log(f'could not register stream yet: {e}')

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
            # 'any' requests are for maps only; a detection answers only a detection request
            ok = ('detection',) if args.push == 'detection' else ('any', args.push)
            rid = next((r['id'] for r in get_pending(args) if r['format'] in ok), None)
            send(args, data, filename, rid, meta)
            if not args.watch and not args.stream:
                return

        if args.watch or args.stream:
            log(f'watching {args.server} as team "{args.team}" (Ctrl+C to stop)'
                if args.watch else 'serving the camera (Ctrl+C to stop)')
            warned = False
            last_reg = time.monotonic()
            while True:
                try:
                    if args.watch:
                        pend = get_pending(args)
                        if pend:
                            answer(args, pend[0])
                    if stream_url and time.monotonic() - last_reg > 30:   # survives a server restart
                        register_stream(args, stream_url)
                        last_reg = time.monotonic()
                    warned = False
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