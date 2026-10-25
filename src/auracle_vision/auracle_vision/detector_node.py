import os
import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

NAMES = ['metal', 'organic', 'plastic']
IMG = 640


def letterbox(img):
    h, w = img.shape[:2]
    r = min(IMG / h, IMG / w)
    nh, nw = round(h * r), round(w * r)
    resized = cv2.resize(img, (nw, nh))
    canvas = np.full((IMG, IMG, 3), 114, np.uint8)
    top, left = (IMG - nh) // 2, (IMG - nw) // 2
    canvas[top:top + nh, left:left + nw] = resized
    return canvas, r, left, top


class WasteDetector(Node):
    def __init__(self):
        super().__init__('waste_detector')
        default_model = os.path.join(
            get_package_share_directory('auracle_vision'),
            'models', 'waste_yolo26n.onnx')
        self.declare_parameter('model_path', default_model)
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('iou', 0.45)
        self.declare_parameter('image_topic', '/camera/image_raw')

        self.conf = self.get_parameter('conf').value
        self.iou = self.get_parameter('iou').value
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 4
        self.sess = ort.InferenceSession(
            self.get_parameter('model_path').value, opts,
            providers=['CPUExecutionProvider'])
        self.inp = self.sess.get_inputs()[0].name
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Detection2DArray, '/vision/detections', 10)
        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.on_image, 1)

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        canvas, r, left, top = letterbox(frame)
        blob = np.ascontiguousarray(
            canvas[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32) / 255.0

        preds = self.sess.run(None, {self.inp: blob})[0][0].T   # (8400, 7)
        scores = preds[:, 4:]
        cls = scores.argmax(1)
        conf = scores.max(1)
        keep = conf > self.conf
        preds, cls, conf = preds[keep], cls[keep], conf[keep]

        out = Detection2DArray()
        out.header = msg.header
        if len(conf):
            cx, cy, w, h = preds[:, 0], preds[:, 1], preds[:, 2], preds[:, 3]
            x = (cx - w / 2 - left) / r
            y = (cy - h / 2 - top) / r
            bw, bh = w / r, h / r
            boxes = np.stack([x, y, bw, bh], 1)
            idx = cv2.dnn.NMSBoxes(boxes.tolist(), conf.tolist(), self.conf, self.iou)
            for i in np.array(idx).flatten():
                d = Detection2D()
                d.bbox.center.position.x = float(x[i] + bw[i] / 2)
                d.bbox.center.position.y = float(y[i] + bh[i] / 2)
                d.bbox.size_x = float(bw[i])
                d.bbox.size_y = float(bh[i])
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = NAMES[int(cls[i])]
                hyp.hypothesis.score = float(conf[i])
                d.results.append(hyp)
                out.detections.append(d)
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(WasteDetector())
    rclpy.shutdown()
