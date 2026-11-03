import cv2
import numpy as np
from rknnlite.api import RKNNLite

_OBJ_THRESH = 0.35
_NMS_THRESH = 0.45
INPUT_SIZE = 640
CLS_PERSON = 0


def _nms(boxes, scores, thresh):
    x1 = boxes[:, 0] - boxes[:, 2] / 2
    y1 = boxes[:, 1] - boxes[:, 3] / 2
    x2 = boxes[:, 0] + boxes[:, 2] / 2
    y2 = boxes[:, 1] + boxes[:, 3] / 2
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)
        inds = np.where(iou <= thresh)[0]
        order = order[inds + 1]
    return keep


class Detector:
    def __init__(self, model_path):
        self.rknn = RKNNLite()
        ret = self.rknn.load_rknn(model_path)
        if ret != 0:
            raise RuntimeError('load_rknn failed %d' % ret)
        ret = self.rknn.init_runtime()
        if ret != 0:
            raise RuntimeError('init_runtime failed %d' % ret)

    def detect(self, frame):
        h0, w0 = frame.shape[:2]
        scale = min(INPUT_SIZE / w0, INPUT_SIZE / h0)
        nw = int(w0 * scale)
        nh = int(h0 * scale)
        img = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((INPUT_SIZE, INPUT_SIZE, 3), 114, dtype=np.uint8)
        canvas[0:nh, 0:nw] = img
        blob = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        blob = blob.astype(np.float32) / 255.0
        blob = np.transpose(blob, (2, 0, 1))[None, ...]
        out = self.rknn.inference(inputs=[blob])
        pred = out[0]
        if pred.ndim == 3:
            pred = pred[0]
        boxes = pred[:, 0:4]
        obj = pred[:, 4]
        cls_probs = pred[:, 5:]
        cls_ids = np.argmax(cls_probs, axis=1)
        cls_conf = np.max(cls_probs, axis=1)
        score = obj * cls_conf
        mask = score > _OBJ_THRESH
        boxes = boxes[mask]
        score = score[mask]
        cls_ids = cls_ids[mask]
        if boxes.size == 0:
            return []
        keep = _nms(boxes, score, _NMS_THRESH)
        result = []
        for i in keep:
            cx, cy, w, h = boxes[i]
            result.append({
                'cls': int(cls_ids[i]),
                'conf': float(score[i]),
                'cx': float(cx),
                'cy': float(cy),
                'w': float(w),
                'h': float(h),
            })
        return result

    def release(self):
        try:
            self.rknn.release()
        except Exception:
            pass
