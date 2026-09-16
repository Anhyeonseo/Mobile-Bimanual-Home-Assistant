"""Pinned YOLOv8 detection ONNX adapter (axis-aligned, no objectness column).

No downloads or inferred class mapping. User-provided model hash and ordered
class IDs bind the output tensor contract. Bounding boxes do not prove grasp.
"""
import hashlib
from pathlib import Path
import numpy as np
from .capture_port import ImageDetection
from .fetch import InvalidTask
from .navigation import identifier, number


class OnnxDetector:
    def __init__(self, model_path, sha256, class_ids, *, size=640, confidence=.5, iou=.45, session=None):
        path = Path(model_path)
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != sha256:
            raise InvalidTask('detector model missing or hash mismatch')
        if not isinstance(class_ids,list) or not 1 <= len(class_ids) <= 256 or len(set(class_ids)) != len(class_ids):
            raise InvalidTask('ordered unique model class mapping required')
        for name in class_ids: identifier(name,'detector class')
        if type(size) is not int or not 32 <= size <= 2048 or size % 32:
            raise InvalidTask('explicit square model input required')
        if not 0 < number(confidence,'confidence') < 1 or not 0 < number(iou,'IoU') < 1:
            raise InvalidTask('invalid detector thresholds')
        if session is None:
            import onnxruntime as ort
            session = ort.InferenceSession(str(path),providers=['CPUExecutionProvider'])
        inputs = session.get_inputs()
        if len(inputs)!=1 or inputs[0].shape != [1,3,size,size] or inputs[0].type != 'tensor(float)':
            raise InvalidTask('model must accept float32 NCHW RGB with fixed square size')
        self.session,self.input,self.classes,self.size = session,inputs[0].name,tuple(class_ids),size
        self.confidence,self.iou = confidence,iou

    def __call__(self, rgb):
        if not isinstance(rgb,np.ndarray) or rgb.dtype!=np.uint8 or rgb.ndim!=3 or rgb.shape[2]!=3:
            raise InvalidTask('uint8 RGB image required')
        h,w = rgb.shape[:2]
        if not 1 <= min(h,w) <= max(h,w) <= 8192: raise InvalidTask('bounded image required')
        scale=min(self.size/w,self.size/h);nw,nh=max(1,round(w*scale)),max(1,round(h*scale))
        # Deterministic nearest-neighbour resize, explicit RGB/114 letterbox.
        xs=np.minimum((np.arange(nw)/scale).astype(int),w-1)
        ys=np.minimum((np.arange(nh)/scale).astype(int),h-1)
        x0,y0=(self.size-nw)//2,(self.size-nh)//2
        canvas=np.full((self.size,self.size,3),114,np.uint8)
        canvas[y0:y0+nh,x0:x0+nw]=rgb[ys[:,None],xs]
        outputs=self.session.run(None,{self.input:canvas.transpose(2,0,1)[None].astype(np.float32)/255})
        if len(outputs)!=1: raise InvalidTask('one YOLOv8 detection output required')
        values=np.asarray(outputs[0])
        if values.ndim!=3 or values.shape[0]!=1 or values.shape[1]!=4+len(self.classes) or not 1<=values.shape[2]<=50000 or not np.isfinite(values).all():
            raise InvalidTask('expected finite [1,4+classes,anchors] detection output')
        rows=values[0].T; scores=rows[:,4:]; labels=np.argmax(scores,axis=1)
        if np.any(scores<0) or np.any(scores>1): raise InvalidTask('detector scores are not probabilities')
        candidates=[]
        for n in np.argsort(-scores[np.arange(len(rows)),labels],kind='stable'):
            row=rows[n];score=float(scores[n,labels[n]])
            if score<self.confidence:break
            if row[2]<=0 or row[3]<=0:continue
            x1,x2=np.clip((np.array([row[0]-row[2]/2,row[0]+row[2]/2])-x0)/scale,0,w-1)
            y1,y2=np.clip((np.array([row[1]-row[3]/2,row[1]+row[3]/2])-y0)/scale,0,h-1)
            if x2<=x1 or y2<=y1:continue
            box=(float(x1),float(y1),float(x2),float(y2));label=int(labels[n])
            overlap=False
            for previous,other,_ in candidates:
                if other!=label:continue
                a,b,c,d=previous
                intersection=max(0,min(x2,c)-max(x1,a))*max(0,min(y2,d)-max(y1,b))
                union=(x2-x1)*(y2-y1)+(c-a)*(d-b)-intersection
                if intersection/union>self.iou:overlap=True;break
            if not overlap:candidates.append((box,label,score))
            if len(candidates)==64:break
        return [ImageDetection(self.classes[label],score,((a,b),(c,b),(c,d),(a,d))) for (a,b,c,d),label,score in candidates]
