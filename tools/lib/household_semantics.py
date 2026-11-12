"""Optional local crop review for household semantic mapping, never motion input."""
import numpy as np

MODEL_ID='google/siglip2-base-patch16-224'
MODEL_REVISION='75de2d55ec2d0b4efc50b3e9ad70dba96a7b2fa2'
# Include background and small objects so the model need not call everything furniture.
LABELS={
    'table':'a table', 'chair':'a chair', 'sofa':'a sofa or couch',
    'bed':'a bed', 'refrigerator':'a refrigerator or fridge',
    'cabinet':'a storage cabinet or cupboard', 'wardrobe':'a wardrobe for clothes',
    'bookshelf':'a bookshelf', 'nightstand':'a bedside table',
    'storage_basket':'a storage basket', 'fan':'a small electric fan',
    'air_purifier':'an air purifier', 'floor':'an empty floor', 'wall':'a wall',
    'door':'a door', 'curtain':'a curtain'}


def crop_boxes(box,shape):
    h,w=shape[:2];a,b,c,d=map(float,box)
    if not np.isfinite([a,b,c,d]).all() or not 0<=a<c<=w or not 0<=b<d<=h:
        raise ValueError('bounded crop required')
    result=[]
    for fraction in (0.,.12):
        dx=(c-a)*fraction;dy=(d-b)*fraction
        result.append([max(0,int(np.floor(a-dx))),max(0,int(np.floor(b-dy))),
                       min(w,int(np.ceil(c+dx))),min(h,int(np.ceil(d+dy)))])
    return result


def review_scores(original_label,logits,labels):
    """Two-context agreement is evidence, not a calibrated correctness probability."""
    values=np.asarray(logits,float)
    if values.shape!=(2,len(labels)) or len(labels)<2 or not np.isfinite(values).all():
        raise ValueError('two finite score rows required')
    rankings=np.argsort(-values,axis=1);agrees=rankings[0,0]==rankings[1,0]
    margins=[float(values[i,r[0]]-values[i,r[1]]) for i,r in enumerate(rankings)]
    # Near ties stay unresolved. A 0.5 logit gap is an explicit review heuristic.
    consistent=agrees and min(margins)>=.5
    proposed=labels[rankings[0,0]]
    major={'table','chair','sofa','bed','refrigerator','cabinet','wardrobe','bookshelf','nightstand'}
    original=original_label.strip().lower().rstrip('.').replace('_',' ')
    if original in ('desk','coffee table'):original='table'
    original_options={label for label in major if label in original.split()}
    resolved=consistent
    label=proposed if consistent else original_label
    # Different major classes are a disagreement, not permission to rename.
    # Preserve object geometry under a generic name until cross-view/user evidence.
    conflict=bool(consistent and proposed in major and original_options and original_options!={proposed})
    if conflict:
        label='unclassified_furniture';resolved=False
    return dict(label=label,proposed_label=proposed,cross_model_conflict=conflict,
        original_label=original_label,context_agreement=bool(agrees),resolved=bool(resolved),
        minimum_logit_gap=.5,logit_margins=margins,scores_are_probabilities=False,
        alternatives=[[dict(label=labels[j],logit=float(values[i,j])) for j in row[:3]] for i,row in enumerate(rankings)])


class HouseholdCropReviewer:
    def __init__(self,cache_dir,device='cuda'):
        import torch
        from transformers import AutoProcessor,AutoModel
        self.torch,self.device=torch,device
        options=dict(revision=MODEL_REVISION,cache_dir=str(cache_dir),local_files_only=True,trust_remote_code=False)
        self.processor=AutoProcessor.from_pretrained(MODEL_ID,**options)
        self.model=AutoModel.from_pretrained(MODEL_ID,**options).to(device).eval()
        self.labels=list(LABELS)
        self.text=[f'This is a photo of {phrase} in a home.' for phrase in LABELS.values()]

    def refine(self,bgr,detections):
        from PIL import Image
        if len(detections)>64:raise ValueError('bounded detections required')
        result=[]
        for detection in detections:
            boxes=crop_boxes(detection['box_xyxy'],bgr.shape)
            images=[Image.fromarray(bgr[y1:y2,x1:x2,::-1].copy()) for x1,y1,x2,y2 in boxes]
            inputs=self.processor(text=self.text,images=images,padding='max_length',max_length=64,return_tensors='pt').to(self.device)
            with self.torch.inference_mode():logits=self.model(**inputs).logits_per_image.float().cpu().numpy()
            review=review_scores(detection['label'],logits,self.labels)
            result.append(dict(detection,label=review['label'],original_label=detection['label'],
                household_review=dict(review,model_id=MODEL_ID,model_revision=MODEL_REVISION,crop_boxes=boxes)))
        return result
