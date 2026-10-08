"""Original rm_auto_aim MLP inference and number preprocessing.

Preprocessing adapted from ChenJun's MIT-licensed number_classifier.cpp;
see assets/rm_auto_aim/LICENSE and source.json. No simulator dependencies.
"""
from pathlib import Path
import cv2
import numpy as np

MODEL_DIR=Path(__file__).resolve().parents[1]/'assets/rm_auto_aim'


class NumberClassifier:
    def __init__(self,threshold=.8):
        self.net=cv2.dnn.readNetFromONNX(str(MODEL_DIR/'mlp.onnx'))
        self.labels=(MODEL_DIR/'label.txt').read_text().splitlines()
        self.threshold=threshold

    def classify_roi(self,gray):
        gray=cv2.resize(gray,(20,28),interpolation=cv2.INTER_LINEAR)
        _,binary=cv2.threshold(gray,0,255,cv2.THRESH_BINARY|cv2.THRESH_OTSU)
        self.net.setInput(cv2.dnn.blobFromImage(binary.astype(np.float32)/255.))
        logits=self.net.forward().ravel()
        probabilities=np.exp(logits-logits.max());probabilities/=probabilities.sum()
        index=int(probabilities.argmax());confidence=float(probabilities[index])
        label=self.labels[index]
        # These are small armor plates; use the upstream type/ID constraints.
        if confidence<self.threshold or label in ('negative','1','base') or np.std(gray)<2:
            label=None
        return label,confidence,binary

    def classify(self,rgb,corners):
        # Detector order: left top, left bottom, right top, right bottom.
        source=np.asarray(corners[[1,0,2,3]],np.float32)
        destination=np.array([[0,19],[0,7],[31,7],[31,19]],np.float32)
        transform=cv2.getPerspectiveTransform(source,destination)
        warped=cv2.warpPerspective(rgb,transform,(32,28))
        gray=cv2.cvtColor(warped[:,6:26],cv2.COLOR_RGB2GRAY)
        return self.classify_roi(gray)
