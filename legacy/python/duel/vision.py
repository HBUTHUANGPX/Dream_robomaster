"""RGB-only armor detection and calibrated planar PnP."""
from dataclasses import dataclass
import cv2
import numpy as np
from duel.classifier import NumberClassifier


@dataclass
class Detection:
    corners: np.ndarray
    tvec: np.ndarray
    rvec: np.ndarray
    error: float
    projected: np.ndarray
    number: object=None
    confidence: float=0.
    number_roi: object=None
    pose_candidates: object=None


class Detector:
    def __init__(self, width=800, height=600, fovy=45, enemy='red'):
        f = .5*height/np.tan(np.deg2rad(fovy)/2)
        self.K = np.array([[f,0,(width-1)/2],[0,f,(height-1)/2],[0,0,1.]])
        self.enemy = enemy
        self.classifier = NumberClassifier()
        # Visible light centreline endpoints from the existing MJCF model.
        self.points = np.array([[-.062,-.026,0],[-.062,.026,0],
                                [.062,-.026,0],[.062,.026,0]], np.float64)

    def detect(self, rgb):
        channels = rgb.astype(np.int16)
        main, other = (0,2) if self.enemy == 'red' else (2,0)
        mask = ((channels[:,:,main] > 100) &
                (channels[:,:,main]-channels[:,:,other] > 65) &
                (channels[:,:,main]-channels[:,:,1] > 45)).astype(np.uint8)*255
        contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        lights = []
        for contour in contours:
            # A distant 6 mm strip can be one pixel wide (zero contour area).
            if len(contour) < 4:
                continue
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            edges = np.roll(box,-1,axis=0)-box
            lengths = np.linalg.norm(edges,axis=1)
            long = int(np.argmax(lengths))
            height, width = lengths[long]+1, lengths[(long+1)%4]+1
            axis = edges[long]/max(lengths[long],1e-6)
            if axis[1] < 0:
                axis = -axis
            if height < 5 or height/width < 1.5 or abs(axis[1]) < .7:
                continue
            center = np.array(rect[0])
            lights.append((center,center-axis*height/2,center+axis*height/2,height))
        lights.sort(key=lambda light: light[0][0])
        detections = []
        for i,left in enumerate(lights):
            for right in lights[i+1:]:
                mean_height = (left[3]+right[3])/2
                dx,dy = right[0]-left[0]
                if not .8 < dx/mean_height < 3.8 or abs(dy) > mean_height*.65:
                    continue
                if min(left[3],right[3])/max(left[3],right[3]) < .6:
                    continue
                corners = np.array([left[1],left[2],right[1],right[2]],np.float64)
                ok,rvecs,tvecs,_ = cv2.solvePnPGeneric(self.points,corners,self.K,None,flags=cv2.SOLVEPNP_IPPE)
                if not ok:
                    continue
                poses=[]
                for rvec,tvec in zip(rvecs,tvecs):
                    tvec=tvec.ravel()
                    if not np.isfinite(tvec).all() or not .3 < tvec[2] < 15:
                        continue
                    projected,_ = cv2.projectPoints(self.points,rvec,tvec,self.K,None)
                    projected=projected.reshape(-1,2)
                    error=float(np.sqrt(np.mean(np.sum((projected-corners)**2,axis=1))))
                    if error < 2.5:
                        poses.append((rvec,tvec,error,projected))
                if poses:
                    number,confidence,roi=self.classifier.classify(rgb,corners)
                    rvec,tvec,error,projected=poses[0]
                    detections.append(Detection(corners,tvec,rvec,error,projected,number,confidence,roi,poses))
        return sorted(detections,key=lambda d:d.error)

    def annotate(self, rgb, detection=None, prediction=None):
        frame=rgb.copy()
        h,w=frame.shape[:2]
        cv2.drawMarker(frame,(w//2,h//2),(220,220,220),cv2.MARKER_CROSS,18,1)
        if detection is not None:
            p=detection.corners.astype(np.int32)
            cv2.polylines(frame,[p[[0,2,3,1]]],True,(70,255,130),1,cv2.LINE_AA)
            for xy in detection.projected:
                cv2.circle(frame,tuple(np.round(xy).astype(int)),2,(255,215,70),-1)
            cv2.putText(frame,f'PnP {np.linalg.norm(detection.tvec):.2f}m  reproj {detection.error:.2f}px',
                        (12,25),cv2.FONT_HERSHEY_SIMPLEX,.55,(90,255,150),1,cv2.LINE_AA)
            cv2.putText(frame,f'ID {detection.number or "?"}  ONNX {detection.confidence:.1%}',
                        (12,48),cv2.FONT_HERSHEY_SIMPLEX,.55,(90,255,150),1,cv2.LINE_AA)
        if prediction is not None and prediction[2] > .1:
            xy=self.K @ prediction; xy=xy[:2]/xy[2]
            if np.isfinite(xy).all() and -1000 < xy[0] < w+1000 and -1000 < xy[1] < h+1000:
                cv2.drawMarker(frame,tuple(np.round(xy).astype(int)),(255,185,45),cv2.MARKER_DIAMOND,14,2)
        return frame

    def refine_yaw(self,detection,position,camera,rotation):
        """Fixed mounting tilt reprojection search, as in sp_vision_25's solver.

        Uses measured PnP translation and RGB corners only. Unlike upstream's
        inward convention, this returns the outward normal's world yaw.
        """
        bearing=np.arctan2(camera[1]-position[1],camera[0]-position[0])
        yaws=bearing+np.deg2rad(np.arange(-80,81,dtype=float))
        c,s=np.cos(yaws),np.sin(yaws)
        horizontal=np.stack([-s,c,np.zeros_like(c)],axis=1)
        vertical=np.stack([-np.sin(np.pi/12)*c,-np.sin(np.pi/12)*s,np.full_like(c,np.cos(np.pi/12))],axis=1)
        world=position+self.points[None,:,0,None]*horizontal[:,None,:]-self.points[None,:,1,None]*vertical[:,None,:]
        local=(world-camera) @ rotation
        pixels=local @ self.K.T
        pixels=pixels[:,:,:2]/pixels[:,:,2,None]
        errors=np.mean(np.sum((pixels-detection.corners)**2,axis=2),axis=1)
        return float(yaws[np.argmin(errors)])
