"""Conditional cha/sang expert and bounded glyph-colour side evidence."""
from pathlib import Path
import math

import cv2
import numpy as np
from PIL import Image

DEFAULT_EXPERT = Path(__file__).resolve().parents[2]/'models/piece_classifier_cha_sang_224/best.pt'


def glyph_colour_evidence(image):
    """Central saturated glyphs only; green and blue both mean the blue/Cho side."""
    rgb = np.asarray(image.convert('RGB'))
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    h, s, v = (hsv[:,:,i] for i in range(3))
    height, width = h.shape
    yy, xx = np.ogrid[:height,:width]
    centre = ((xx-(width-1)/2)/(width*.38))**2 + ((yy-(height-1)/2)/(height*.38))**2 <= 1
    eligible = centre & (s >= 64) & (v >= 30)
    red = eligible & ((h <= 12) | (h >= 150))
    cho = eligible & (h >= 35) & (h <= 140)
    count = int(red.sum()+cho.sum())
    area = int(centre.sum())
    if count < max(12, area*.006):
        return dict(red_fraction=.5, reliability=0., pixels=count)
    weights = s.astype(float)/255.
    red_weight, cho_weight = float(weights[red].sum()), float(weights[cho].sum())
    fraction = (red_weight+2.)/(red_weight+cho_weight+4.)
    coverage = min(1., count/max(1., area*.04))
    return dict(red_fraction=fraction, reliability=coverage*abs(2*fraction-1), pixels=count)


class RefinedClassifier:
    def __init__(self, base, expert=None, *, threshold=.85, expert_strength=.75, colour_strength=1.5):
        if not 0 <= threshold <= 1 or not 0 <= expert_strength <= 1:
            raise ValueError('expert threshold/strength must be in [0, 1]')
        if not math.isfinite(colour_strength) or not 0 <= colour_strength <= 3:
            raise ValueError('colour strength must be finite and in [0, 3]')
        if expert is not None and set(expert.classes) != {'cha','sang'}:
            raise ValueError('expert must contain exactly cha and sang (without side labels)')
        self.base, self.expert = base, expert
        self.classes, self.size = list(base.classes), base.size
        self.threshold, self.expert_strength, self.colour_strength = threshold, expert_strength, colour_strength
        self.index = {label:i for i,label in enumerate(self.classes)}
        self.last_diagnostics = []

    @property
    def path(self):
        """Keep the main checkpoint interface used by session/count-log metadata."""
        return self.base.path

    @staticmethod
    def _distribution(value, length):
        p = np.asarray(value, float)
        if p.shape != (length,) or not np.isfinite(p).all() or (p < 0).any() or p.sum() <= 0:
            raise ValueError('invalid classifier probability distribution')
        return p/p.sum()

    def predict_proba(self, images):
        if not images:
            self.last_diagnostics = []
            return []
        raw = self.base.predict_proba(images)
        if len(raw) != len(images):
            raise ValueError('base classifier result count differs from crops')
        probabilities = [self._distribution(p,len(self.classes)).copy() for p in raw]
        diagnostics, selected = [], []
        complete_pair = all(f'{s}_{k}' in self.index for s in ('blue','red') for k in ('cha','sang'))
        for row,p in enumerate(probabilities):
            kinds = {}
            for label,value in zip(self.classes,p):
                kind = label.split('_',1)[1]
                kinds[kind] = kinds.get(kind,0.)+value
            winner = max(kinds, key=kinds.get)
            eligible = (self.expert is not None and self.expert_strength > 0 and complete_pair
                        and winner in ('cha','sang') and kinds[winner] < self.threshold
                        and kinds.get('cha',0.)+kinds.get('sang',0.) >= .5)
            if eligible:
                selected.append(row)
            diagnostics.append(dict(expert_applied=False, initial_kind=winner,
                                    initial_kind_confidence=float(kinds[winner])))
        if selected:
            # One additional batched forward pass only for uncertain cha/sang crops.
            expert_images = [images[i].resize((self.expert.size,self.expert.size), Image.Resampling.BICUBIC) for i in selected]
            output = self.expert.predict_proba(expert_images)
            if len(output) != len(selected):
                raise ValueError('expert result count differs from selected crops')
            order = [self.expert.classes.index(k) for k in ('cha','sang')]
            for row,raw_expert in zip(selected,output):
                p = probabilities[row]
                expert = self._distribution(raw_expert,2)[order]
                prior = np.array([sum(p[self.index[f'{s}_{k}']] for s in ('blue','red')) for k in ('cha','sang')])
                prior /= prior.sum()
                fused = np.exp((1-self.expert_strength)*np.log(np.maximum(prior,1e-12))
                               + self.expert_strength*np.log(np.maximum(expert,1e-12)))
                fused /= fused.sum()
                for side in ('blue','red'):
                    indices = [self.index[f'{side}_{k}'] for k in ('cha','sang')]
                    p[indices] = p[indices].sum()*fused
                diagnostics[row]['expert_applied'] = True
        for row,(image,p) in enumerate(zip(images,probabilities)):
            evidence = glyph_colour_evidence(image) if self.colour_strength else dict(red_fraction=.5,reliability=0.,pixels=0)
            diagnostics[row]['colour'] = evidence
            signal = self.colour_strength*evidence['reliability']*(2*evidence['red_fraction']-1)
            for kind in {label.split('_',1)[1] for label in self.classes}:
                names = [f'{s}_{kind}' for s in ('blue','red')]
                if not all(name in self.index for name in names):
                    continue
                indices = [self.index[name] for name in names]
                mass = p[indices].sum()
                adjusted = p[indices]*np.exp([-signal,signal])
                if adjusted.sum() > 0:
                    p[indices] = mass*adjusted/adjusted.sum()
        self.last_diagnostics = diagnostics
        return [p.tolist() for p in probabilities]

    def predict(self, images):
        return [(self.classes[int(np.argmax(p))],float(max(p))) for p in self.predict_proba(images)]
