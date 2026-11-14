"""Main 14-class CNN + conditional pair experts + glyph-colour side evidence.

Same interface as piece_detection.classifier.MobileNetClassifier (classes, size,
path, predict_proba, predict), so the realtime UI can use it unchanged through
tools/cnn2/run_ui.sh. Configured by a bundle json:

{
  "base": "best.pt",
  "experts": [{"path": "expert_cha_sang/best.pt", "classes": ["cha", "sang"],
               "threshold": 0.9, "strength": 0.7}],
  "colour_strength": 1.0
}

Expert rule (per crop): the main model's best kind is in the pair, the pair holds
>= 50% of the kind mass and the best kind is below `threshold`. Then the expert's
2-class probability is fused with the main model's pair split in log space
(`strength` = expert weight) and redistributed inside each side, so an expert can
never change the side. Colour evidence then re-weights blue/red within each kind.
"""
import json, sys
from pathlib import Path
import numpy as np
from PIL import Image
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'janggi'))
from piece_detection.classifier import MobileNetClassifier
from piece_detection.refinement import glyph_colour_evidence

SIDES = ('blue', 'red')


class HierarchicalClassifier:
    def __init__(self, base, experts=(), colour_strength=1.0, path=None):
        self.base, self.experts, self.colour_strength = base, list(experts), float(colour_strength)
        self.classes, self.size = list(base.classes), base.size
        self.index = {c: i for i, c in enumerate(self.classes)}
        self.path = Path(path) if path else base.path
        self.last_diagnostics = []
        for expert, cfg in self.experts:
            if sorted(expert.classes) != sorted(cfg['classes']):
                raise ValueError('expert classes differ from bundle')
            if not all(f'{s}_{k}' in self.index for s in SIDES for k in cfg['classes']):
                raise ValueError('main model lacks an expert pair class')

    @classmethod
    def from_bundle(cls, bundle, device=None, batch_size=32):
        bundle = Path(bundle); cfg = json.loads(bundle.read_text())
        base = MobileNetClassifier(bundle.parent/cfg['base'], device, batch_size)
        experts = [(MobileNetClassifier(bundle.parent/e['path'], device, batch_size, allowed_classes=set(e['classes'])), e)
                   for e in cfg.get('experts', [])]
        return cls(base, experts, cfg.get('colour_strength', 1.0), path=bundle)

    def _kinds(self, p):
        kinds = {}
        for label, v in zip(self.classes, p):
            k = label.split('_', 1)[1]; kinds[k] = kinds.get(k, 0.)+v
        return kinds

    def predict_proba(self, images):
        if not images:
            self.last_diagnostics = []; return []
        probabilities = [np.asarray(p, float)/np.sum(p) for p in self.base.predict_proba(images)]
        diagnostics = [dict(experts=[]) for _ in images]
        for expert, cfg in self.experts:
            pair = list(cfg['classes']); rows = []
            for row, p in enumerate(probabilities):
                kinds = self._kinds(p); best = max(kinds, key=kinds.get)
                if best in pair and kinds[best] < cfg.get('threshold', .9) and sum(kinds[k] for k in pair) >= .5:
                    rows.append(row)
            if not rows:
                continue
            crops = [images[r] if images[r].size == (expert.size, expert.size)
                     else images[r].resize((expert.size, expert.size), Image.Resampling.BICUBIC) for r in rows]
            order = [expert.classes.index(k) for k in pair]; strength = cfg.get('strength', .7)
            for row, raw in zip(rows, expert.predict_proba(crops)):
                p = probabilities[row]; e = np.asarray(raw, float)[order]; e /= e.sum()
                prior = np.array([sum(p[self.index[f'{s}_{k}']] for s in SIDES) for k in pair]); prior /= prior.sum()
                fused = np.exp((1-strength)*np.log(np.maximum(prior, 1e-12))+strength*np.log(np.maximum(e, 1e-12))); fused /= fused.sum()
                for s in SIDES:
                    idx = [self.index[f'{s}_{k}'] for k in pair]; p[idx] = p[idx].sum()*fused
                diagnostics[row]['experts'].append(dict(pair=pair, expert=e.tolist(), prior=prior.tolist(), fused=fused.tolist()))
        if self.colour_strength:
            for row, (image, p) in enumerate(zip(images, probabilities)):
                ev = glyph_colour_evidence(image); diagnostics[row]['colour'] = ev
                signal = self.colour_strength*ev['reliability']*(2*ev['red_fraction']-1)
                for k in {c.split('_', 1)[1] for c in self.classes}:
                    idx = [self.index.get(f'{s}_{k}') for s in SIDES]
                    if None in idx: continue
                    mass = p[idx].sum(); adj = p[idx]*np.exp([-signal, signal])
                    if adj.sum() > 0: p[idx] = mass*adj/adj.sum()
        self.last_diagnostics = diagnostics
        return [p.tolist() for p in probabilities]

    def predict(self, images):
        return [(self.classes[int(np.argmax(p))], float(max(p))) for p in self.predict_proba(images)]
