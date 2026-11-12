from types import SimpleNamespace
from pathlib import Path
import sys
from PIL import Image, ImageDraw
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from piece_detection.refinement import RefinedClassifier, glyph_colour_evidence

CLASSES=['blue_cha','blue_sang','red_cha','red_sang','blue_po','red_po']


def base(distribution):
    return SimpleNamespace(classes=CLASSES,size=64,
                           predict_proba=lambda images:[list(distribution) for _ in images])


def glyph(colour):
    image=Image.new('RGB',(64,64),'white')
    ImageDraw.Draw(image).rectangle((26,18,38,46),fill=colour)
    return image


def test_uncertain_pair_only_is_batched_and_other_kind_mass_is_preserved():
    calls=[]
    specialist=SimpleNamespace(classes=['cha','sang'],size=64,
                               predict_proba=lambda images:calls.append(len(images)) or [[.01,.99] for _ in images])
    model=RefinedClassifier(base([.35,.15,.35,.05,.05,.05]),specialist,colour_strength=0)
    p=np.array(model.predict_proba([glyph('white')]*2))
    assert calls==[2]
    assert np.allclose(p.sum(axis=1),1)
    assert p[0,1]>p[0,0] and p[0,3]>p[0,2]
    assert p[0,4:].tolist()==pytest.approx([.05,.05])
    assert p[0,:2].sum()==pytest.approx(.5)
    assert p[0,2:4].sum()==pytest.approx(.4)


@pytest.mark.parametrize('distribution', [[.9,.01,.01,.01,.04,.03], [.05,.05,.05,.05,.7,.1]])
def test_confident_pair_and_nonpair_do_not_call_expert(distribution):
    specialist=SimpleNamespace(classes=['cha','sang'],size=64,
                               predict_proba=lambda images:pytest.fail('expert must not run'))
    model=RefinedClassifier(base(distribution),specialist,colour_strength=0)
    assert model.predict_proba([glyph('white')])[0]==pytest.approx(distribution)


@pytest.mark.parametrize('colour,side',[('green','blue'),('blue','blue'),('red','red')])
def test_colour_adjusts_side_but_preserves_kind_probability(colour,side):
    model=RefinedClassifier(base([.2,.15,.2,.15,.15,.15]))
    p=model.predict_proba([glyph(colour)])[0]
    offset=0 if side=='blue' else 2
    assert p[offset]>p[2-offset]
    assert p[0]+p[2]==pytest.approx(.4)
    assert p[1]+p[3]==pytest.approx(.3)
    assert p[4]+p[5]==pytest.approx(.3)
    assert sum(p)==pytest.approx(1)


def test_no_colour_and_wood_background_are_neutral():
    for colour in ['white','gray',(180,140,90)]:
        assert glyph_colour_evidence(glyph(colour))['reliability']==0
    image=Image.new('RGB',(64,64),'red')
    ImageDraw.Draw(image).ellipse((5,5,59,59),fill='white')
    assert glyph_colour_evidence(image)['reliability']==0


def test_bad_distributions_and_wrong_expert_labels_fail():
    with pytest.raises(ValueError,match='probability'):
        RefinedClassifier(base([float('nan')]*6)).predict_proba([glyph('red')])
    with pytest.raises(ValueError,match='exactly'):
        RefinedClassifier(base([1,0,0,0,0,0]),SimpleNamespace(classes=['blue_cha','blue_sang']))
    assert RefinedClassifier(base([1,0,0,0,0,0])).predict_proba([])==[]
