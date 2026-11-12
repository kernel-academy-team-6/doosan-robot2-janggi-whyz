"""Compare main CNN, conditional expert and glyph colour on existing held-out splits."""
import argparse
import json
from pathlib import Path
import sys
from PIL import Image
import numpy as np
import torch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'janggi'))
from piece_detection.classifier import MobileNetClassifier, DEFAULT_CLASSIFIER
from piece_detection.refinement import RefinedClassifier, DEFAULT_EXPERT
from train_piece_classifier import metrics_from_confusion, load_datasets


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device',default='cpu')
    args=parser.parse_args()
    torch.set_num_threads(4)
    dataset=ROOT/'tools/datasets/janggi_pieces2/exports/224'
    datasets,info=load_datasets(dataset,224)
    base=MobileNetClassifier(DEFAULT_CLASSIFIER,args.device)
    expert=MobileNetClassifier(DEFAULT_EXPERT,args.device,allowed_classes={'cha','sang'})
    refined=RefinedClassifier(base,expert)
    result=dict(dataset=str(dataset), threshold=refined.threshold,
                expert_strength=refined.expert_strength,colour_strength=refined.colour_strength,splits={})
    for split in ['val','test']:
        files=datasets[split].samples
        original,adjusted,diagnostics=[],[],[]
        for start in range(0,len(files),32):
            images=[]
            for filename,_ in files[start:start+32]:
                with Image.open(filename) as source:
                    images.append(source.convert('RGB'))
            original.extend(base.predict_proba(images))
            adjusted.extend(refined.predict_proba(images))
            diagnostics.extend(refined.last_diagnostics)
        reports={}
        for name,probabilities in [('base',original),('refined',adjusted)]:
            matrix=np.zeros((len(base.classes),len(base.classes)),dtype=int)
            side_correct=kind_correct=0
            for (_,truth),p in zip(files,probabilities):
                guess=int(np.argmax(p));matrix[truth,guess]+=1
                truth_side,truth_kind=base.classes[truth].split('_',1)
                guess_side,guess_kind=base.classes[guess].split('_',1)
                side_correct+=truth_side==guess_side;kind_correct+=truth_kind==guess_kind
            reports[name]=metrics_from_confusion(matrix,base.classes)
            reports[name].update(side_accuracy=side_correct/len(files),kind_accuracy=kind_correct/len(files))
        reports['expert_calls']=sum(d['expert_applied'] for d in diagnostics)
        reports['colour_evidence_crops']=sum(d['colour']['reliability']>0 for d in diagnostics)
        reports['changes']=[dict(file=str(Path(file).relative_to(dataset)),truth=base.classes[truth],
                                 before=base.classes[int(np.argmax(a))],after=base.classes[int(np.argmax(b))])
                            for (file,truth),a,b in zip(files,original,adjusted) if np.argmax(a)!=np.argmax(b)]
        result['splits'][split]=reports
        print(split,{k:dict(accuracy=v['accuracy'],side_accuracy=v['side_accuracy'],kind_accuracy=v['kind_accuracy'])
                     for k,v in reports.items() if k in ('base','refined')},
              'expert_calls',reports['expert_calls'],flush=True)
    output=ROOT/'models/piece_classifier_cha_sang_224/pipeline_metrics.json'
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(output)


if __name__=='__main__':
    main()
