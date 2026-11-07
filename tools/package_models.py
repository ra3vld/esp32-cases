"""Create the standalone STEP/STL release and its integrity manifest."""
import argparse
import hashlib
import json
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

ROOT=Path(__file__).resolve().parents[1]
MODELS={'crowpanel-0.3':ROOT/'crowpanel/mechanical/capsule-v03',
        'esp32-3248s035c-0.1':ROOT/'esp32-3248s035c/capsule-v01'}


def package(version, destination):
    destination.mkdir(parents=True,exist_ok=True)
    files={}
    for name,folder in MODELS.items():
        out=folder/'output'
        report=json.loads((out/'validation.json').read_text())
        assert report.get('all_checks_passed') or report.get('status')=='PASS',name
        if name=='crowpanel-0.3':
            quality=json.loads((out/'mesh-validation.json').read_text())
            assert quality['all_checks_passed']
            hashes={**quality['geometry_step_sha256'],**quality['file_sha256']}
            for rel,digest in hashes.items():assert hashlib.sha256((out/rel).read_bytes()).hexdigest()==digest,rel
        else:
            for group in ('input_sha256','geometry_sha256'):
                for rel,digest in report[group].items():assert hashlib.sha256((folder/rel).read_bytes()).hexdigest()==digest,rel
        for sub,suffix in (('step','.step'),('stl','.stl'),('print_stl','.stl')):
            parts=sorted((out/sub).glob('*'+suffix))
            assert len(parts)==(7 if name=='crowpanel-0.3' else 8),(name,sub)
            for p in parts:files[f'{name}/{sub}/{p.name}']=p
        files[f'{name}/assembly.step']=out/'assembly.step'
        if name=='crowpanel-0.3':files[f'{name}/crowpanel-module-simplified.step']=out/'crowpanel-module-simplified.step'
    for p in [ROOT/'LICENSE',ROOT/'THIRD_PARTY_NOTICES.md',*sorted((ROOT/'LICENSES').glob('*'))]:
        files[p.relative_to(ROOT).as_posix()]=p
    manifest=[{'path':name,'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for name,p in sorted(files.items())]
    target=destination/f'esp32-cases-step-stl-{version}.zip'
    with ZipFile(target,'w',ZIP_DEFLATED,compresslevel=9) as zipped:
        for name,p in sorted(files.items()):zipped.write(p,name)
        zipped.writestr('manifest.json',json.dumps(manifest,indent=2)+'\n')
        zipped.writestr('README.txt','ESP32 Cases '+version+'\n\nprint_stl/: millimetres, 100% scale, oriented for printing.\nstl/: assembly-coordinate meshes. step/: editable CAD parts.\nassembly.step: assembled enclosure and reference components.\nThe CrowPanel module is an independently generated simplified reference.\n\nAssembly instructions and full source/model packages:\nhttps://github.com/ra3vld/esp32-cases\nhttps://github.com/ra3vld/esp32-cases/releases/tag/'+version+'\n\nProject CAD models: CC BY 4.0. Code: MIT. See LICENSES/ and THIRD_PARTY_NOTICES.md.\n')
    with ZipFile(target) as zipped:
        assert zipped.testzip() is None
        for row in manifest:assert hashlib.sha256(zipped.read(row['path'])).hexdigest()==row['sha256']
    print(json.dumps({'archive':str(target),'files':len(manifest)+2,'bytes':target.stat().st_size},indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--version',default='v1.0.0');parser.add_argument('--output',type=Path,default=ROOT/'release-dist')
    args=parser.parse_args();package(args.version,args.output)
