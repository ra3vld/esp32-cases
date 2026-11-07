"""Package only the verified delivery set, excluding intermediate diagnostics."""
import hashlib
import json
from pathlib import Path
from zipfile import ZIP_DEFLATED,ZipFile

ROOT=Path(__file__).resolve().parent
out=ROOT/'output'
report=json.loads((out/'validation.json').read_text())
if not report.get('all_checks_passed'):
    raise SystemExit('Refusing to package: CAD validation has not passed')
quality=json.loads((out/'mesh-validation.json').read_text())
if not quality.get('all_checks_passed'):raise SystemExit('Mesh quality checks have not passed')
for path,sha in {**quality['geometry_step_sha256'],**quality['file_sha256']}.items():
    if hashlib.sha256((out/path).read_bytes()).hexdigest()!=sha:raise SystemExit(f'Stale mesh validation: {path}')
files=[ROOT/n for n in ('README.md','build.py','module_reference.py','mesh_quality.py','refine_mesh.py','preview.py','package.py','viewer.template.html','parameters.json','requirements.txt')]
for name,digest in report['input_sha256'].items():
    if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest:raise SystemExit(f'Stale source validation: {name}')
for folder in ('print_stl','stl','step'):
    files.extend(sorted((out/folder).glob('*')))
files.extend(out/n for n in ('derived.json','assembly.step','crowpanel-module-simplified.step','cad-preview.png','cad-construction.png','reinforcement-preview.png','capsule-3d.html'))
if not all(p.is_file() for p in files):raise SystemExit('Missing a delivery file')
manifest={str(p.relative_to(ROOT)):{'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in files}
(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
files.append(out/'manifest.json')
project=ROOT.parents[2]
legal=[project/'LICENSE',project/'THIRD_PARTY_NOTICES.md',*sorted((project/'LICENSES').glob('*'))]
target=ROOT/'crowpanel-capsule-0.3.zip'
with ZipFile(target,'w',ZIP_DEFLATED) as z:
    for p in files:z.write(p,Path('capsule-0.3')/p.relative_to(ROOT))
    for p in legal:z.write(p,Path('capsule-0.3')/p.relative_to(project))
with ZipFile(target) as z:
    assert z.testzip() is None
    assert len(z.namelist())==len(files)+len(legal)
print(target)
print('Files:',len(files),'archive bytes:',target.stat().st_size)
