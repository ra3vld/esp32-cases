"""Build a compact GitHub Pages site from the current offline CAD viewers."""
import argparse
import gzip
import json
from pathlib import Path
import re
import shutil

ROOT = Path(__file__).resolve().parents[1]
MODELS = {
    'crowpanel': ROOT/'crowpanel/mechanical/capsule-v03/output',
    'esp32-3248s035c': ROOT/'esp32-3248s035c/capsule-v01/output',
}


def build(destination):
    destination = destination.resolve()
    destination.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ROOT/'site/index.html', destination/'index.html')
    (destination/'.nojekyll').write_text('')
    for path in [ROOT/'LICENSE',ROOT/'THIRD_PARTY_NOTICES.md',*sorted((ROOT/'LICENSES').glob('*'))]:
        target=destination/path.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,target)
    assets = destination/'assets'; assets.mkdir(exist_ok=True)
    sizes = {}
    for name, output in MODELS.items():
        folder = destination/name; folder.mkdir(exist_ok=True)
        document = (output/'capsule-3d.html').read_text()
        match = re.search(r'<script type="application/json" id="mesh-data">(.*?)</script>', document, re.S)
        if match is None:
            raise ValueError('Missing mesh payload: '+name)
        payload = match.group(1).encode()
        json.loads(payload)
        (folder/'meshes.json.gz').write_bytes(gzip.compress(payload, compresslevel=9, mtime=0))
        document = document[:match.start()]+document[match.end():]
        document = document.replace('(()=>{', '(async()=>{', 1)
        loader = """const response=await fetch('meshes.json.gz');
if(!response.ok)throw new Error('HTTP '+response.status);
if(!('DecompressionStream' in window))throw new Error('Для 3D-просмотра нужен современный браузер.');
const compressed=await response.arrayBuffer();
const stream=new Blob([compressed]).stream().pipeThrough(new DecompressionStream('gzip'));
const meshes=JSON.parse(await new Response(stream).text()),canvas="""
        document, count = re.subn(r'const meshes=JSON.parse\(document.getElementById\([\'"]mesh-data[\'"]\).textContent\),canvas=', loader, document, count=1)
        if count != 1:
            raise ValueError('Viewer loader not found: '+name)
        document = document.replace('<main>', '<main><a href="../" style="display:inline-block;margin-bottom:16px;color:inherit">← Все корпуса</a>', 1)
        # Show a useful message if a browser cannot load the model.
        document = document.replace('})();', '})().catch(error=>{const c=document.querySelector("canvas");if(c){const p=document.createElement("p");p.textContent="Не удалось загрузить модель: "+error.message;c.replaceWith(p);}console.error(error);});', 1)
        (folder/'index.html').write_text(document)
        shutil.copy2(output/'cad-preview.png', assets/(name+'.png'))
        sizes[name] = {'mesh_gzip_bytes':(folder/'meshes.json.gz').stat().st_size,
                       'viewer_bytes':(folder/'index.html').stat().st_size}
    shutil.copy2(ROOT/'crowpanel/photos/capsule-assembled.jpg', assets/'crowpanel-assembled.jpg')
    print(json.dumps({'destination':str(destination),'models':sizes}, indent=2))


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,default=ROOT/'.pages-dist')
    build(parser.parse_args().output)
