"""Package the current source and model files into a single ZIP archive."""
import hashlib
import json
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

ROOT = Path(__file__).resolve().parent


def package():
    out = ROOT / 'output'
    validation = json.loads((out / 'validation.json').read_text())
    assert validation['status'] == 'PASS', validation['failures']
    for group in ('input_sha256', 'geometry_sha256'):
        for name, digest in validation[group].items():
            assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == digest, 'Validation is stale: ' + name
    assert all(validation['front_revision']['unchanged_export_sha256'].values())

    source_names = (
        'README.md', 'build.py', 'validate.py', 'mesh_quality.py', 'preview.py',
        'drawings.py', 'package.py', 'viewer.template.html', 'parameters.json',
        'compatibility-baseline.json', 'sources.json', 'requirements.txt',
        'requirements.lock.txt', 'qa/check-viewer.mjs',
    )
    output_names = (
        'assembly.step', 'dimensions.json', 'parts.json', 'capsule-v01-drawings.pdf',
        'capsule-3d.html', 'cad-preview.png', 'cad-construction.png',
        'cad-front-joint.png', 'cad-skirts-and-roots.png', 'cad-surface-details.png',
    )
    files = [ROOT / name for name in source_names]
    files.extend(out / name for name in output_names)
    for folder, suffix in (('print_stl', '.stl'), ('stl', '.stl'), ('step', '.step'), ('reference_step', '.step')):
        parts = sorted((out / folder).glob('*' + suffix))
        if folder != 'reference_step':
            assert len(parts) == 8, folder
        else:
            assert parts, folder
        files.extend(parts)
    assert all(path.is_file() for path in files), 'Missing a delivery file'

    manifest = [
        {'path': path.relative_to(ROOT).as_posix(), 'bytes': path.stat().st_size,
         'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
        for path in files
    ]
    manifest_path = out / 'manifest.json'
    manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')
    project = ROOT.parents[1]
    legal = [project/'LICENSE', project/'THIRD_PARTY_NOTICES.md', *sorted((project/'LICENSES').glob('*'))]
    archive = ROOT / 'ldg-capsule-v01.zip'
    with ZipFile(archive, 'w', ZIP_DEFLATED, compresslevel=6) as zipped:
        for path in files + [manifest_path]:
            zipped.write(path, arcname='capsule-v01/' + path.relative_to(ROOT).as_posix())
        for path in legal:
            zipped.write(path, arcname='capsule-v01/' + path.relative_to(project).as_posix())
    with ZipFile(archive) as zipped:
        assert zipped.testzip() is None
        for entry in manifest:
            assert hashlib.sha256(zipped.read('capsule-v01/' + entry['path'])).hexdigest() == entry['sha256']
    print(json.dumps({'archive': str(archive), 'files': len(files) + 1, 'bytes': archive.stat().st_size}, indent=2))


if __name__ == '__main__':
    package()
