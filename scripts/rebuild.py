"""Rebuild CAD, the ROS description and the preview; optionally create a ZIP."""
import argparse
import shutil
import subprocess
import sys
import zipfile

from project_paths import ROOT, OUT, PKG, DIST


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', action='store_true', help='Also create dist/robotiq_digit_description.zip')
    parser.add_argument('--skip-preview', action='store_true', help='Skip rendering; retain any existing preview')
    args = parser.parse_args()
    if not __debug__:
        parser.error('Run without -O: the CAD and URDF validation assertions must remain enabled.')

    steps = ['build_assembly.py', 'package_robot.py']
    if not args.skip_preview:
        steps.append('render_preview.py')
    for name in steps:
        print(f'Running {name} ...', flush=True)
        subprocess.run([sys.executable, str(ROOT / 'scripts' / name)], cwd=ROOT, check=True)

    preview = OUT / 'assembly_preview.png'
    if preview.is_file():
        shutil.copy2(preview, PKG / preview.name)
    if args.skip_preview:
        print('Preview rendering skipped; any existing preview was retained.', flush=True)

    if args.archive:
        DIST.mkdir(exist_ok=True)
        destination = DIST / 'robotiq_digit_description.zip'
        temporary = destination.with_suffix('.zip.tmp')
        with zipfile.ZipFile(temporary, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
            for path in sorted(PKG.rglob('*')):
                if path.is_file() and '__pycache__' not in path.parts and path.suffix not in ('.pyc', '.pyo'):
                    archive.write(path, path.relative_to(ROOT))
        with zipfile.ZipFile(temporary) as archive:
            assert archive.testzip() is None
        temporary.replace(destination)
        print(f'Archive: {destination.relative_to(ROOT)}', flush=True)
    print('Rebuild and validation complete.', flush=True)


if __name__ == '__main__':
    main()
