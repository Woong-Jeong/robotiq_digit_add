"""Repository paths, independent of the caller's working directory."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
INPUT = ROOT / 'input'
OUT = ROOT / 'output'
INTERMEDIATE = OUT / 'intermediate'
ALIGNED = INTERMEDIATE / 'aligned_parts_mm'
PKG = ROOT / 'robotiq_digit_description'
UPSTREAM = ROOT / 'third_party' / 'tacto'
DIST = ROOT / 'dist'
