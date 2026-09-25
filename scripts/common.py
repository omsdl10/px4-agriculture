from pathlib import Path
import math
import yaml
ROOT = Path(__file__).resolve().parents[1]
def config(name):
    return yaml.safe_load((ROOT / 'config' / f'{name}.yaml').read_text())
def elevation(x, y, cfg=None):
    cfg = cfg or config('simulation')
    scale = cfg['map_size_m'] / 1000
    x, y = x / scale, y / scale
    # Smooth grading leaves a level launch zone and central farm lanes.
    ramp = min(1., max(0., (max(abs(x), abs(y)) - 25) / 70))
    return cfg['terrain_amplitude_m'] * ramp * (0.5 + 0.25 * math.sin(x/100) + 0.25 * math.sin(y/140))
