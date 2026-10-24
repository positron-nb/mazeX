"""The exploration package must never access ground-truth maze information.

Ground truth lives in mazemind_simulation (maze generator, world files,
JSON) and is exposed on /ground_truth/* topics for the evaluator only.
This test fails if runtime exploration code references any of it.
"""

import pathlib
import re

PKG = pathlib.Path(__file__).resolve().parents[1] / 'mazemind_exploration'

FORBIDDEN = [
    r'ground_truth',
    r'mazemind_simulation',
    r'mazemind_evaluation',
    r'\.json[\'"]',             # loading a maze file
    r'\.sdf',
    r'/world/',                 # Gazebo world/model state topics
    r'/model/',
    r'gz\.msgs|ignition\.msgs',
]


def test_exploration_sources_do_not_reference_ground_truth():
    offenders = []
    for path in sorted(PKG.rglob('*.py')):
        text = path.read_text(encoding='utf-8')
        # docstrings may *mention* the boundary; only check code lines
        code = '\n'.join(line for line in text.splitlines()
                         if not line.strip().startswith('#'))
        code = re.sub(r'"""[\s\S]*?"""', '', code)
        for pattern in FORBIDDEN:
            if re.search(pattern, code):
                offenders.append(f'{path.name}: {pattern}')
    assert not offenders, f'ground-truth access in exploration code: {offenders}'


def test_exploration_parameters_do_not_include_ground_truth():
    cfg = PKG.parents[1] / 'mazemind_bringup' / 'config' / 'exploration.yaml'
    if cfg.exists():
        assert 'ground_truth' not in cfg.read_text(encoding='utf-8')
