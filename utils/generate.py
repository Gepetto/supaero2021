#!/usr/bin/env python3
"""Cut python files in bits loadable by ipython."""

from pathlib import Path
import json

hashtags = ['jupyter_snippet']
#hashtags = [ 'do_load', 'do_not_load', 'load' ]

def generate(tp_number: int):
    tp = Path() / f'tp{tp_number}'
    ipynb = next(Path().glob(f'{tp_number}_*.ipynb'))
    print(f'processing {ipynb} & {tp}')
    with ipynb.open() as f:
        data = json.load(f)
    cells_copy = data['cells'].copy()
    generated = tp / 'generated'
    generated.mkdir(exist_ok=True)
    for filename in tp.glob('*.py'):
        print(f' processing {filename}')
        content = []
        hidden = False
        dest = None
        with filename.open() as f_in:
            for line_number, line in enumerate(f_in):
                if any([ line.startswith(f'# %{hashtag}') for hashtag in hashtags ]):
                    if dest is not None:
                        raise SyntaxError(f'%{hashtags[0]} block open twice at line {line_number + 1}')
                    dest = generated / f'{filename.stem}_{line.split()[2]}'
                    hidden = False
                elif any([ line.strip() == f'# %end_{hashtag}' for hashtag in hashtags ]):
                    if dest is None:
                        raise SyntaxError(f'%{hashtags[0]} block before open at line {line_number + 1}')
                    with dest.open('w') as f_out:
                        f_out.write(''.join(content))
                    for cell_number, cell in enumerate(cells_copy):
                        if cell['source'][0].endswith(f'%load {dest}'):
                            data['cells'][cell_number]['source'] = [f'# %load {dest}\n'] + content
                        #if f'%do_not_load {dest}' in cell['source'][0]:
                        #    data['cells'][cell_number]['source'] = [f'%do_not_load {dest}\n']
                    content = []
                    hidden = False
                    dest = None
                elif dest is not None:
                    content.append(line)
    with ipynb.open('w') as f:
        f.write(json.dumps(data, indent=1))


if __name__ == '__main__':
    for tp_number in [0,2]:
        generate(tp_number)
