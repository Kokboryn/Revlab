#!/usr/bin/env python3
"""Plot a Revlab telemetry CSV.

usage:  tools/plot.py [run.csv] [-o out.png] [--show]

Panels are chosen from whichever columns are present, so this keeps working when the logger's column list changes. """
import argparse, csv, os, sys

RAD_S_TO_RPM = 60.0 / (2.0 * 3.141592653589793)

C_TRUE  = '#c1440e'   # ground truth
C_CRANK = '#2a628f'
C_CAM   = '#3f7d20'
C_TRQ   = '#7d3f9c'
C_FUEL  = '#b8860b'

DTC_LABEL = {0: 'passed', 1: 'pending', 2: 'confirmed'}


def load(path):
    with open(path, newline='') as f:
        rdr = csv.DictReader(f)
        names = rdr.fieldnames
        rows = [r for r in rdr
                if all(r.get(k) not in (None, '') for k in names)]
    if not rows:
        sys.exit(f'{path}: no complete data rows')
    cols = {k: [float(r[k]) for r in rows] for k in names}
    if 'omega' in cols:
        cols['rpm'] = [v * RAD_S_TO_RPM for v in cols['omega']]
    return cols


def dtc_spans(t, dtc):
    """Contiguous [start, end, state] runs, skipping state 0."""
    out, start, cur = [], t[0], dtc[0]
    for ti, d in zip(t, dtc):
        if d != cur:
            if cur:
                out.append((start, ti, cur))
            start, cur = ti, d
    if cur:
        out.append((start, t[-1], cur))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', nargs='?', default='run.csv')
    ap.add_argument('-o', '--out', default=None)
    ap.add_argument('--title', default=None)
    ap.add_argument('--show', action='store_true')
    a = ap.parse_args()

    import matplotlib
    if not a.show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    c = load(a.csv)
    t = c['t_s']

    panels = []
    if 'rpm' in c:
        panels.append('speed')
    if 't_arb' in c:
        panels.append('torque')
    if 'q_cmd' in c:
        panels.append('fuel')
    if not panels:
        sys.exit('nothing plottable: need omega, t_arb or q_cmd')

    heights = {'speed': 2.4, 'torque': 1.0, 'fuel': 1.0}
    fig, axes = plt.subplots(
        len(panels), 1, sharex=True,
        figsize=(10, 1.7 * sum(heights[p] for p in panels)),
        gridspec_kw={'height_ratios': [heights[p] for p in panels]})
    if len(panels) == 1:
        axes = [axes]
    ax = dict(zip(panels, axes))

    # --- DTC shading across every panel, so faults line up visually
    spans = dtc_spans(t, c['dtc']) if 'dtc' in c else []
    for s, e, state in spans:
        col = '#f4c542' if state == 1 else '#c1440e'
        for axis in axes:
            axis.axvspan(s, e, color=col,
                         alpha=0.10 if state == 2 else 0.20, lw=0)

    # --- speed
    if 'speed' in ax:
        p = ax['speed']
        if 'n_crank' in c:
            p.plot(t, c['n_crank'], lw=0.9, color=C_CRANK, alpha=0.9,
                   label='crank sensor')
        if 'n_cam' in c:
            p.plot(t, c['n_cam'], lw=0.9, color=C_CAM, alpha=0.85,
                   label='cam sensor')
        p.plot(t, c['rpm'], lw=4.0, color=C_TRUE, alpha=0.30, label='true speed (plant)', zorder=1)
        p.set_ylabel('engine speed\n[rpm]')
        p.legend(loc='best', fontsize=8.5, framealpha=0.95, ncols=3)
        for s, _, state in spans:
            p.annotate(f'P0016 {DTC_LABEL.get(int(state), state)}',
                       xy=(s, p.get_ylim()[1]), xytext=(-3, -6),
                       textcoords='offset points', rotation=90,
                       fontsize=7.5, va='top', ha='right', color='0.25')

    # --- torque
    if 'torque' in ax:
        p = ax['torque']
        p.plot(t, c['t_arb'], lw=1.2, color=C_TRQ)
        p.set_ylabel('ECU torque\n[Nm]')

    # --- fuel
    if 'fuel' in ax:
        p = ax['fuel']
        p.plot(t, c['q_cmd'], lw=1.2, color=C_FUEL)
        p.set_ylabel('fuel cmd\n[mg/stroke]')

    for axis in axes:
        axis.grid(alpha=0.22)
        axis.margins(x=0)
    axes[-1].set_xlabel('simulation time [s]')

    title = a.title or f'Revlab — {os.path.basename(a.csv)}'
    axes[0].set_title(title, fontsize=11)
    fig.tight_layout()

    out = a.out or os.path.splitext(a.csv)[0] + '.png'
    fig.savefig(out, dpi=150)
    print(f'wrote {out}')
    if a.show:
        plt.show()


if __name__ == '__main__':
    main()