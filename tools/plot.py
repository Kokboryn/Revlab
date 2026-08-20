#!/usr/bin/env python3
"""Plot a Revlab telemetry CSV.

usage:  tools/plot.py [run.csv] [-o out.png] [--show]

Panels are derived from  the CSV's columns. Known columns are grouped and scaled via SPEC; anything unrecognized gets its
own panel, so new logger columns appear without editing this file."""
import argparse, csv, os, sys

RPM = 60.0 / (2.0 * 3.141592653589793)
K2C = lambda v: v - 273.15
PA2KPA = lambda v: v / 1000.0

# column -> (panel, label, transform, color, twin?)
SPEC = {
    'omega':        ('speed',   'true speed (plant)',   lambda v: v * RPM,    '#c1440e',  False),
    'n_crank':      ('speed',   'crank sensor',         None,                           '#2a628f',  False),
    'n_cam':        ('speed',   'cam sensor',           None,                           '#3f7d20',  False),
    'n_model':      ('speed',   'ECU model',            None,                           '#9467bd',  False),

    't_arb':        ('torque',  'arbitrated',           None,                           '#7d3f9c',  False),
    't_ind_req':    ('torque',  'indicated req',        None,                           '#c17d0e',  False),
    't_loss':       ('torque',  'losses',               None,                           '#888888',  False),
    't_load':       ('torque',  'external load',        None,                           '#2a628f',  False),

    'q_cmd':        ('fuel',    'commanded',            None,                           '#b8860b',  False),
    'q_lim':        ('fuel',    'smoke limit',          None,                           '#c1440e',  False),

    'p_im':         ('air',     'MAP [kPa]',            PA2KPA,                         '#1f7a8c',  False),
    'p_em':         ('air',     'exhaust [kPa]',        PA2KPA,                         '#c1440e',  False),
    'afr':          ('air',     'AFR',                  None,                           '#8c8c1f',  False),

    'm_air':        ('flow',    'true air [g/s]',       lambda v: v*1000,      '#c1440e', False),
    'm_air_est':    ('flow',    'ECU estimate [g/s]',   lambda v: v*1000,      '#2a628f', False),
    'm_maf_s':      ('flow',    'MAF sensor [g/s]',     lambda v: v*1000,      '#3f7d20', False),

    't_em':         ('temp',    'EGT [C]',              K2C,                            '#c1440e', False),
    't_cool':       ('temp',    'coolant [C]',          K2C,                            '#1f7a8c', False),
    't_oil':        ('temp',    'oil [C]',              K2C,                            '#b8860b', False),

    'n_tc':         ('turbo',   'turbo speed [rpm]',    None,                           '#7d3f9c', False),
    'visc_mult':    ('turbo',   'oil visc mult',        None,                           '#8c8c1f', True),

    'pedal':        ('pedal',   'pedal [%]',            lambda v: v*100,       '#555555', False),
}

PANEL_LABEL = {
    'speed': 'engine speed\n[rpm]', 'torque': 'torque\n[Nm]',
    'fuel': 'fuel\n[mg/stroke]', 'air': 'pressure\n[kPa]',
    'flow': 'air flow\n[g/s]', 'temp': 'temperature\n[C]',
    'turbo': 'turbo', 'pedal': 'pedal [%]',
}

PANEL_HEIGHT = {'speed': 2.4, 'pedal': 0.7}
PANEL_ORDER = ['speed', 'torque', 'fuel', 'air', 'flow', 'temp', 'turbo', 'pedal']

DTC_LABEL = {0: 'passed', 1: 'pending', 2: 'confirmed'}


def load(path):
    with open(path, newline='') as f:
        rdr = csv.DictReader(f)
        names = rdr.fieldnames
        rows = [r for r in rdr if all(r.get(k) not in (None, '') for k in names)]
    if not rows:
        sys.exit(f'{path}: no complete data rows')
    return names, {k: [float(r[k]) for r in rows] for k in names}


def dtc_spans(t, dtc):
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

    names, c = load(a.csv)
    t = c['t_s']

    # --- assign every column to a panel; unknown ones get their own
    panels = {}
    for n in names:
        if n in ('t_s', 'dtc'):
            continue
        if n in SPEC:
            key = SPEC[n][0]
        else:
            key = f'_{n}'           # auto panel for an unrecognized column
        panels.setdefault(key, []).append(n)

    ordered = [k for k in PANEL_ORDER if k in panels]
    ordered += [k for k in panels if k not in PANEL_ORDER]

    heights = [PANEL_HEIGHT.get(k, 1.1) for k in ordered]
    fig, axes = plt.subplots(len(ordered), 1, sharex=True,
                             figsize=(10, 1.7 * sum(heights)),
                             gridspec_kw={'height_ratios': heights})
    if len(ordered) == 1:
        axes = [axes]
    ax = dict(zip(ordered, axes))

    spans = dtc_spans(t, c['dtc']) if 'dtc' in c else []
    for s, e, st in spans:
        col = '#f4c542' if st == 1 else '#c1440e'
        for axis in axes:
            axis.axvspan(s, e, color=col, alpha=0.10 if st == 2 else 0.20, lw=0)

    for key in ordered:
        p = ax[key]
        handles, twin = [], None
        for n in panels[key]:
            panel, label, fn, colour, is_twin = SPEC.get(
                n, (key, n, None, None, False))
            y = [fn(v) for v in c[n]] if fn else c[n]
            target = p
            if is_twin:
                twin = twin or p.twinx()
                target = twin
                twin.set_ylabel(label, color=colour)
            wide = (n == 'omega')
            ln, = target.plot(t, y, lw=4.0 if wide else 1.1, color=colour,
                              alpha=0.30 if wide else 0.9, label=label,
                              zorder=1 if wide else 2)
            handles.append(ln)
        p.set_ylabel(PANEL_LABEL.get(key, key.lstrip('_')))
        if len(handles) > 1:
            p.legend(handles=handles, loc='best', fontsize=8, framealpha=0.95,
                     ncols=min(3, len(handles)))
        p.grid(alpha=0.22)
        p.margins(x=0)

    if 'speed' in ax:
        for s, _, st in spans:
            ax['speed'].annotate(f'P0016 {DTC_LABEL.get(int(st), st)}',
                                 xy=(s, ax['speed'].get_ylim()[1]),
                                 xytext=(-3, -6), textcoords='offset points',
                                 rotation=90, fontsize=7.5, va='top',
                                 ha='right', color='0.25')

    axes[-1].set_xlabel('simulation time [s]')
    axes[0].set_title(a.title or f'Revlab — {os.path.basename(a.csv)}',
                      fontsize=11)
    fig.tight_layout()

    out = a.out or os.path.splitext(a.csv)[0] + '.png'
    fig.savefig(out, dpi=150)
    print(f'wrote {out}')
    if a.show:
        plt.show()


if __name__ == '__main__':
    main()