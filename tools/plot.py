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
    # --- engine
    'omega':        ('speed',   'true speed (plant)',           lambda v: v * RPM,  '#c1440e',  False),
    'n_crank':      ('speed',   'crank sensor',                 None,               '#2a628f',  False),
    'n_cam':        ('speed',   'cam sensor',                   None,               '#3f7d20',  False),
    'n_model':      ('speed',   'ECU model',                    None,               '#9467bd',  False),
    't_arb':        ('torque',  'arbitrated',                   None,               '#7d3f9c',  False),
    't_ind_req':    ('torque',  'indicated req',                None,               '#c17d0e',  False),
    't_loss':       ('torque',  'losses',                       None,               '#888888',  False),
    't_load':       ('torque',  'external load',                None,               '#2a628f',  False),
    't_clutch':     ('torque',  'clutch reaction',              None,               '#3f7d20',  False),
    'q_cmd':        ('fuel',    'commanded',                    None,               '#b8860b',  False),
    'q_lim':        ('fuel',    'smoke limit',                  None,               '#c1440e',  False),
    'pedal':        ('pedal',   'pedal [%]',                    lambda v: v * 100,  '#555555',  False),

    # --- airpath
    'p_im':         ('air',     'MAP [kPa]',                    PA2KPA,             '#1f7a8c',  False),
    'p_em':         ('air',     'exhaust [kPa]',                PA2KPA,             '#c1440e',  False),
    'afr':          ('afr',     'AFR',                          None,               '#8c8c1f',  False),
    'm_air':        ('flow',    'true air [g/s]',               lambda v: v*1000,   '#c1440e',  False),
    'm_air_est':    ('flow',    'ECU estimate [g/s]',           lambda v: v*1000,   '#2a628f',  False),
    'm_maf_s':      ('flow',    'MAF sensor [g/s]',             lambda v: v*1000,   '#3f7d20',  False),
    'n_tc':         ('turbo',   'turbo speed [rpm]',            None,               '#7d3f9c',  False),

    # --- thermal
    't_em':         ('egt',     'EGT [C]',                      K2C,                '#c1440e',  False),
    't_cool':       ('temp',    'coolant [C]',                  K2C,                '#1f7a8c',  False),
    't_ect':        ('temp',    'ECT sensor [C]',               K2C,                '#2a628f',  False),
    't_oil':        ('temp',    'oil [C]',                      K2C,                '#b8860b',  False),
    'visc_mult':    ('temp',    'oil visc mult',                None,               '#8c8c1f',  True),
    'q_coolant':    ('heat',    'to coolant [kW]',              lambda v: v / 1000, '#1f7a8c',  False),
    'q_fric':       ('heat',    'friction [kW]',                lambda v: v / 1000, '#b8860b',  False),

    # --- diagnostics
    'freeze':       ('flags',       'freeze',                   None,               '#7d3f9c',  False),
    'crank_valid':  ('flags',       'crank valid',              None,               '#2a628f',  False),
    'cam_valid':    ('flags',       'cam valid',                None,               '#3f7d20',  False),
    'speed_source': ('flags',       'on cam',                   None,               '#c1440e',  False),
    'overheat':     ('flags',       'clutch overheat',          None,               '#c17d0e',  False),
    't_bias':       ('bias',        'observer bias [Nm]',       None,               '#7d3f9c',  False),
    'eta_ind':      ('eta',         'indicated efficiency',     None,               '#1f7a8c',  False),

    # --- driveline
    'v_veh':        ('vspeed',      'vehicle [km/h]',           lambda v: v * 3.6,  '#c1440e',  False),
    'n_wheel':      ('vspeed',      'wheel [rpm]',              None,               '#888888',  True),
    'omega_in':     ('shafts',      'input shaft (plant)',      lambda v: v * RPM,  '#c1440e',  False),
    'n_in_s':       ('shafts',      'input shaft sensor',       None,               '#2a628f',  False),
    'j_ref':        ('shafts',      'reflected J [kg·m²]',      None,               '#888888',  True),
    'slip':         ('slip',        'slip [rpm]',               lambda v: v* RPM,   '#7d3f9c',  False),
    'f_road':       ('road',        'road force [N]',           None,               '#c1440e',  False),
    't_out':        ('road',        'shaft reaction [Nm]',      None,               '#2a628f',  True),

    # --- TCU and clutch
    'lever':        ('selector',    'lever P/R/N/D',            None,               '#555555',  False),
    'gear':         ('selector',    'gear',                     None,               '#c1440e',  False),
    'clutch_state': ('selector',    'clutch open/eng/closed',   None,               '#2a628f',  False),
    'clutch_cmd':   ('ccmd',        'clutch command',           None,               '#2a628f',  False),
    't_disc':       ('disc',        'disc (plant) [C]',         K2C,                '#c1440e',  False),
    't_disc_est1':  ('disc',        'TCU estimate [C]',         K2C,                '#2a628f',  False),
    't_disc_est2':  ('disc',        'TCU estimate [C]',         K2C,                '#2a628f',  False),
    'q_clutch':     ('qclutch',     'clutch heat [kW]',         lambda v: v / 1000, '#c1440e',  False),
    'wear_um':      ('damage',      'wear [µm]',                None,               '#555555',  False),
    'glaze':        ('damage',      'glaze',                    None,               '#c1440e',  True),
}

# Discrete signals draw as steps: a linear ramp between gear 3 and gear 4 implies a gear 3.5 that never existed
STEP = {'lever', 'gear', 'clutch_state'}

# One PNG per page. A page is written only if at least one of its columns is in the CSV, so older runs still plot
PAGES = [
    ('engine',      ['speed', 'torque', 'fuel', 'pedal']),
    ('airpath',     ['air', 'afr', 'flow', 'turbo']),
    ('thermal',     ['egt', 'temp', 'heat']),
    ('diag',        ['flags', 'bias', 'eta']),
    ('driveline',   ['vspeed', 'shafts', 'slip', 'road']),
    ('tcu',         ['selector', 'ccmd', 'disc', 'qclutch', 'damage']),
]

PANEL_LABEL = {
    'speed': 'engine speed\n[rpm]', 'torque': 'torque\n[Nm]', 'fuel': 'fuel\n[mg/stroke]',
    'pedal': 'pedal [%]', 'air': 'pressure\n[kPa]', 'afr': 'AFR', 'flow': 'air flow\n[g/s]',
    'turbo': 'turbo\n[rpm]', 'egt': 'EGT\n[C]', 'temp': 'temperature\n[C]', 'heat': 'heat\n[kW]',
    'flags': '', 'bias': 'bias\n[Nm]', 'eta': 'eta_ind', 'vspeed': 'vehicle\n[km/h]',
    'shafts': 'input shaft\n[rpm]', 'slip': 'slip\n[rpm]', 'road': 'road force\n[N]',
    'selector': 'selector', 'ccmd': 'clutch cmd', 'disc': 'disc temp\n[C]',
    'qclutch': 'clutch heat\n[kW]', 'damage': 'wear\n[µm]',
}

PANEL_HEIGHT = {'speed': 2.0, 'pedal': 0.7, 'flags': 1.4, 'selector': 1.0, 'ccmd': 0.8}

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

def draw_panel(p, key, cols, c, t):
    handles, twin = [], None
    for n in cols:
        _, label, fn, color, is_twin = SPEC.get(n, (key, n, None, None, False))
        y = [fn(v) for v in c[n]] if fn else c[n]
        target = p
        if is_twin:
            twin = twin or p.twinx()
            target = twin
            twin.set_ylabel(label, color=color)
        if n in STEP:
            ln, = target.step(t, y, where='post', lw=1.1, color=color, label=label)
        else:
            wide = (n == 'omega')
            ln, = target.plot(t, y, lw=4.0 if wide else 1.1, color=color,
                              alpha=0.30 if wide else 0.9, label=label,
                              zorder=1 if wide else 2)
        handles.append(ln)
    p.set_ylabel(PANEL_LABEL.get(key, key.lstrip('_')))
    if len(handles) > 1:
        p.legend(handles=handles, loc='best', fontsize=8, framealpha=0.95, ncols=min(3, len(handles)))

def draw_flags(p, cols, c, t):
    # One lane per flag rather than a full panel each. A flag that is true 0.1% of the time reads as a few thin ticks in
    # its lane instead of a wall of full-height spikes.
    for i, n in enumerate(cols):
        y = [i * 1.5 + (1.0 if v > 0.5 else 0.0) for v in c[n]]
        p.step(t, y, where='post', lw=0.9, color=SPEC[n][3])
    p.set_yticks([i * 1.5 + 0.5 for i in range(len(cols))])
    p.set_yticklabels([SPEC[n][1] for n in cols], fontsize=8)
    p.set_ylim(-0.3, len(cols) * 1.5)


def render(plt, page, keys, panels, c, t, spans, title):
    heights = [PANEL_HEIGHT.get(k, 1.1) for k in keys]
    fig, axes = plt.subplots(len(keys), 1, sharex=True, squeeze=False,
                             figsize=(11, 1.7 * sum(heights) + 0.6),
                             gridspec_kw={'height_ratios': heights})
    axes = list(axes[:, 0])

    for s, e, st in spans:
        col = '#f4c542' if st == 1 else '#c1440e'
        for axis in axes:
            axis.axvspan(s, e, color=col, alpha=0.10 if st == 2 else 0.20, lw=0)

    for p, key in zip(axes, keys):
        if key == 'flags':
            draw_flags(p, panels[key], c, t)
        else:
            draw_panel(p, key, panels[key], c, t)
        p.grid(alpha=0.22)
        p.margins(x=0)

    if 'speed' in keys:
        sp = axes[keys.index('speed')]
        for s, _, st in spans:
            sp.annotate(f'P0016 {DTC_LABEL.get(int(st), st)}',
                        xy=(s, sp.get_ylim()[1]), xytext=(-3, -6),
                        textcoords='offset points', rotation=90, fontsize=7.5,
                        va='top', ha='right', color='0.25')

    axes[-1].set_xlabel('simulation time [s]')
    axes[0].set_title(f'{title} — {page}', fontsize=11)
    fig.tight_layout()
    return fig

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
    spans = dtc_spans(t, c['dtc']) if 'dtc' in c else []

    panels = {}
    for n in names:
        if n in ('t_s', 'dtc'):
            continue
        key = SPEC[n][0] if n in SPEC else f'_{n}'
        panels.setdefault(key, []).append(n)

    pages = [(pg, [k for k in ks if k in panels]) for pg, ks in PAGES]
    # Anything SPEC does not know goes on its own page rather than vanishing,
    # so a new logger column is visible the day it is added.
    known = {k for _, ks in PAGES for k in ks}
    other = [k for k in panels if k not in known]
    if other:
        pages.append(('other', other))

    title = a.title or f'Revlab — {os.path.basename(a.csv)}'
    base = os.path.splitext(a.out or a.csv)[0]
    for page, keys in pages:
        if not keys:
            continue
        fig = render(plt, page, keys, panels, c, t, spans, title)
        out = f'{base}.{page}.png'
        fig.savefig(out, dpi=150)
        print(f'wrote {out}')
        if not a.show:
            plt.close(fig)      # six figures per run add up across a sweep
    if a.show:
        plt.show()


if __name__ == '__main__':
    main()