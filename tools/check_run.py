#!/usr/bin/env python3
"""Summarize one run: DTC timings and the health metrics we validate against"""
import sys
import numpy as np

d = np.genfromtxt(sys.argv[1], delimiter=',', names=True)
t = d['t_s']
m = t > 60 if t[-1] > 120 else t > 2    # Skip startup transient

err = d['n_model'][m] - d['n_crank'][m]
print(' n_model     mean %+7.1f     max |%.1f|  rpm' % (err.mean(), abs(err).max()))
print(' t_arb       mean %+7.2f     max |%.2f|  Nm' % (d['t_arb'][m].mean(), abs(d['t_arb'][m]).max()))
print(' freeze      %5.2f%%         cam_valid %5.1f%%   crank_valid %5.1f%%' % (100*d['freeze'][m].mean(), 100*d['cam_valid'].mean(), 100*d['crank_valid'].mean()))

for lvl, name in ((1, 'pending  '), (2, 'confirmed')):
    hit = d['dtc'] >= lvl
    print(' %s %s' % (name, 't=%.2f s' % t[np.argmax(hit)] if hit.any() else 'never'))