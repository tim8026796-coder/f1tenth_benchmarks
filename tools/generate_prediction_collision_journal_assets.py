"""Render journal figures from the existing V1 modules; no planner changes."""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle
import numpy as np

from f1tenth_benchmarks.research.overtaking.prediction_v1 import predict_polyline
from f1tenth_benchmarks.research.overtaking.collision_v1 import evaluate_collision


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', default='docs/research/journal_2026_10_23')
    args = parser.parse_args()
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    names = ('prediction_v1_demo.png', 'collision_v1_demo.png')
    if any((output / name).exists() for name in names):
        parser.error('Figure already exists: choose a new output directory.')
    plt.rcParams.update({'font.size': 12, 'axes.titlesize': 15,
                         'figure.facecolor': 'white', 'axes.facecolor': 'white',
                         'savefig.facecolor': 'white', 'font.family': 'DejaVu Sans'})

    angle = np.linspace(0., np.pi/2, 25)
    path = np.column_stack((4*np.sin(angle), 4*(1-np.cos(angle))))
    prediction = predict_polyline(path, path[0], 1.5, .25, 12,
                                  geometry_id='journal-synthetic-curve', frame_id='synthetic_map')
    if prediction['status'] != 'complete':
        raise RuntimeError(prediction['reason'])
    samples = prediction['samples']
    fig, ax = plt.subplots(figsize=(10, 7))
    fig.subplots_adjust(left=.1, right=.86, bottom=.2, top=.87)
    ax.plot(path[:, 0], path[:, 1], '--', color='.55', lw=2,
            label='Reference polyline (25 vertices)', zorder=1)
    scatter = ax.scatter([p['x'] for p in samples], [p['y'] for p in samples],
                         c=[p['t_s'] for p in samples], cmap='viridis', s=65,
                         edgecolors='white', linewidth=.6, zorder=3, label='Predicted samples (0.25 s)')
    for index, offset in ((0, (12, -28)), (4, (8, -24)), (8, (10, -20)), (12, (10, -10))):
        sample = samples[index]
        ax.annotate(f"t = {sample['t_s']:.0f} s", (sample['x'], sample['y']),
                    xytext=offset, textcoords='offset points', fontsize=12, weight='bold')
        ax.arrow(sample['x'], sample['y'], .33*np.cos(sample['yaw']), .33*np.sin(sample['yaw']),
                 head_width=.07, length_includes_head=True, color='navy', zorder=4)
    ax.scatter([0], [0], marker='*', s=200, color='crimson', zorder=5, label='Start position (t = 0)')
    ax.set(title='Prediction V1: future positions along a reference path', xlabel='x [m]', ylabel='y [m]',
           xlim=(-.45, 4.65), ylim=(-.55, 4.25))
    ax.set_aspect('equal')
    ax.grid(alpha=.2)
    ax.legend(loc='upper left', fontsize=10, framealpha=.95)
    fig.colorbar(scatter, ax=ax, fraction=.045, pad=.04, label='Relative prediction time [s]')
    fig.text(.5, .07, 'Artificial case | constant speed = 1.5 m/s | horizon = 3 s\n'
             'XY arc-length interpolation; arrows show polyline-tangent yaw. Inspection only.',
             ha='center', fontsize=11)
    fig.savefig(output / names[0], dpi=250)
    plt.close(fig)

    ep = np.array([[-2., 0.], [4., 0.]])
    lp = np.array([[0., -2.], [0., 4.]])
    ego = predict_polyline(ep, ep[0], 1., .25, 12, geometry_id='journal-synthetic-crossing', frame_id='synthetic_map')
    lead = predict_polyline(lp, lp[0], 1., .25, 12, geometry_id='journal-synthetic-crossing', frame_id='synthetic_map')
    body = dict(length_m=.6, width_m=.8, margin_m=0.)
    collision = evaluate_collision(ego, lead, common_start_time_s=0., ego_start_time_s=0.,
                                   lead_start_time_s=0., ego_body=body, lead_body=body)
    if not collision['evaluation_complete'] or collision['first_contact_grid_t_s'] != 1.5:
        raise RuntimeError('Unexpected synthetic collision result')
    fig, (ax, distances) = plt.subplots(1, 2, figsize=(13, 6), gridspec_kw={'width_ratios': [1.1, 1]})
    fig.subplots_adjust(left=.07, right=.98, bottom=.27, top=.83, wspace=.27)
    rows = collision['samples']
    first = collision['first_contact_grid_t_s']
    for vehicle, path, color, marker in (('ego', ep, 'tab:blue', 'o'), ('lead', lp, 'tab:orange', '^')):
        ax.plot(path[:, 0], path[:, 1], '--', color=color, alpha=.35, label=vehicle.capitalize()+' reference')
        ax.plot([p[vehicle+'_x'] for p in rows], [p[vehicle+'_y'] for p in rows],
                color=color, marker=marker, markersize=5, label=vehicle.capitalize()+' predicted samples')
        for index, time in enumerate((0., 1., first)):
            row = next(p for p in rows if p['t_s'] == time)
            xy = (row[vehicle+'_x'], row[vehicle+'_y'])
            ax.add_patch(Circle(xy, .5, fill=False, color=color,
                                alpha=1. if time == first else .35,
                                lw=2.3 if time == first else 1.3,
                                linestyle='-' if time == first else ':'))
            offset = (-48, -18) if vehicle == 'ego' else (12, -5)
            ax.annotate(f'{time:g} s', xy, xytext=offset, textcoords='offset points', fontsize=10, color=color)
            if time == first:
                ax.scatter(*xy, marker='*', s=190, color='crimson', edgecolors='white', zorder=6)
    ax.set(xlabel='x [m]', ylabel='y [m]', xlim=(-2.75, 1.75), ylim=(-2.75, 1.75),
           title='Aligned future positions and bounding circles')
    ax.set_aspect('equal')
    ax.grid(alpha=.2)
    times = [p['t_s'] for p in rows]
    distances.plot(times, [p['center_distance_m'] for p in rows], 'o-', color='slateblue',
                   markersize=4, label='Center distance')
    distances.axhline(1., color='firebrick', ls='--', label='Contact threshold: 1.0 m')
    distances.axvline(first, color='crimson', ls=':', label=f'First contact sample: {first:g} s')
    contacts = [p for p in rows if p['contact']]
    distances.scatter([p['t_s'] for p in contacts], [p['center_distance_m'] for p in contacts],
                      color='crimson', zorder=5, label='Contact at checked samples')
    distances.set(xlabel='Relative prediction time [s]', ylabel='Center distance [m]',
                  title='Contact rule: center distance <= radius sum', xlim=(-.05, 3.05), ylim=(-.15, 3.1))
    distances.grid(alpha=.2)
    distances.legend(loc='upper right', fontsize=9)
    handles, labels = ax.get_legend_handles_labels()
    handles += [Line2D([0], [0], marker='*', ls='', color='crimson', markersize=11)]
    labels += ['First contact sample (1.5 s)']
    fig.legend(handles, labels, loc='lower center', bbox_to_anchor=(.5, .09), ncol=3, fontsize=10)
    fig.suptitle('Collision V1: discrete-time collision evaluation', fontsize=17, y=.96)
    fig.text(.5, .025, 'Artificial parameters: each vehicle 0.6 x 0.8 m, margin 0 m, radius 0.5 m; speed 1 m/s; dt 0.25 s.\n'
             'Discrete samples only: first contact is a grid time, not exact continuous TTC. No ESP integration.',
             ha='center', fontsize=10)
    fig.savefig(output / names[1], dpi=250)
    plt.close(fig)
    print('Prediction: complete, 13 samples, 0-3 s')
    print('Collision: complete, first contact grid time 1.5 s, minimum center distance 0 m at 2 s')
    print('Generated: '+', '.join(str(output / name) for name in names))


if __name__ == '__main__':
    main()
