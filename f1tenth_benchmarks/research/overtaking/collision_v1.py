"""V1 offline, discrete-time circumscribed-circle inspection; never a control gate."""

import argparse
import csv
import json
import math
from numbers import Integral
from pathlib import Path

METHOD = 'aligned_discrete_circumscribed_circles_v1'
LIMITATIONS = [
    'Only discrete timestamps are checked; continuous swept motion is not checked.',
    'First contact is a grid timestamp, not an exact continuous collision time.',
    'Circle overlap is conservative relative to rectangular bodies; no dynamics are verified.',
    'Time origins are caller assertions in one common clock, not inferred from relative t_s.',
]


def _finite(value):
    if isinstance(value, bool):
        raise ValueError('boolean_is_not_a_number')
    number = float(value)
    if not math.isfinite(number):
        raise ValueError('nonfinite_number')
    return number


def _body(spec):
    values = {key: _finite(spec[key]) for key in ('length_m', 'width_m', 'margin_m')}
    if values['length_m'] <= 0 or values['width_m'] <= 0 or values['margin_m'] < 0:
        raise ValueError('invalid_body_dimensions')
    radius = math.hypot(values['length_m'], values['width_m']) / 2 + values['margin_m']
    if not math.isfinite(radius):
        raise ValueError('nonfinite_radius')
    return dict(values, radius_m=radius)


def _prediction(prediction):
    if not isinstance(prediction, dict) or prediction.get('status') not in ('complete', 'path_exhausted'):
        raise ValueError('prediction_rejected_or_unknown_status')
    if prediction.get('time_origin') != 'relative_prediction_start':
        raise ValueError('unsupported_relative_time_origin')
    parameters = prediction['parameters']
    dt = _finite(parameters['dt_s'])
    steps = parameters['steps']
    if dt <= 0 or isinstance(steps, bool) or not isinstance(steps, Integral) or not 0 <= steps <= 100000:
        raise ValueError('invalid_requested_grid')
    horizon = _finite(prediction['requested_horizon_s'])
    if not math.isfinite(dt * steps) or horizon != dt * steps:
        raise ValueError('inconsistent_requested_horizon')
    samples = prediction['samples']
    if not isinstance(samples, list) or not 1 <= len(samples) <= steps + 1:
        raise ValueError('missing_or_excess_samples')
    clean = []
    for index, sample in enumerate(samples):
        item = {key: _finite(sample[key]) for key in ('t_s', 'x', 'y', 'yaw', 'speed_mps')}
        if item['speed_mps'] < 0 or item['t_s'] != index * dt:
            raise ValueError('invalid_sample_or_nonmatching_grid')
        clean.append(item)
    complete = prediction.get('prediction_complete')
    if prediction['status'] == 'complete':
        if complete is not True or len(samples) != steps + 1:
            raise ValueError('inconsistent_complete_prediction')
    elif complete is not False or len(samples) >= steps + 1:
        raise ValueError('inconsistent_incomplete_prediction')
    if _finite(prediction['last_sample_t_s']) != clean[-1]['t_s']:
        raise ValueError('inconsistent_last_sample')
    available = _finite(prediction['available_until_t_s'])
    if not clean[-1]['t_s'] <= available <= horizon:
        raise ValueError('inconsistent_available_range')
    if complete and available != horizon:
        raise ValueError('inconsistent_complete_range')
    return dt, int(steps), clean


def evaluate_collision(ego_prediction, lead_prediction, *, common_start_time_s,
                       ego_start_time_s, lead_start_time_s, ego_body, lead_body):
    """Inspect matching prediction_v1 grids without interpolation.

    All three mandatory origins are seconds in the SAME caller-defined clock and
    must be exactly equal. Per-vehicle origins attach external timestamps to
    prediction_v1's relative samples. Bodies require length_m, width_m, margin_m.
    available_until_t_s bounds the usable samples; path_end_time_s may retain
    the raw physical estimate before prediction_v1's one-ULP endpoint rule.
    Malformed inputs return rejected/unknown; valid truncated prefixes return
    incomplete, even when contact is found. No result authorizes planning.
    """
    sources = {}
    for name, prediction in (('ego', ego_prediction), ('lead', lead_prediction)):
        sources[name] = {key: prediction.get(key) for key in (
            'candidate_id', 'geometry_id', 'frame_id', 'method', 'status', 'reason', 'source_reason'
        )} if isinstance(prediction, dict) else None
    result = dict(schema_version=1, method=METHOD, status='rejected', reason=None,
                  assessment='unknown', evaluation_complete=False, checked_range_s=None,
                  requested_range_s=None, current_contact=None,
                  any_contact_in_checked_samples=None, first_contact_grid_t_s=None,
                  first_future_contact_grid_t_s=None, minimum_center_distance_m=None,
                  minimum_center_distance_t_s=None, minimum_clearance_m=None,
                  minimum_clearance_t_s=None, samples=[], sources=sources,
                  inspection_only=True, planning_allowed=False,
                  continuous_sweep_checked=False, discrete_only=True,
                  contact_rule='d_center <= r_ego + r_lead',
                  clearance_rule='max(0, d_center - r_ego - r_lead)',
                  units={'length': 'm', 'time': 's'}, limitations=list(LIMITATIONS))
    try:
        common = _finite(common_start_time_s)
        if _finite(ego_start_time_s) != common or _finite(lead_start_time_s) != common:
            raise ValueError('prediction_start_time_mismatch')
        result['time_alignment'] = dict(common_start_time_s=common,
            ego_start_time_s=common, lead_start_time_s=common,
            clock='caller_supplied_common_clock', grid_comparison='exact_no_interpolation')
        bodies = dict(ego=_body(ego_body), lead=_body(lead_body))
        result['bodies'] = bodies
        ego_grid = _prediction(ego_prediction)
        lead_grid = _prediction(lead_prediction)
        frame = ego_prediction.get('frame_id')
        if not isinstance(frame, str) or not frame or frame != lead_prediction.get('frame_id'):
            raise ValueError('frame_id_missing_or_mismatch')
        result['frame_id'] = frame
        if ego_grid[:2] != lead_grid[:2]:
            raise ValueError('requested_time_grid_mismatch')
        dt, steps, ego = ego_grid
        lead = lead_grid[2]
        result['requested_range_s'] = [0., dt * steps]
        radius_sum = bodies['ego']['radius_m'] + bodies['lead']['radius_m']
        if not math.isfinite(radius_sum):
            raise ValueError('nonfinite_radius_sum')
        # Both arrays have been validated as prefixes of the SAME requested grid.
        # Explicitly expose the shorter checked prefix; never silently zip arrays.
        count = min(len(ego), len(lead))
        rows = []
        for index in range(count):
            e, l = ego[index], lead[index]
            distance = math.hypot(e['x'] - l['x'], e['y'] - l['y'])
            if not math.isfinite(distance):
                raise ValueError('nonfinite_center_distance')
            rows.append(dict(t_s=e['t_s'], ego_x=e['x'], ego_y=e['y'],
                lead_x=l['x'], lead_y=l['y'], center_distance_m=distance,
                clearance_m=max(0., distance-radius_sum), contact=distance <= radius_sum))
        complete = count == steps + 1
        contacts = [row['t_s'] for row in rows if row['contact']]
        future = [time for time in contacts if time > 0]
        nearest = min(rows, key=lambda row: row['center_distance_m'])
        clearance = min(rows, key=lambda row: row['clearance_m'])
        incomplete = [name + ':' + prediction['reason'] for name, prediction in
                      (('ego', ego_prediction), ('lead', lead_prediction))
                      if prediction['status'] == 'path_exhausted' and isinstance(prediction.get('reason'), str)]
        result.update(status='complete' if complete else 'incomplete',
            reason=None if complete else 'incomplete_prediction_range',
            incomplete_reasons=incomplete, evaluation_complete=complete,
            assessment='contact_detected' if contacts else ('no_contact_at_samples' if complete else 'unknown'),
            checked_range_s=[0., rows[-1]['t_s']], checked_sample_count=count,
            input_sample_counts={'ego': len(ego), 'lead': len(lead)},
            unchecked_range_s=None if complete else {'after_t_s': rows[-1]['t_s'], 'through_t_s': dt*steps},
            current_contact=rows[0]['contact'], any_contact_in_checked_samples=bool(contacts),
            first_contact_grid_t_s=contacts[0] if contacts else None,
            first_future_contact_grid_t_s=future[0] if future else None,
            minimum_center_distance_m=nearest['center_distance_m'],
            minimum_center_distance_t_s=nearest['t_s'],
            minimum_clearance_m=clearance['clearance_m'],
            minimum_clearance_t_s=clearance['t_s'], samples=rows)
    except (ValueError, TypeError, KeyError, OverflowError) as exc:
        result['reason'] = str(exc)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--dt-s', type=float, default=.2)
    parser.add_argument('--horizon-s', type=float, default=6.)
    for vehicle in ('ego', 'lead'):
        for field, default in (('length', .6), ('width', .8), ('margin', 0.)):
            parser.add_argument('--' + vehicle + '-' + field + '-m', type=float, default=default,
                                help='Artificial demonstration parameter only')
    args = parser.parse_args()
    from .prediction_v1 import predict_polyline
    try:
        dt, horizon = _finite(args.dt_s), _finite(args.horizon_s)
        if dt <= 0 or horizon < 0 or not math.isfinite(horizon / dt):
            raise ValueError('invalid_time_parameters')
        steps = round(horizon/dt)
        if not 0 <= steps <= 100000 or abs(steps*dt-horizon) > 1e-9:
            raise ValueError('horizon_s must be an integer multiple of dt_s, at most 100000 steps')
        bodies = {vehicle: _body({field + '_m': getattr(args, vehicle + '_' + field + '_m')
                                 for field in ('length', 'width', 'margin')})
                  for vehicle in ('ego', 'lead')}
        cases = {
            'catch_up': ([[0., 0.], [30., 0.]], 2., [[3., 0.], [30., 0.]], 1.),
            'parallel_separated': ([[0., 0.], [30., 0.]], 1., [[0., 2.], [30., 2.]], 1.),
            'crossing_synchronous': ([[-2., 0.], [20., 0.]], 1., [[0., -2.], [0., 20.]], 1.),
            'crossing_asynchronous': ([[-2., 0.], [20., 0.]], 1., [[0., -5.], [0., 20.]], 1.),
            'initial_overlap': ([[0., 0.], [30., 0.]], 1., [[.5, 0.], [30., 0.]], 1.),
            'path_exhausted': ([[0., 0.], [1., 0.]], 1., [[3., 0.], [30., 0.]], 0.),
        }
        results, predictions = {}, {}
        for name, (ep, ev, lp, lv) in cases.items():
            pair = {vehicle: predict_polyline(path, path[0], speed, dt, steps,
                    geometry_id='artificial-' + name, frame_id='synthetic_map')
                    for vehicle, path, speed in (('ego', ep, ev), ('lead', lp, lv))}
            predictions[name] = pair
            results[name] = evaluate_collision(pair['ego'], pair['lead'], common_start_time_s=0.,
                ego_start_time_s=0., lead_start_time_s=0., ego_body=bodies['ego'], lead_body=bodies['lead'])
            if results[name]['status'] == 'rejected':
                raise ValueError(name + ': ' + results[name]['reason'])
        output = Path(args.output_dir)
        output.mkdir(parents=True, exist_ok=False)
    except (ValueError, FileExistsError, OverflowError) as exc:
        parser.error(str(exc))
    payload = dict(method=METHOD, parameter_source='artificial_test_parameters_only',
        dt_s=dt, steps=steps, horizon_s=steps*dt, bodies=bodies,
        input_cases={name: dict(ego_path=case[0], ego_speed_mps=case[1],
                              lead_path=case[2], lead_speed_mps=case[3]) for name, case in cases.items()},
        predictions=predictions, results=results, inspection_only=True, planning_allowed=False,
        limitations=LIMITATIONS)
    with (output/'collision.json').open('x', encoding='utf-8') as stream:
        json.dump(payload, stream, indent=2, allow_nan=False)
    fields = ['case', 'evaluation_status', 't_s', 'ego_x', 'ego_y', 'lead_x', 'lead_y',
              'center_distance_m', 'clearance_m', 'contact']
    with (output/'collision.csv').open('x', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for name, result in results.items():
            for sample in result['samples']:
                writer.writerow(dict(sample, case=name, evaluation_status=result['status']))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    fig, axes = plt.subplots(2, 3, figsize=(18, 10), constrained_layout=True)
    for ax, (name, result) in zip(axes.flat, results.items()):
        case = cases[name]
        for vehicle, path, color, marker in (('ego', case[0], 'tab:blue', 'o'),
                                            ('lead', case[2], 'tab:orange', '^')):
            ax.plot([p[0] for p in path], [p[1] for p in path], '--', color=color, alpha=.4,
                    label=vehicle + ' input path')
            rows = result['samples']
            ax.scatter([r[vehicle+'_x'] for r in rows], [r[vehicle+'_y'] for r in rows],
                       c=[r['t_s'] for r in rows], cmap='viridis', vmin=0, vmax=max(horizon, dt),
                       marker=marker, s=23, label=vehicle + ' aligned samples')
            for index in sorted(set((0, len(rows)//2, len(rows)-1))):
                row = rows[index]
                ax.annotate(f"{row['t_s']:.2g}s", (row[vehicle+'_x'], row[vehicle+'_y']),
                            xytext=(2, 8 if vehicle == 'ego' else -13), textcoords='offset points', fontsize=8)
            if result['first_contact_grid_t_s'] is not None:
                row = next(r for r in rows if r['t_s'] == result['first_contact_grid_t_s'])
                ax.add_patch(Circle((row[vehicle+'_x'], row[vehicle+'_y']),
                                    bodies[vehicle]['radius_m'], fill=False, color=color, linewidth=2))
        # Crop unused long path tails while retaining all inspected positions and circles.
        xs = [r[v+'_x'] for r in rows for v in ('ego', 'lead')]
        ys = [r[v+'_y'] for r in rows for v in ('ego', 'lead')]
        pad = max(b['radius_m'] for b in bodies.values()) + .5
        ax.set_xlim(min(xs)-pad, max(xs)+pad)
        ax.set_ylim(min(ys)-pad, max(ys)+pad)
        ax.set_aspect('equal', adjustable='box')
        ax.set(title=name + '\n' + result['status'] + ' / ' + result['assessment'], xlabel='x [m]', ylabel='y [m]')
        ax.grid(alpha=.2)
        ax.legend(fontsize=7, loc='upper left', bbox_to_anchor=(0, -.17), ncol=2)
    fig.suptitle('Artificial two-vehicle cases | discrete inspection only | circles at first grid contact')
    fig.savefig(output/'collision.png', dpi=200, facecolor='white')
    plt.close(fig)
    print(json.dumps(dict(output_dir=str(output), results={name: dict(status=r['status'],
        first_contact_grid_t_s=r['first_contact_grid_t_s']) for name, r in results.items()})))


if __name__ == '__main__':
    main()
