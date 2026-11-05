"""V1 offline constant-speed XY-polyline timing; no collision or control approval."""
import argparse
import csv
import json
import math
from numbers import Integral
from pathlib import Path

import numpy as np


METHOD = 'xy_polyline_arclength_constant_speed_v1'
LIMITATIONS = [
    'Open finite polyline; no wrapping, acceleration or opponent prediction.',
    'Yaw is the outgoing XY segment tangent, not the source point yaw.',
    'No geometry, occupancy, footprint, collision or dynamic safety validation.',
    'Projection may move t=0 within the explicitly configured start tolerance.',
]


def _number(value, name, positive=False):
    try:
        if isinstance(value, (bool, np.bool_)):
            raise ValueError
        value = float(value)
        if not math.isfinite(value) or (value <= 0 if positive else value < 0):
            raise ValueError
    except (TypeError, ValueError, OverflowError):
        raise ValueError('invalid_' + name) from None
    return value


def _base(geometry_id, frame_id):
    return dict(schema_version=1, method=METHOD, geometry_id=geometry_id,
                frame_id=frame_id, inspection_only=True, planning_allowed=False,
                collision_evaluated=False, dynamics_verified=False,
                status='rejected', reason=None, prediction_complete=False, samples=[],
                time_origin='relative_prediction_start',
                units=dict(position='m', time='s', speed='m/s', yaw='rad'),
                limitations=list(LIMITATIONS))


def predict_polyline(path_xy, start_xy, speed_mps, dt_s, steps, *,
                     max_start_error_m=.05, projection_tolerance_m=1e-6,
                     ambiguity_arc_tolerance_m=1e-5, geometry_id=None, frame_id='map'):
    """Return H+1 grid samples at k*dt, or a rejected/truncated result.

    t=0 is the nearest projection, reported with its error. Near-equal nearest
    projections at distinct arc positions are rejected, not resolved by index.
    Consecutive identical points are removed. At a vertex yaw uses the outgoing
    segment (the last segment at the endpoint). No source points.t is used.
    steps is an integer H in [0, 100000]; all times are relative seconds.
    """
    result = _base(geometry_id, frame_id)
    try:
        if any(v is not None and (not isinstance(v, str) or not v)
               for v in (geometry_id, frame_id)):
            result.update(geometry_id=None, frame_id=None)
            raise ValueError('invalid_metadata')
        speed = _number(speed_mps, 'speed_mps')
        dt = _number(dt_s, 'dt_s', positive=True)
        error_limit = _number(max_start_error_m, 'max_start_error_m')
        tie = _number(projection_tolerance_m, 'projection_tolerance_m')
        arc_tie = _number(ambiguity_arc_tolerance_m, 'ambiguity_arc_tolerance_m')
        if isinstance(steps, (bool, np.bool_)) or not isinstance(steps, Integral) or not 0 <= steps <= 100000:
            raise ValueError('invalid_steps')
        steps = int(steps)
        horizon = steps * dt
        if not math.isfinite(horizon) or not math.isfinite(speed * horizon):
            raise ValueError('nonfinite_prediction_horizon')
        result['parameters'] = dict(speed_mps=speed, dt_s=dt, steps=steps,
            max_start_error_m=error_limit, projection_tolerance_m=tie,
            ambiguity_arc_tolerance_m=arc_tie)
        try:
            xy = np.asarray(path_xy, dtype=float)
            start = np.asarray(start_xy, dtype=float)
        except (TypeError, ValueError):
            raise ValueError('invalid_coordinates') from None
        if xy.ndim != 2 or xy.shape[1] != 2 or len(xy) < 2 or not np.isfinite(xy).all():
            raise ValueError('invalid_path_xy')
        if start.shape != (2,) or not np.isfinite(start).all():
            raise ValueError('invalid_start_xy')
        original_count = len(xy)
        xy = xy[np.r_[True, np.any(xy[1:] != xy[:-1], axis=1)]]
        if len(xy) < 2:
            raise ValueError('zero_length_path')
        with np.errstate(over='ignore', invalid='ignore', divide='ignore'):
            delta = np.diff(xy, axis=0)
            lengths = np.hypot(delta[:, 0], delta[:, 1])
            arc = np.r_[0., np.cumsum(lengths)]
            if not np.isfinite(arc).all() or np.any(np.diff(arc) <= 0):
                raise ValueError('unresolvable_path_length')
            unit = delta / lengths[:, None]
            along = np.clip(np.sum((start - xy[:-1]) * unit, axis=1), 0., lengths)
            projected = xy[:-1] + along[:, None] * unit
            distances = np.hypot(*(projected - start).T)
            projected_s = arc[:-1] + along
        if not np.isfinite(distances).all() or not np.isfinite(projected_s).all():
            raise ValueError('unresolvable_projection')
        best = int(np.argmin(distances))
        near = projected_s[distances <= distances[best] + tie]
        result['projection'] = dict(input_xy=start.tolist(), projected_xy=projected[best].tolist(),
            error_m=float(distances[best]), arc_s_m=float(projected_s[best]),
            near_projection_count=len(near), near_arc_span_m=float(np.ptp(near)))
        if distances[best] > error_limit:
            raise ValueError('start_too_far_from_path')
        if np.ptp(near) > arc_tie:
            raise ValueError('ambiguous_start_projection')
        s0 = min(float(projected_s[best]), float(arc[-1]))
        remaining = float(arc[-1]) - s0
        end_time = remaining / speed if speed > 0 else None
        if end_time is not None and not math.isfinite(end_time):
            raise ValueError('nonfinite_path_end_time')
        available = min(horizon, end_time) if end_time is not None else horizon
        times = np.arange(steps + 1, dtype=float) * dt
        if end_time is not None:
            times = times[times <= np.nextafter(end_time, math.inf)]
        # Report the usable grid range, including an endpoint already admitted
        # by the existing one-ULP rule. Keep raw path_end_time_s unchanged.
        available = max(available, float(times[-1]))
        positions_s = np.minimum(s0 + speed * times, arc[-1])
        indices = np.clip(np.searchsorted(arc, positions_s, side='right') - 1, 0, len(lengths) - 1)
        positions = xy[indices] + (positions_s - arc[indices])[:, None] * unit[indices]
        yaws = (np.arctan2(delta[indices, 1], delta[indices, 0]) + math.pi) % (2 * math.pi) - math.pi
        complete = len(times) == steps + 1
        result.update(status='complete' if complete else 'path_exhausted',
            reason=None if complete else 'requested_horizon_exceeds_path',
            prediction_complete=complete, requested_horizon_s=horizon,
            available_until_t_s=available, last_sample_t_s=float(times[-1]),
            path_end_time_s=end_time, path_end_xy=xy[-1].tolist(),
            path_length_m=float(arc[-1]), remaining_length_m=remaining,
            removed_duplicate_points=original_count-len(xy),
            samples=[dict(t_s=float(t), x=float(p[0]), y=float(p[1]), yaw=float(yaw), speed_mps=speed)
                     for t, p, yaw in zip(times, positions, yaws)])
    except ValueError as exc:
        result['reason'] = str(exc)
    return result


def predict_candidate(candidate, start_xy, speed_mps, dt_s, steps, *, context=None, **options):
    """Adapt one candidate, optionally inheriting metadata from its plan event.

    Reject native-rejected candidates and any invalid point. Do not bridge gaps,
    regenerate paths, interpret points.t as seconds, or reuse source yaw as control.
    """
    if not isinstance(candidate, dict) or (context is not None and not isinstance(context, dict)):
        result = _base(None, None)
        result['reason'] = 'invalid_candidate_or_context'
        return result
    context = context or {}
    result = _base(candidate.get('geometry_id', context.get('geometry_id')),
                   candidate.get('frame_id', context.get('frame_id')))
    result.update(candidate_id=candidate.get('candidate_id'), source_reason=candidate.get('reason'))
    if candidate.get('sampled_valid') is not True:
        result['reason'] = 'candidate_rejected'
        return result
    if isinstance(result['candidate_id'], (bool, np.bool_)) or not isinstance(result['candidate_id'], Integral):
        result['reason'] = 'invalid_candidate_id'
        return result
    result['candidate_id'] = int(result['candidate_id'])
    for key in ('geometry_id', 'frame_id'):
        own, parent = candidate.get(key), context.get(key)
        if own is not None and parent is not None and own != parent:
            result['reason'] = 'metadata_mismatch'
            return result
        if not isinstance(result[key], str) or not result[key]:
            result['reason'] = 'missing_metadata'
            return result
    if set(options) - {'max_start_error_m', 'projection_tolerance_m', 'ambiguity_arc_tolerance_m'}:
        result['reason'] = 'invalid_prediction_options'
        return result
    points = candidate.get('points')
    if not isinstance(points, list) or len(points) < 2:
        result['reason'] = 'unusable_candidate_points'
        return result
    xy = []
    for i, point in enumerate(points):
        try:
            if not isinstance(point, dict) or point.get('valid') is not True:
                raise ValueError
            pair = [float(point['x']), float(point['y'])]
            if not np.isfinite(pair).all():
                raise ValueError
            if point.get('yaw') is not None and not math.isfinite(float(point['yaw'])):
                raise ValueError
        except (KeyError, TypeError, ValueError, OverflowError):
            result.update(reason='invalid_candidate_point', invalid_point_index=i,
                          source_point_reason=point.get('reason') if isinstance(point, dict) else None)
            return result
        xy.append(pair)
    prediction = predict_polyline(xy, start_xy, speed_mps, dt_s, steps,
        geometry_id=result['geometry_id'], frame_id=result['frame_id'], **options)
    prediction.update(candidate_id=result['candidate_id'], source_reason=result['source_reason'])
    return prediction


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--speed-mps', type=float, default=1.)
    parser.add_argument('--dt-s', type=float, default=.2)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--steps', type=int)
    group.add_argument('--horizon-s', type=float)
    args = parser.parse_args()
    try:
        dt = _number(args.dt_s, 'dt_s', positive=True)
        steps = 20 if args.steps is None else args.steps
        if args.horizon_s is not None:
            ratio = _number(args.horizon_s, 'horizon_s') / dt
            if not math.isfinite(ratio) or ratio > 100000 or abs(ratio-round(ratio)) > 1e-9:
                raise ValueError('horizon_s must be an integer multiple of dt_s')
            steps = round(ratio)
        angle = np.linspace(0., math.pi/2, 25)
        paths = dict(straight=np.array([[0., 0.], [8., 0.]]),
                     curved=np.column_stack((4*np.sin(angle), 4*(1-np.cos(angle)))))
        results = {name: predict_polyline(path, path[0], args.speed_mps, dt, steps,
                    geometry_id='synthetic-demo-v1', frame_id='demo') for name, path in paths.items()}
        for result in results.values():
            if result['status'] == 'rejected':
                raise ValueError(result['reason'])
        output = Path(args.output_dir)
        output.mkdir(parents=True, exist_ok=False)
    except (ValueError, FileExistsError) as exc:
        parser.error(str(exc))
    payload = dict(method=METHOD, results=results,
                   input_paths={name: path.tolist() for name, path in paths.items()},
                   source='synthetic_offline_demo', limitations=list(LIMITATIONS),
                   inspection_only=True, planning_allowed=False)
    with (output/'prediction.json').open('x') as stream:
        json.dump(payload, stream, indent=2, allow_nan=False)
    with (output/'prediction.csv').open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=['case', 't_s', 'x', 'y', 'yaw', 'speed_mps', 'prediction_status'])
        writer.writeheader()
        for name, result in results.items():
            for sample in result['samples']:
                writer.writerow(dict(sample, case=name, prediction_status=result['status']))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import Normalize
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    norm = Normalize(0., max(dt * steps, dt))
    for ax, (name, path) in zip(axes, paths.items()):
        result = results[name]
        samples = result['samples']
        ax.plot(path[:, 0], path[:, 1], '--', color='gray', label='Original polyline')
        scatter = ax.scatter([p['x'] for p in samples], [p['y'] for p in samples],
                             c=[p['t_s'] for p in samples], norm=norm, cmap='viridis', label='Prediction')
        labels = np.unique(np.linspace(0, len(samples)-1, min(5, len(samples))).astype(int)) if args.speed_mps else [0]
        for i in labels:
            p = samples[i]
            ax.annotate(f"{p['t_s']:.2g} s", (p['x'], p['y']), xytext=(3, 7), textcoords='offset points')
        ax.set(title=f"{name}: {result['status']}", xlabel='x [m]', ylabel='y [m]')
        ax.set_aspect('equal', adjustable='datalim')
        ax.grid(alpha=.2)
        ax.legend()
    fig.colorbar(scatter, ax=axes, label='Prediction time [s]')
    fig.suptitle('Offline constant-speed prediction — inspection only')
    fig.savefig(output/'prediction.png', dpi=200)
    plt.close(fig)
    print(json.dumps(dict(output_dir=str(output), status={k: v['status'] for k, v in results.items()})))


if __name__ == '__main__':
    main()
