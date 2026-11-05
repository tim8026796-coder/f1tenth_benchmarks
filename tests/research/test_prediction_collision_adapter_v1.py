"""Synthetic offline contracts; not an ESP two-vehicle simulation."""
import copy
import json
from types import SimpleNamespace

import pytest

from f1tenth_benchmarks.research.overtaking import prediction_collision_adapter_v1 as adapter
from f1tenth_benchmarks.research.overtaking.prediction_v1 import predict_polyline
from f1tenth_benchmarks.research.overtaking.session import make_plan, SessionConfig

BODY = dict(length_m=.6, width_m=.8, margin_m=0.)


def inputs():
    paths = [dict(candidate_id=i, sampled_valid=True, reason=None, geometry_id='g', frame_id='map',
        points=[dict(x=0., y=0., yaw=0., valid=True, t=0.),
                dict(x=10., y=0., yaw=0., valid=True, t=1.)]) for i in range(1, 6)]
    plan = dict(plan_id=7, status='completed', paths=paths, source_pose=[0., 0., 0.],
                source_sim_time_s=10., available_sim_time_s=99., source_control_step=4, geometry_id='g')
    options = dict(source_state=dict(valid=True, pose=[0., 0., 0.], speed_mps=1.,
                                     sim_time_s=10., control_step=4),
                   session_metadata=dict(geometry_id='g', frame_id='map'),
                   opponent_prediction=predict_polyline([[0., 3.], [10., 3.]], [0., 3.], 1., .5, 8,
                                                        geometry_id='opponent_route', frame_id='map'),
                   opponent_start_time_s=10., opponent_provenance=dict(kind='synthetic', description='Artificial parallel case'),
                   ego_body=BODY, lead_body=BODY, dt_s=.5, steps=8)
    return plan, options


def run(plan=None, options=None):
    p, opts = inputs()
    return adapter.evaluate_plan_candidates(p if plan is None else plan, **(opts if options is None else options))


def test_all_five_and_no_mutation():
    p, opts = inputs()
    before = copy.deepcopy((p, opts))
    r = run(p, opts)
    assert r['batch_status'] == 'evaluated' and len(r['paths']) == 5
    assert [c['candidate_id'] for c in r['paths']] == [1, 2, 3, 4, 5]
    assert all(c['outcome'] == 'no_contact_at_samples' and c['evaluation_complete'] for c in r['paths'])
    assert (p, opts) == before
    assert all(c['source_sim_time_s'] == 10. for c in r['paths'])  # not available=99
    assert r['opponent_prediction']['geometry_id'] == 'opponent_route'


def test_order_preserved_and_metadata_inherited():
    p, opts = inputs()
    p['paths'].reverse()
    for c in p['paths']:
        del c['geometry_id']; del c['frame_id']
    assert [c['candidate_id'] for c in run(p, opts)['paths']] == [5, 4, 3, 2, 1]


def test_native_rejection_not_promoted():
    p, opts = inputs()
    p['paths'][1].update(sampled_valid=False, reason='fit_not_accepted', points=[])
    c = run(p, opts)['paths'][1]
    assert c['outcome'] == 'rejected' and c['native_reason'] == 'fit_not_accepted'
    assert c['prediction_status'] == 'rejected' and c['reason'] == 'candidate_rejected'
    assert c['collision_status'] == 'not_evaluated' and c['collision_assessment'] == 'unknown'
    assert c['minimum_clearance_m'] is None


def test_only_one_candidate_contacts():
    p, opts = inputs()
    opts['opponent_prediction'] = predict_polyline([[3., 0.], [10., 0.]], [3., 0.], 0., .5, 8,
                                                 geometry_id='g', frame_id='map')
    for c in p['paths'][1:]:
        c['points'][1].update(x=0., y=10.)
    rows = run(p, opts)['paths']
    assert [c['outcome'] for c in rows] == ['contact_detected'] + ['no_contact_at_samples']*4
    assert rows[0]['first_future_contact_grid_t_s'] == 2.


def test_crossing_but_different_arrival_times():
    p, opts = inputs()
    for c in p['paths']:
        c['points'] = [dict(x=-2., y=0., valid=True), dict(x=10., y=0., valid=True)]
    p['source_pose'] = opts['source_state']['pose'] = [-2., 0., 0.]
    opts['opponent_prediction'] = predict_polyline([[0., -5.], [0., 10.]], [0., -5.], 1., .5, 12,
                                                 geometry_id='g', frame_id='map')
    opts['steps'] = 12
    assert all(c['outcome'] == 'no_contact_at_samples' for c in run(p, opts)['paths'])


@pytest.mark.parametrize('who', ['ego', 'opponent'])
def test_incomplete_is_unknown(who):
    p, opts = inputs()
    if who == 'ego':
        p['paths'][0]['points'][-1]['x'] = .75
    else:
        opts['opponent_prediction'] = predict_polyline([[0., 3.], [.75, 3.]], [0., 3.], 1., .5, 8,
                                                     geometry_id='g', frame_id='map')
    c = run(p, opts)['paths'][0]
    assert c['outcome'] == 'unknown' and c['collision_status'] == 'incomplete'
    assert not c['evaluation_complete'] and c['checked_range_s'] == [0., .5]
    assert c['unchecked_range_s']['through_t_s'] == 4.


def test_incomplete_contact_stays_incomplete():
    p, opts = inputs()
    p['paths'][0]['points'][-1]['x'] = .75
    opts['opponent_prediction'] = predict_polyline([[0., 0.], [10., 0.]], [0., 0.], 0., .5, 8,
                                                 geometry_id='g', frame_id='map')
    c = run(p, opts)['paths'][0]
    assert c['outcome'] == 'contact_detected' and not c['evaluation_complete']
    assert c['current_contact'] and c['unchecked_range_s'] is not None


@pytest.mark.parametrize('field', ['source_state', 'session_metadata', 'opponent_prediction',
    'opponent_start_time_s', 'opponent_provenance', 'ego_body', 'lead_body', 'dt_s', 'steps'])
def test_missing_required_parameter_rejects_all(field):
    p, opts = inputs(); del opts[field]
    r = run(p, opts)
    assert r['batch_status'] == 'rejected' and len(r['paths']) == 5
    assert all(c['outcome'] == 'rejected' and c['collision_assessment'] == 'unknown' for c in r['paths'])


@pytest.mark.parametrize('field', ['source_pose', 'source_sim_time_s', 'source_control_step', 'geometry_id'])
def test_missing_plan_source(field):
    p, opts = inputs(); del p[field]
    assert run(p, opts)['batch_status'] == 'rejected'


@pytest.mark.parametrize('change', ['time', 'control_step', 'pose', 'speed', 'valid', 'frame', 'geometry'])
def test_source_state_mismatch(change):
    p, opts = inputs(); s = opts['source_state']
    if change == 'time': s['sim_time_s'] = 11.
    elif change == 'control_step': s['control_step'] = 5
    elif change == 'pose': s['pose'] = [1., 0., 0.]
    elif change == 'speed': s['speed_mps'] = float('nan')
    elif change == 'valid': s['valid'] = False
    elif change == 'frame': s['frame_id'] = 'odom'
    else: s['geometry_id'] = 'other'
    r = run(p, opts)
    assert r['batch_status'] == 'rejected'
    json.dumps(r, allow_nan=False)


@pytest.mark.parametrize('change', ['time', 'frame', 'dt', 'rejected', 'missing_sample', 'invalid_xy'])
def test_bad_opponent(change):
    p, opts = inputs(); op = opts['opponent_prediction']
    if change == 'time': opts['opponent_start_time_s'] = 11.
    elif change == 'frame': op['frame_id'] = 'odom'
    elif change == 'dt': opts['dt_s'] = .25
    elif change == 'rejected': op.update(status='rejected', reason='start_too_far_from_path')
    elif change == 'missing_sample': op['samples'].pop(1)
    else: op['samples'][1]['x'] = float('inf')
    r = run(p, opts)
    assert r['batch_status'] == 'rejected'
    assert all(c['current_contact'] is None for c in r['paths'])
    json.dumps(r, allow_nan=False)


@pytest.mark.parametrize('change', ['frame', 'geometry', 'point'])
def test_individual_invalid_candidate(change):
    p, opts = inputs()
    if change == 'frame': p['paths'][0]['frame_id'] = 'odom'
    elif change == 'geometry': p['paths'][0]['geometry_id'] = 'other'
    else: p['paths'][0]['points'][1].update(valid=False, reason='occupied')
    rows = run(p, opts)['paths']
    assert rows[0]['outcome'] == 'rejected'
    assert all(c['outcome'] == 'no_contact_at_samples' for c in rows[1:])


@pytest.mark.parametrize('change', ['duplicate', 'short', 'extra', 'bad_id'])
def test_bad_envelope_preserves_supplied_slots(change):
    p, opts = inputs()
    if change == 'duplicate': p['paths'][1]['candidate_id'] = 1
    elif change == 'short': p['paths'].pop()
    elif change == 'extra': p['paths'].append(copy.deepcopy(p['paths'][0]))
    else: p['paths'][0]['candidate_id'] = True
    r = run(p, opts)
    assert r['batch_status'] == 'rejected'
    assert len(r['paths']) == len(p['paths'])
    assert [c['candidate_id'] for c in r['paths']] == [c['candidate_id'] for c in p['paths']]


def test_missing_paths_has_explicit_batch_rejection():
    assert run({})['reason'] == 'missing_plan_paths'
    assert run({})['paths'] == []


def test_stationary_initial_contact():
    p, opts = inputs(); opts['source_state']['speed_mps'] = 0.
    opts['opponent_prediction'] = predict_polyline([[.5, 0.], [10., 0.]], [.5, 0.], 0., .5, 8,
                                                 geometry_id='g', frame_id='map')
    c = run(p, opts)['paths'][0]
    assert c['current_contact'] and c['first_contact_grid_t_s'] == 0.
    assert c['first_future_contact_grid_t_s'] == .5 and c['evaluation_complete']


def test_json_and_complete_keys():
    r = json.loads(json.dumps(run(), allow_nan=False))
    keys = {'candidate_id','native_sampled_valid','native_reason','prediction_status','prediction_complete',
            'collision_status','collision_assessment','evaluation_complete','checked_range_s','unchecked_range_s',
            'first_future_contact_grid_t_s','minimum_center_distance_m','minimum_clearance_m',
            'source_sim_time_s','geometry_id','frame_id','inspection_only','planning_allowed'}
    for c in r['paths']:
        assert keys <= c.keys() and c['inspection_only'] and not c['planning_allowed']
    assert r['inspection_only'] and not r['planning_allowed']


def test_public_session_and_shared_opponent_not_repredicted(monkeypatch):
    p, opts = inputs()
    state = opts['source_state']
    state['frenet'] = dict(valid=True)
    def generator(geometry, x, y, yaw, **kwargs):
        return dict(paths=copy.deepcopy(p['paths']), frame_id='map')
    event = make_plan(SimpleNamespace(geometry_id='g'), state, SessionConfig(), 7, generator=generator)
    calls = []
    original = adapter.predict_candidate
    def record(*args, **kwargs):
        calls.append(args[0]['candidate_id'])
        return original(*args, **kwargs)
    monkeypatch.setattr(adapter, 'predict_candidate', record)
    r = run(event, opts)
    assert calls == [1, 2, 3, 4, 5] and r['batch_status'] == 'evaluated'
    assert all(c['evaluation_complete'] for c in r['paths'])
    assert r['opponent_prediction'] == opts['opponent_prediction']


@pytest.mark.parametrize('body', [dict(length_m=0., width_m=.8, margin_m=0.),
    dict(length_m=.6, width_m=-.8, margin_m=0.), dict(length_m=.6, width_m=.8, margin_m=-.1),
    dict(length_m=.6, width_m=.8, margin_m=float('nan')), {}])
def test_illegal_body_is_not_no_contact(body):
    p, opts = inputs(); opts['ego_body'] = body
    r = run(p, opts)
    assert r['batch_status'] == 'rejected'
    assert all(c['outcome'] == 'rejected' for c in r['paths'])
    json.dumps(r, allow_nan=False)


@pytest.mark.parametrize('dt,steps', [(0.,8),(-.5,8),(float('inf'),8),(.5,1.5),(.5,True)])
def test_illegal_grid(dt,steps):
    p, opts = inputs(); opts.update(dt_s=dt, steps=steps)
    assert run(p, opts)['batch_status'] == 'rejected'


def test_missing_opponent_geometry_is_rejected():
    p, opts = inputs(); del opts['opponent_prediction']['geometry_id']
    assert run(p, opts)['reason'] == 'missing_opponent_geometry_id'
