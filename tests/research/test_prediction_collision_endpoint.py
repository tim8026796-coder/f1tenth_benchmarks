"""Endpoint metadata regressions and latest session-format compatibility only."""
import copy
import math
from types import SimpleNamespace

import numpy as np
import pytest

from f1tenth_benchmarks.research.overtaking.prediction_v1 import predict_polyline, predict_candidate
from f1tenth_benchmarks.research.overtaking.collision_v1 import evaluate_collision
from f1tenth_benchmarks.research.overtaking.session import make_plan, SessionConfig

BODY = dict(length_m=.6, width_m=.8, margin_m=0.)


def predict(length=.3, y=0., speed=.1):
    return predict_polyline([[0., y], [length, y]], [0., y], speed, .1, 30, frame_id='map')


def evaluate(ego, lead):
    return evaluate_collision(ego, lead, common_start_time_s=10., ego_start_time_s=10.,
                              lead_start_time_s=10., ego_body=BODY, lead_body=BODY)


def test_endpoint_metadata_matches_existing_one_ulp_grid_acceptance():
    p = predict()
    assert p['status'] == 'complete' and len(p['samples']) == 31
    assert p['last_sample_t_s'] == p['requested_horizon_s'] == 3.
    assert p['available_until_t_s'] == 3.
    assert p['path_end_time_s'] == .3/.1  # physical estimate stays unchanged
    assert p['last_sample_t_s'] <= np.nextafter(p['path_end_time_s'], math.inf)
    assert p['samples'][-1]['x'] == .3
    assert len({s['x'] for s in p['samples']}) == 31


def test_decimal_endpoint_prediction_to_collision_complete():
    result = evaluate(predict(), predict(y=2.))
    assert result['status'] == 'complete', result['reason']
    assert result['evaluation_complete'] and result['checked_sample_count'] == 31
    assert result['assessment'] == 'no_contact_at_samples'
    assert result['inspection_only'] and not result['planning_allowed']


@pytest.mark.parametrize('length', [.299999, .295, .29, .1,
    np.nextafter(np.nextafter(np.nextafter(.3, 0.), 0.), 0.)])
def test_real_shortage_remains_incomplete_without_endpoint_padding(length):
    p = predict(length=length)
    assert p['status'] == 'path_exhausted'
    assert not p['prediction_complete'] and len(p['samples']) < 31
    assert p['last_sample_t_s'] <= p['available_until_t_s'] < 3.
    assert p['last_sample_t_s'] <= np.nextafter(p['path_end_time_s'], math.inf)
    assert len({s['x'] for s in p['samples']}) == len(p['samples'])
    assert max(s['x'] for s in p['samples']) <= length
    result = evaluate(p, predict(y=2.))
    assert result['status'] == 'incomplete' and result['assessment'] == 'unknown'
    assert not result['evaluation_complete'] and result['unchecked_range_s'] is not None


def test_off_grid_available_range_is_not_rounded_up():
    p = predict(length=.295)
    assert p['available_until_t_s'] == .295/.1
    assert p['last_sample_t_s'] < p['available_until_t_s'] < 3.


@pytest.mark.parametrize('available', [2.8, 3.1])
def test_collision_still_rejects_inconsistent_available_metadata(available):
    p = predict()
    p['available_until_t_s'] = available
    result = evaluate(p, predict(y=2.))
    assert result['status'] == 'rejected'
    assert result['assessment'] == 'unknown'


def test_stationary_endpoint_retains_full_time_grid():
    p = predict_polyline([[0., 0.], [.3, 0.]], [.3, 0.], 0., .1, 30, frame_id='map')
    assert p['status'] == 'complete' and p['available_until_t_s'] == 3.
    assert p['path_end_time_s'] is None
    assert all(s['x'] == .3 for s in p['samples'])


def test_latest_session_event_single_candidate_format_not_batch_adapter():
    state = dict(control_step=20, sim_time_s=10., pose=[0., 0., 0.],
                 speed_mps=.1, valid=True, frenet=dict(valid=True))
    def artificial_generator(geometry, x, y, yaw, **settings):
        return dict(paths=[dict(candidate_id=i, geometry_id=geometry.geometry_id, frame_id='map',
            sampled_valid=True, status='sampled_valid', reason=None,
            points=[dict(x=x, y=y, yaw=yaw, valid=True, t=0., s=0., d=0.),
                    dict(x=.3, y=y, yaw=yaw, valid=True, t=1., s=.3, d=0.)])
            for i in range(1, 6)], frame_id='map', inspection_only=True, planning_allowed=False)
    event = make_plan(SimpleNamespace(geometry_id='synthetic-session'), state,
                      SessionConfig(), 1, generator=artificial_generator)
    candidate = event['paths'][0]
    p = predict_candidate(candidate, event['source_pose'][:2], state['speed_mps'], .1, 30,
                          context=event)
    assert p['status'] == 'complete' and p['samples'][-1]['t_s'] == 3.
    assert candidate['points'][-1]['t'] == 1.  # normalized parameter is not seconds
    result = evaluate_collision(p, predict(y=2.),
        common_start_time_s=event['source_sim_time_s'], ego_start_time_s=event['source_sim_time_s'],
        lead_start_time_s=10., ego_body=BODY, lead_body=BODY)
    assert result['evaluation_complete'] and result['sources']['ego']['candidate_id'] == 1
    rejected = copy.deepcopy(candidate)
    rejected.update(sampled_valid=False, reason='start_pose_mismatch')
    failure = predict_candidate(rejected, [0., 0.], .1, .1, 30, context=event)
    assert failure['reason'] == 'candidate_rejected'
    assert failure['source_reason'] == 'start_pose_mismatch'
