"""Cost input normalization; fixture evaluation happens before the pure call."""
from copy import deepcopy
import json
from pathlib import Path

import pytest

from test_prediction_collision_adapter_v1 import inputs
from test_current_prediction_contract import reference_event, evaluate as reference_evaluate
from f1tenth_benchmarks.research.overtaking.prediction_collision_adapter_v1 import evaluate_plan_candidates
from f1tenth_benchmarks.research.overtaking.prediction_v1 import predict_polyline
from f1tenth_benchmarks.research.overtaking.cost_input_v1 import build_cost_inputs


def example():
    plan, opts = inputs()
    result = evaluate_plan_candidates(plan, **opts)
    return plan, result, dict(source_state=opts['source_state'], session_metadata=opts['session_metadata'])


def test_complete_and_no_mutation():
    p,a,kw=example();before=deepcopy((p,a,kw))
    r=build_cost_inputs(p,a,**kw)
    assert r['contract_valid'] and r['source_contract_valid']
    assert [c['candidate_id'] for c in r['paths']]==[1,2,3,4,5]
    assert all(c['eligible_for_general_comparison'] and c['has_collision_features'] for c in r['paths'])
    assert all(c['comparison_status']=='comparable_complete' and c['exclusion_reason'] is None for c in r['paths'])
    assert not r['plan_content_binding_verified'] and r['source_binding_level']=='metadata_join_only'
    assert (p,a,kw)==before
    json.dumps(r,allow_nan=False)


def test_id_join_not_array_position():
    p,a,kw=example();p['paths'].reverse();a['paths']=[a['paths'][i] for i in [2,0,4,1,3]]
    a['paths'][0]['minimum_center_distance_m']=2.5
    r=build_cost_inputs(p,a,**kw)
    assert [c['candidate_id'] for c in r['paths']]==[5,4,3,2,1]
    assert [c['input_index'] for c in r['paths']]==[0,1,2,3,4]
    assert r['paths'][2]['minimum_center_distance_m']==2.5


@pytest.mark.parametrize('who',['plan','adapter'])
@pytest.mark.parametrize('change',['duplicate','missing','extra','boolean'])
def test_bad_ids(who,change):
    p,a,kw=example();rows=p['paths'] if who=='plan' else a['paths']
    if change=='duplicate':rows[1]['candidate_id']=1
    elif change=='missing':rows.pop()
    elif change=='extra':rows.append(deepcopy(rows[0]))
    else:rows[0]['candidate_id']=True
    r=build_cost_inputs(p,a,**kw)
    assert not r['contract_valid']
    assert all(not c['eligible_for_general_comparison'] for c in r['paths'])
    assert len(r['paths'])==len(p['paths'])


def test_native_rejected_and_tampered_promotion():
    p,opts=inputs();p['paths'][0].update(sampled_valid=False,reason='fit_not_accepted',points=[])
    a=evaluate_plan_candidates(p,**opts);kw=dict(source_state=opts['source_state'],session_metadata=opts['session_metadata'])
    c=build_cost_inputs(p,a,**kw)['paths'][0]
    assert c['contract_valid'] and c['outcome']=='rejected'
    assert c['exclusion_reason']=='fit_not_accepted' and not c['has_collision_features']
    a['paths'][0]['outcome']='no_contact_at_samples'
    c=build_cost_inputs(p,a,**kw)['paths'][0]
    assert not c['contract_valid'] and c['outcome']=='rejected'
    assert not c['eligible_for_general_comparison']


def test_reference_only_latest_make_plan():
    g,p,state=reference_event(.02);a=reference_evaluate(g,p,state)
    kw=dict(source_state=state,session_metadata=dict(geometry_id=g.geometry_id,frame_id='map'))
    c=build_cost_inputs(p,a,**kw)['paths'][2]
    assert c['contract_valid'] and c['outcome']=='reference_only'
    assert c['exclusion_reason']=='reference_not_ego_trajectory'
    assert not c['eligible_for_general_comparison'] and not c['has_collision_features']
    a['paths'][2]['outcome']='no_contact_at_samples'
    c=build_cost_inputs(p,a,**kw)['paths'][2]
    assert not c['contract_valid'] and c['outcome']=='reference_only'


@pytest.mark.parametrize('incomplete',[False,True])
def test_contact_full_and_partial_and_t0(incomplete):
    p,opts=inputs()
    if incomplete:p['paths'][0]['points'][-1]['x']=.75
    opts['opponent_prediction']=predict_polyline([[.5,0.],[10.,0.]],[.5,0.],0.,.5,8,geometry_id='g',frame_id='map')
    a=evaluate_plan_candidates(p,**opts)
    c=build_cost_inputs(p,a,source_state=opts['source_state'],session_metadata=opts['session_metadata'])['paths'][0]
    assert c['contract_valid'] and c['outcome']=='contact_detected'
    assert c['current_contact'] and c['first_contact_grid_t_s']==0.
    assert c['first_future_contact_grid_t_s']==.5 and c['has_collision_features']
    assert c['eligible_for_general_comparison'] is (not incomplete)
    assert c['comparison_status']==('partial_features' if incomplete else 'comparable_complete')
    if incomplete:assert c['unchecked_range_s']==dict(after_t_s=.5,through_t_s=4.)


def test_unknown_partial_has_features_but_not_general_eligibility():
    p,opts=inputs();p['paths'][0]['points'][-1]['x']=.75
    a=evaluate_plan_candidates(p,**opts)
    c=build_cost_inputs(p,a,source_state=opts['source_state'],session_metadata=opts['session_metadata'])['paths'][0]
    assert c['outcome']=='unknown' and c['contract_valid']
    assert c['has_collision_features'] and not c['full_horizon_checked']
    assert not c['eligible_for_general_comparison'] and c['exclusion_reason']=='incomplete_horizon'


@pytest.mark.parametrize('field',['minimum_center_distance_m','minimum_clearance_m','minimum_center_distance_t_s','minimum_clearance_t_s'])
def test_missing_feature_stays_null(field):
    p,a,kw=example();del a['paths'][0][field]
    c=build_cost_inputs(p,a,**kw)['paths'][0]
    assert c[field] is None and c['contract_valid'] and c['full_horizon_checked']
    assert not c['eligible_for_general_comparison'] and c['exclusion_reason']=='missing_collision_metrics'


@pytest.mark.parametrize('field,value',[('plan_id',8),('source_sim_time_s',11.),('source_control_step',5),
    ('geometry_id','other'),('frame_id','odom'),('source_pose',[1.,0.,0.])])
def test_different_source_rejected(field,value):
    p,a,kw=example();a[field]=value
    r=build_cost_inputs(p,a,**kw)
    assert not r['contract_valid'] and all(not c['has_collision_features'] for c in r['paths'])


@pytest.mark.parametrize('change',['native','frame','geometry','time','path_type','outcome','complete','future','nan','negative'])
def test_bad_candidate_contract(change):
    p,a,kw=example();c=a['paths'][0]
    if change=='native':c['native_sampled_valid']=False
    elif change=='frame':c['frame_id']='odom'
    elif change=='geometry':c['geometry_id']='other'
    elif change=='time':c['source_sim_time_s']=11.
    elif change=='path_type':c['path_type']='reference_centerline'
    elif change=='outcome':c['outcome']='safe'
    elif change=='complete':c['evaluation_complete']=False
    elif change=='future':c['first_future_contact_grid_t_s']=0.
    elif change=='nan':c['minimum_center_distance_m']=float('nan')
    else:c['minimum_clearance_m']=-1.
    r=build_cost_inputs(p,a,**kw)
    assert not r['contract_valid'] and r['source_contract_valid']
    assert not r['paths'][0]['eligible_for_general_comparison']
    assert all(c['eligible_for_general_comparison'] for c in r['paths'][1:])
    json.dumps(r,allow_nan=False)


@pytest.mark.parametrize('case',['metadata','body','opponent_time','provenance','state_time','source_speed'])
def test_missing_or_bad_context(case):
    p,a,kw=example()
    if case=='metadata':kw['session_metadata']=None
    elif case=='body':del a['ego_body']
    elif case=='opponent_time':a['opponent_start_time_s']=12.
    elif case=='provenance':a['opponent_provenance']={}
    elif case=='state_time':kw['source_state']['sim_time_s']=12.
    else:kw['source_state']['speed_mps']=2.
    assert not build_cost_inputs(p,a,**kw)['contract_valid']


def test_no_samples_and_no_execution(monkeypatch):
    p,a,kw=example()
    def forbidden(*args,**kwargs):pytest.fail('Pure cost input must not execute evaluation')
    from f1tenth_benchmarks.research.overtaking import prediction_v1,collision_v1,prediction_collision_adapter_v1,robust_adaptive_paths,session
    for module,name in [(prediction_v1,'predict_polyline'),(prediction_v1,'predict_candidate'),(collision_v1,'evaluate_collision'),
                        (prediction_collision_adapter_v1,'evaluate_plan_candidates'),(robust_adaptive_paths,'generate_robust_paths'),(session,'make_plan')]:
        monkeypatch.setattr(module,name,forbidden)
    r=build_cost_inputs(p,a,**kw)
    encoded=json.dumps(r,allow_nan=False)
    assert '"samples"' not in encoded and '"points"' not in encoded
    assert r['inspection_only'] and not r['planning_allowed']
    assert all(c['inspection_only'] and not c['planning_allowed'] for c in r['paths'])


@pytest.mark.parametrize('case',['A_far_centerline','B_stationary_inside_candidate_region','C_crossing_different_arrival',
                                 'D_incomplete_opponent','E_saved_terminal_rejection'])
def test_preserved_esp_summaries(case):
    root=Path('docs/research/esp_collision_offline_v1')
    summary=json.loads((root/'results'/case/'summary.json').read_text())
    adapter=dict(summary['result'],paths=summary['paths'])
    source=root/'sources'/('esp_timeout_mutated_test' if case.startswith('E_') else 'esp_short_preview')
    plans=[json.loads(s) for s in (source/'plans.jsonl').read_text().splitlines()]
    states=[json.loads(s) for s in (source/'states.jsonl').read_text().splitlines()]
    plan=next(p for p in plans if p['plan_id']==1)
    state=next(s for s in states if s['control_step']==plan['source_control_step'])
    metadata=json.loads((source/'metadata.json').read_text())
    r=build_cost_inputs(plan,adapter,source_state=state,session_metadata=metadata)
    assert [c['candidate_id'] for c in r['paths']]==[1,2,3,4,5]
    if case.startswith('E_'):
        assert not r['source_contract_valid']
        assert all(c['outcome']=='rejected' and not c['eligible_for_general_comparison'] for c in r['paths'])
    else:
        assert r['contract_valid']
        assert [c['outcome'] for c in r['paths']]==[c['outcome'] for c in adapter['paths']]
        if case.startswith('D_'):assert not any(c['eligible_for_general_comparison'] for c in r['paths'])
        else:assert all(c['eligible_for_general_comparison'] for c in r['paths'])


def test_unknown_cannot_claim_complete_or_no_contact():
    p,a,kw=example();a['paths'][0]['outcome']='unknown'
    c=build_cost_inputs(p,a,**kw)['paths'][0]
    assert not c['contract_valid'] and not c['eligible_for_general_comparison']


@pytest.mark.parametrize('change',['state_invalid','state_frame','type_origin','flags','contact_unchecked','unknown_with_contact'])
def test_additional_inconsistencies_do_not_gain_comparability(change):
    p,a,kw=example();c=a['paths'][0]
    if change=='state_invalid':kw['source_state']['valid']=False
    elif change=='state_frame':kw['source_state']['frame_id']='odom'
    elif change=='type_origin':c['path_type_origin']='explicit'
    elif change=='flags':c['planning_allowed']=True
    elif change=='contact_unchecked':
        c.update(outcome='contact_detected',collision_assessment='contact_detected',
                 prediction_complete=False,prediction_status='rejected',evaluation_complete=False,
                 collision_status='not_evaluated',checked_range_s=None,current_contact=None,
                 any_contact_in_checked_samples=None)
        for k in ['minimum_center_distance_m','minimum_center_distance_t_s','minimum_clearance_m','minimum_clearance_t_s']:
            c[k]=None
    else:
        c.update(outcome='unknown',collision_assessment='unknown',evaluation_complete=False,collision_status='incomplete',
                 checked_range_s=[0.,1.],unchecked_range_s=dict(after_t_s=1.,through_t_s=4.),first_contact_grid_t_s=.5)
    r=build_cost_inputs(p,a,**kw)
    assert not r['contract_valid'] and not r['paths'][0]['eligible_for_general_comparison']
