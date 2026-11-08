"""Latest make_plan/reference semantics; no optimizer or control model."""
from copy import deepcopy
from types import SimpleNamespace
import math
import json
from pathlib import Path

import pytest
import yaml

from test_reference_path import reference_fixture, event_for
from f1tenth_benchmarks.research.overtaking.prediction_v1 import predict_polyline, predict_candidate
from f1tenth_benchmarks.research.overtaking.prediction_collision_adapter_v1 import evaluate_plan_candidates
from f1tenth_benchmarks.research.overtaking.session import SessionConfig, make_plan
from f1tenth_benchmarks.research.overtaking.millimetre_domain import vehicle_clearance_profile

BODY = dict(length_m=.51,width_m=.27,margin_m=0.)


def reference_event(offset):
    geometry,source,projection,path = reference_fixture()
    source = [1.5,3.+offset,0.]
    projection = geometry.to_frenet(source[:2],yaw=source[2])
    origin = projection['s_wrapped']
    path.update(reference_s_origin=origin)
    path['points'] = [dict(x=float(geometry.reference(origin+t)['xy'][0]),
                           y=float(geometry.reference(origin+t)['xy'][1]),
                           yaw=float(geometry.reference(origin+t)['psi']),valid=True,t=t/3.) for t in [0.,1.,2.,3.]]
    event = event_for(path,geometry,source,projection)
    state = dict(control_step=0,sim_time_s=0.,pose=source,speed_mps=1.,valid=True)
    return geometry,event,state


def evaluate(geometry,event,state,body=BODY):
    opponent = predict_polyline([[0.,10.],[10.,10.]],[0.,10.],1.,.5,4,
                                geometry_id='synthetic-opponent',frame_id='map')
    return evaluate_plan_candidates(event,source_state=state,
        session_metadata=dict(geometry_id=geometry.geometry_id,frame_id='map'),
        opponent_prediction=opponent,opponent_start_time_s=0.,
        opponent_provenance=dict(kind='synthetic',description='Separated artificial opponent'),
        ego_body=body,lead_body=BODY,dt_s=.5,steps=4)


@pytest.mark.parametrize('offset',[0.,.02,.12])
def test_native_reference_valid_but_not_ego_prediction(offset):
    g,event,state=reference_event(offset);path=event['paths'][2]
    assert path['sampled_valid'] and path['reference_contract_check']['valid']
    xy=[[p['x'],p['y']] for p in path['points']]
    raw=predict_polyline(xy,state['pose'][:2],1.,.5,4,geometry_id=g.geometry_id,frame_id='map')
    assert raw['status']==('complete' if offset<=.05 else 'rejected')
    result=predict_candidate(path,state['pose'][:2],1.,.5,4,context=dict(geometry_id=g.geometry_id,frame_id='map'))
    assert result['status']=='rejected'
    assert result['reason']=='reference_path_not_predictable_as_ego_trajectory'


@pytest.mark.parametrize('offset',[0.,.02,.12])
def test_adapter_reference_only_is_not_no_contact(offset):
    g,event,state=reference_event(offset)
    r=evaluate(g,event,state);c=r['paths'][2]
    assert c['native_sampled_valid'] and c['outcome']=='reference_only'
    assert c['path_type']=='reference_centerline'
    assert c['collision_assessment']=='unknown' and not c['evaluation_complete']
    assert c['minimum_clearance_m'] is None and c['collision_status']=='not_evaluated'
    assert all(p['outcome']=='rejected' for p in r['paths'] if p['candidate_id']!=3)
    json.dumps(r,allow_nan=False)


def test_native_rejected_reference_keeps_native_reason():
    g,event,state=reference_event(.02)
    event['paths'][2].update(sampled_valid=False,reason='reference_tracking_envelope_exceeded')
    c=evaluate(g,event,state)['paths'][2]
    assert c['outcome']=='rejected' and c['prediction_reason']=='candidate_rejected'
    assert c['native_reason']=='reference_tracking_envelope_exceeded'


@pytest.mark.parametrize('path_type',['state_matched_trajectory',None])
@pytest.mark.parametrize('mode',['optimized','direct_then_repair','width_fraction_then_repair'])
def test_new_modes_do_not_change_time_semantics(path_type,mode):
    g,source,_,_=reference_fixture()
    def generator(*args,**kwargs):
        paths=[]
        for i in range(1,6):
            c=dict(candidate_id=i,geometry_id=g.geometry_id,frame_id='map',sampled_valid=True,reason=None,
                points=[dict(x=source[0],y=source[1],yaw=source[2],valid=True,t=0.),
                        dict(x=source[0]+3,y=source[1],yaw=source[2],valid=True,t=1.)])
            if path_type:c['path_type']=path_type
            paths.append(c)
        return dict(paths=paths,frame_id='map')
    state=dict(control_step=0,sim_time_s=0.,pose=source,speed_mps=1.,valid=True,frenet=dict(valid=True))
    event=make_plan(g,state,SessionConfig(query_mode='reused_exact',generation_mode=mode),0,generator=generator)
    assert event['sampled_valid_count']==5
    result=evaluate(g,event,state)
    assert all(c['outcome']=='no_contact_at_samples' for c in result['paths'])
    assert all(c['prediction']['samples'][-1]['t_s']==2. for c in result['paths'])


def test_unknown_path_type_rejected():
    g,event,state=reference_event(.02);c=event['paths'][2]
    c['path_type']='future_unspecified_reference'
    r=predict_candidate(c,state['pose'][:2],1.,.5,4,context=dict(geometry_id=g.geometry_id,frame_id='map'))
    assert r['status']=='rejected' and r['reason']=='unsupported_candidate_path_type'


def test_missing_body_still_rejected():
    g,event,state=reference_event(.02)
    assert evaluate(g,event,state,body=None)['batch_status']=='rejected'


def test_current_dimensions_and_margin_are_distinct():
    sim=yaml.safe_load(Path('params/simulator_params.yaml').read_text())
    vehicle=yaml.safe_load(Path('params/vehicle_params.yaml').read_text())
    profile=vehicle_clearance_profile(SimpleNamespace(**sim))
    assert (sim['vehicle_length'],sim['vehicle_width'])==(.51,.27)
    assert profile['centre_prefilter_margin_m']==pytest.approx(.185)
    assert math.hypot(.51,.27)/2==pytest.approx(.28853076092507017)
    assert vehicle['wheelbase']==.33


def test_actual_millimetre_generator_c3_is_reference_only():
    from test_mm_bspline_repair import fixture
    from f1tenth_benchmarks.research.run_mm_lap_comparison import DomainGenerator
    domain,g,state=fixture(narrow=False)
    state['speed_mps']=1.
    event=make_plan(g,state,SessionConfig(safety_margin_m=.15),0,generator=DomainGenerator(domain))
    assert event['paths'][2]['sampled_valid']
    assert event['paths'][2]['path_type']=='reference_centerline'
    assert event['result']['fallback_candidate_id']==3
    result=evaluate(g,event,state)
    assert [c['candidate_id'] for c in result['paths']]==[1,2,3,4,5]
    assert result['paths'][2]['outcome']=='reference_only'
    assert not result['paths'][2]['evaluation_complete']
