"""Original 53 prediction checks, made portable to this worktree."""
import ast
import csv
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest
from PIL import Image

from f1tenth_benchmarks.research.overtaking.prediction_v1 import predict_polyline, predict_candidate

REPO = Path(__file__).resolve().parents[2]
MODULE = 'f1tenth_benchmarks.research.overtaking.prediction_v1'
LINE = [[0., 0.], [10., 0.]]


def test_straight_analytic():
    r = predict_polyline(LINE, [1., 0.], 2., .2, 10)
    assert r['status'] == 'complete' and r['prediction_complete']
    assert len(r['samples']) == 11
    for k, p in enumerate(r['samples']):
        assert [p['t_s'],p['x'],p['y'],p['yaw'],p['speed_mps']] == pytest.approx([k*.2,1.+2*k*.2,0.,0.,2.])
    assert r['inspection_only'] is True and r['planning_allowed'] is False
    json.dumps(r, allow_nan=False)


def test_sparse_dense_same_polyline():
    sparse = [[0,0],[2,0],[2,3],[5,3]]
    dense = [[0,0],[.5,0],[1.,0],[1.5,0],[2,0],[2,1],[2,2],[2,3],[3,3],[4,3],[5,3]]
    a,b=[predict_polyline(p,[.5,0],1.,.25,24)for p in (sparse,dense)]
    assert a['status']==b['status']=='complete'
    for p,q in zip(a['samples'],b['samples']):
        assert list(p.values())==pytest.approx(list(q.values()),abs=1e-12)


def test_duplicate_points_and_zero_speed():
    r=predict_polyline([[0,0],[0,0],[2,0],[2,0],[2,2]], [.5,0],0.,.2,10)
    assert r['status']=='complete' and r['removed_duplicate_points']==2
    assert len(r['samples'])==11 and r['available_until_t_s']==2.
    assert all([p['x'],p['y'],p['speed_mps']]==[.5,0.,0.]for p in r['samples'])
    assert [p['t_s'] for p in r['samples']]==pytest.approx(np.arange(11)*.2)


def test_vertex_yaw_uses_outgoing_segment():
    r=predict_polyline([[0,0],[1,0],[1,1]],[1,0],1.,.5,1)
    assert r['status']=='complete' and r['samples'][0]['yaw']==pytest.approx(math.pi/2)


def test_h_zero():
    r=predict_polyline(LINE,[0,0],1.,.2,0)
    assert r['status']=='complete' and len(r['samples'])==1 and r['samples'][0]['t_s']==0.


def test_start_error_explicit_and_configurable():
    a=predict_polyline(LINE,[0,.04],1.,.2,1)
    assert a['status']=='complete' and a['projection']['error_m']==pytest.approx(.04)
    assert a['projection']['input_xy']==[0.,.04]
    assert a['samples'][0]['y']==0.
    assert predict_polyline(LINE,[0,.04],1.,.2,1,max_start_error_m=.01)['reason']=='start_too_far_from_path'
    assert predict_polyline(LINE,[0,4.],1.,.2,1)['reason']=='start_too_far_from_path'


@pytest.mark.parametrize('path,start', [([[-1,-1],[1,1],[-1,1],[1,-1]],[0,0]),
                                      ([[0,0],[2,0],[2,.02],[0,.02]],[1,.01])])
def test_ambiguous_projection_rejected(path,start):
    r=predict_polyline(path,start,1.,.2,2)
    assert r['reason']=='ambiguous_start_projection' and not r['samples']


def test_exhaustion_off_grid_no_endpoint_padding():
    r=predict_polyline([[0,0],[1,0]],[0,0],2.,.2,10)
    assert r['status']=='path_exhausted' and not r['prediction_complete']
    assert r['available_until_t_s']==.5 and r['path_end_time_s']==.5
    assert r['last_sample_t_s']==.4 and len(r['samples'])==3
    assert r['samples'][-1]['x']==pytest.approx(.8)
    assert r['path_end_xy']==[1.,0.]


def test_exhaustion_on_grid_and_zero_speed_at_end():
    r=predict_polyline([[0,0],[1,0]],[0,0],1.,.25,8)
    assert r['status']=='path_exhausted' and len(r['samples'])==5
    assert r['samples'][-1]['x']==1.
    exact=predict_polyline([[0,0],[1,0]],[0,0],1.,.25,4)
    assert exact['status']=='complete'
    at_end=predict_polyline([[0,0],[1,0]],[1,0],1.,.25,4)
    assert at_end['status']=='path_exhausted' and len(at_end['samples'])==1
    stopped=predict_polyline([[0,0],[1,0]],[1,0],0.,.25,4)
    assert stopped['status']=='complete' and len(stopped['samples'])==5


@pytest.mark.parametrize('path', [[], [[0,0]], [[0,0],[0,0]], [[0,0],[np.nan,1]],
                                [[0,0],[np.inf,1]], [[0,0,0],[1,1,1]], 'bad'])
def test_unusable_paths_rejected(path):
    r=predict_polyline(path,[0,0],1.,.2,2)
    assert r['status']=='rejected' and not r['samples']
    json.dumps(r,allow_nan=False)


@pytest.mark.parametrize('changes', [dict(speed_mps=-1),dict(speed_mps=np.nan),dict(speed_mps=np.inf),
    dict(speed_mps=True),dict(dt_s=0),dict(dt_s=-1),dict(dt_s=np.inf),dict(steps=-1),dict(steps=1.5),
    dict(steps=True),dict(steps=100001),dict(start_xy=[np.nan,0]),dict(start_xy=[0,0,0]),
    dict(max_start_error_m=-1),dict(projection_tolerance_m=np.nan),dict(ambiguity_arc_tolerance_m=-1),
    dict(speed_mps=1e308,dt_s=1e308),dict(speed_mps=1e-320)])
def test_bad_parameters_rejected(changes):
    args=dict(path_xy=LINE,start_xy=[0,0],speed_mps=1.,dt_s=.2,steps=2)
    args.update(changes);r=predict_polyline(**args)
    assert r['status']=='rejected' and r['reason'] and not r['samples']
    json.dumps(r,allow_nan=False)


def candidate():
    return dict(candidate_id=3,geometry_id='source-geometry',frame_id='map',sampled_valid=True,
                reason=None,points=[dict(x=0.,y=0.,yaw=0.,valid=True,t=.9),
                                    dict(x=3.,y=0.,yaw=0.,valid=True,t=.95)])


def test_candidate_metadata_and_t_is_not_time():
    r=predict_candidate(candidate(),[0,0],1.,.2,10)
    assert r['status']=='complete' and r['candidate_id']==3
    assert r['geometry_id']=='source-geometry' and r['frame_id']=='map'
    assert [p['t_s']for p in r['samples']]==pytest.approx(np.arange(11)*.2)
    assert r['samples'][-1]['x']==2.
    assert r['inspection_only'] and not r['planning_allowed']


def test_candidate_rejected_no_promotion():
    c=candidate();c.update(sampled_valid=False,reason='fit_not_accepted')
    r=predict_candidate(c,[0,0],1.,.2,10)
    assert r['status']=='rejected' and r['reason']=='candidate_rejected'
    assert r['source_reason']=='fit_not_accepted' and r['candidate_id']==3 and not r['samples']


@pytest.mark.parametrize('change', ['invalid','missing_xy','nan_xy','nan_yaw','metadata','missing_metadata','missing_id'])
def test_candidate_invalid_samples_and_metadata(change):
    c=candidate();context={}
    if change=='invalid':c['points'][1].update(valid=False,reason='occupied')
    elif change=='missing_xy':c['points'][1].pop('x')
    elif change=='nan_xy':c['points'][1]['x']=np.nan
    elif change=='nan_yaw':c['points'][1]['yaw']=np.nan
    elif change=='metadata':context={'geometry_id':'different'}
    elif change=='missing_metadata':c.pop('frame_id')
    else:c.pop('candidate_id')
    r=predict_candidate(c,[0,0],1.,.2,2,context=context)
    assert r['status']=='rejected' and not r['samples']
    if change=='invalid':assert r['source_point_reason']=='occupied'


def test_context_metadata_inheritance():
    c=candidate();c.pop('geometry_id');c.pop('frame_id')
    r=predict_candidate(c,[0,0],1.,.2,2,context={'geometry_id':'inherited','frame_id':'map'})
    assert r['status']=='complete' and r['geometry_id']=='inherited'


def test_python39_syntax_and_import_no_files(tmp_path):
    source=REPO/'f1tenth_benchmarks/research/overtaking/prediction_v1.py'
    ast.parse(source.read_text(),feature_version=(3,9))
    env=dict(os.environ,PYTHONPATH=str(REPO),PYTHONDONTWRITEBYTECODE='1')
    result=subprocess.run([sys.executable,'-B','-c',f'import {MODULE}'],cwd=tmp_path,env=env,capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    assert result.stdout=='' and list(tmp_path.iterdir())==[]


@pytest.mark.parametrize('arguments', [[],['--speed-mps','0','--steps','2'],['--horizon-s','20'],['--steps','0']])
def test_cli_outputs_and_no_overwrite(tmp_path,arguments):
    output=tmp_path/'demo';cmd=[sys.executable,'-B','-m',MODULE,'--output-dir',str(output)]+arguments
    env=dict(os.environ,PYTHONPATH=str(REPO),MPLCONFIGDIR='/tmp/prediction-mpl',XDG_CACHE_HOME='/tmp/prediction-cache')
    r=subprocess.run(cmd,cwd=REPO,env=env,capture_output=True,text=True);assert r.returncode==0,r.stderr
    assert {p.name for p in output.iterdir()}=={'prediction.json','prediction.csv','prediction.png'}
    data=json.loads((output/'prediction.json').read_text());assert not data['planning_allowed']
    assert len(list(csv.DictReader((output/'prediction.csv').open())))>=2
    with Image.open(output/'prediction.png')as im:im.load();assert im.info['dpi'][0]>=199
    before={p.name:p.read_bytes()for p in output.iterdir()}
    duplicate=subprocess.run(cmd,cwd=REPO,env=env,capture_output=True,text=True)
    assert duplicate.returncode!=0 and before=={p.name:p.read_bytes()for p in output.iterdir()}


@pytest.mark.parametrize('arguments', [['--dt-s','0'],['--horizon-s','.3','--dt-s','.2'],['--speed-mps','nan']])
def test_cli_invalid_inputs_no_artifacts(tmp_path,arguments):
    out=tmp_path/'absent'
    r=subprocess.run([sys.executable,'-B','-m',MODULE,'--output-dir',str(out)]+arguments,
                     cwd=REPO,env=dict(os.environ,PYTHONPATH=str(REPO)),capture_output=True,text=True)
    assert r.returncode!=0 and not out.exists()
