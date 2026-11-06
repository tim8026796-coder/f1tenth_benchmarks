"""Render two journal assets from saved evidence; never invoke a planner."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Patch
import numpy as np

CASES = ['A_far_centerline', 'B_stationary_inside_candidate_region',
         'C_crossing_different_arrival', 'D_incomplete_opponent', 'E_saved_terminal_rejection']
STATES = ['no_contact_at_samples', 'contact_detected', 'unknown', 'rejected']
COLORS = ['#D7E8F5', '#F0A5A5', '#F6DB8C', '#D2D5D9']


def load_matrix(source):
    """Cross-check every CSV cell against stored JSON; never repair evidence."""
    with (source/'all_cases.csv').open(newline='', encoding='utf-8') as f:
        rows = [{k: json.loads(v) for k, v in row.items()} for row in csv.DictReader(f)]
    if len(rows) != 25:
        raise ValueError('Expected 25 candidate rows')
    keyed = {(r['scenario'], r['candidate_id']): r for r in rows}
    if len(keyed) != 25:
        raise ValueError('Duplicate candidate row')
    matrix = []
    for case in CASES:
        data = json.loads((source/'results'/case/'summary.json').read_text())
        scenario = json.loads((source/'scenarios'/(case+'.json')).read_text())
        if data['scenario'] != scenario:
            raise ValueError('Scenario differs: '+case)
        if [r['candidate_id'] for r in data['paths']] != [1, 2, 3, 4, 5]:
            raise ValueError('Invalid saved candidate IDs')
        values = []
        for row in data['paths']:
            csv_row = keyed[(case, row['candidate_id'])]
            for key, value in csv_row.items():
                if key != 'scenario' and row[key] != value:
                    raise ValueError('CSV/JSON mismatch: '+case+'/'+key)
            values.append(STATES.index(row['outcome']))
        matrix.append(values)
    expected = [[0]*5, [0,0,0,1,1], [0]*5, [2]*5, [3]*5]
    if matrix != expected:
        raise ValueError('Saved outcomes differ from the requested journal reference; stop')
    return np.array(matrix)


def pipeline(output):
    fig, ax = plt.subplots(figsize=(15, 9))
    fig.subplots_adjust(left=.025, right=.975, top=.91, bottom=.06)
    ax.set(xlim=(0,1), ylim=(0,1)); ax.axis('off')
    def box(x,y,w,h,label,color='#F1F5F9',dash=False,size=11):
        patch=FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.008,rounding_size=0.012',
            edgecolor='#425466',facecolor=color,linewidth=1.5,linestyle='--' if dash else '-')
        ax.add_patch(patch);ax.text(x+w/2,y+h/2,label,ha='center',va='center',fontsize=size)
    def arrow(points,label=None,color='#425466',dash=False,label_xy=None):
        for a,b in zip(points[:-2],points[1:-1]):
            ax.plot([a[0],b[0]],[a[1],b[1]],color=color,lw=1.4,ls='--' if dash else '-')
        ax.add_patch(FancyArrowPatch(points[-2],points[-1],arrowstyle='-|>',mutation_scale=14,
                                    color=color,linewidth=1.4,linestyle='--' if dash else '-'))
        if label:ax.text(*label_xy,label,fontsize=9,ha='center',va='center',color=color,
                        bbox=dict(facecolor='white',edgecolor='none',pad=1.5))
    # Adapter encloses the orchestration; it is not a post-Collision stage.
    ax.add_patch(FancyBboxPatch((.035,.23),.93,.54,boxstyle='round,pad=0.008',
                               facecolor='#F8FBFE',edgecolor='#286090',linewidth=2))
    ax.text(.50,.705,'Adapter V1: evaluate_plan_candidates()',ha='center',fontsize=15,weight='bold',color='#24577F')
    box(.04,.83,.28,.13,'V5.2 / make_plan()\nSaved plan: C1–C5\nNo regeneration in this workflow',size=10.5)
    box(.35,.83,.30,.13,'Saved State / Session Metadata\nSource pose, speed, time\nGeometry ID and frame',size=10.5)
    box(.70,.83,.27,.13,'External Opponent Prediction\nGenerated once outside Adapter\nHypothetical path + provenance',color='#FFF4DC',size=10)
    box(.075,.49,.24,.16,'Validate shared data contract\nFive IDs; source pose / time\nFrame, grid and body dimensions\nShared opponent precheck',size=10.5)
    box(.375,.49,.23,.16,'Prediction V1\npredict_candidate()\nOne ego prediction per candidate',size=10.5)
    box(.67,.49,.26,.16,'Collision V1\nevaluate_collision()\nAligned discrete-time contact',size=10.5)
    box(.34,.265,.39,.13,'Standardized results: C1–C5\nNative / prediction / collision status\nCompleteness, distance, contact time, sources',color='#E2EDF6',size=10.5)
    arrow([(.18,.83),(.18,.65)])
    arrow([(.50,.83),(.50,.795),(.26,.795),(.26,.65)])
    arrow([(.835,.83),(.835,.785),(.11,.785),(.11,.65)],color='#A56B13')
    arrow([(.315,.57),(.375,.57)],'candidate',label_xy=(.345,.595))
    arrow([(.605,.57),(.67,.57)],'ego samples',label_xy=(.638,.605))
    arrow([(.835,.83),(.835,.65)],'same opponent\nreused',color='#A56B13',label_xy=(.9,.745))
    arrow([(.795,.49),(.795,.33),(.73,.33)])
    arrow([(.49,.49),(.49,.395)])
    ax.text(.51,.445,'Native / prediction rejection',fontsize=9,color='#425466')
    arrow([(.195,.49),(.195,.33),(.34,.33)],'Batch rejection\nrecords',label_xy=(.19,.415))
    box(.34,.04,.39,.115,'Future Cost Evaluation\nNOT IMPLEMENTED',color='#FAFAFA',dash=True,size=12)
    arrow([(.535,.265),(.535,.155)],'future interface only',dash=True,label_xy=(.66,.195))
    fig.suptitle('Five-Candidate Prediction–Collision Pipeline',fontsize=20,weight='bold',y=.97)
    fig.text(.5,.025,'Inspection only; planning not allowed. No ranking, selection or control commands.',ha='center',fontsize=11)
    fig.savefig(output/'01_prediction_collision_pipeline.png',dpi=250,facecolor='white');plt.close(fig)


def outcome_matrix(output, matrix):
    from matplotlib.colors import ListedColormap, BoundaryNorm
    fig,ax=plt.subplots(figsize=(12,7.5));fig.subplots_adjust(left=.31,right=.97,top=.84,bottom=.27)
    ax.imshow(matrix,cmap=ListedColormap(COLORS),norm=BoundaryNorm([-.5,.5,1.5,2.5,3.5],4),aspect='auto')
    ax.set_xticks(range(5));ax.set_xticklabels(['C1','C2','C3','C4','C5'],fontsize=14,weight='bold')
    ax.xaxis.tick_top();ax.tick_params(axis='both',length=0,pad=12)
    ax.set_yticks(range(5));ax.set_yticklabels(['A — Far opponent','B — Stationary opponent',
        'C — Crossing at different times','D — Incomplete prediction','E — Native rejection'],fontsize=12)
    short=['NO CONTACT\n(at samples)','CONTACT','UNKNOWN','REJECTED']
    for i in range(5):
        for j in range(5):ax.text(j,i,short[matrix[i,j]],ha='center',va='center',fontsize=11,weight='bold',color='#202A35')
    ax.set_xticks(np.arange(-.5,5,1),minor=True);ax.set_yticks(np.arange(-.5,5,1),minor=True)
    ax.grid(which='minor',color='white',linewidth=3);ax.tick_params(which='minor',length=0)
    for spine in ax.spines.values():spine.set_visible(False)
    fig.suptitle('Offline Evaluation Outcome Matrix',fontsize=20,weight='bold',y=.96)
    fig.legend(handles=[Patch(facecolor=c,edgecolor='#777',label=s) for c,s in zip(COLORS,STATES)],
               loc='lower center',bbox_to_anchor=(.59,.13),ncol=2,fontsize=11,frameon=False)
    fig.text(.5,.08,'ESP geometry + synthetic opponent; offline discrete-time evaluation',ha='center',fontsize=12)
    fig.text(.5,.035,'No contact at samples does not establish continuous safety.\nE uses a saved terminal rejection; its historical archive contains a deliberately modified plan 0.',ha='center',fontsize=10)
    fig.savefig(output/'02_offline_outcome_matrix.png',dpi=250,facecolor='white');plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-dir',default='docs/research/esp_collision_offline_v1')
    parser.add_argument('--output-dir',default='docs/research/journal_2026_11_05')
    args=parser.parse_args();source=Path(args.source_dir);output=Path(args.output_dir)
    matrix=load_matrix(source)  # Validate all saved evidence before writing any image.
    if any((output/name).exists() for name in ['01_prediction_collision_pipeline.png','02_offline_outcome_matrix.png']):
        parser.error('Output figure already exists; choose a new directory')
    output.mkdir(parents=True,exist_ok=True)
    plt.rcParams.update({'font.family':'DejaVu Sans','figure.facecolor':'white','axes.facecolor':'white'})
    pipeline(output);outcome_matrix(output,matrix)
    print('Verified 25 CSV rows against five result JSON and scenario JSON files; two PNG generated.')


if __name__=='__main__':main()
