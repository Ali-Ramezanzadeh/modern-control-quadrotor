"""Independent hover-model algebra; does not execute MATLAB/Simulink.
Requires Python 3 and NumPy. Ranks use exact rational elimination.
"""
from pathlib import Path
from fractions import Fraction as F
import json
import numpy as np

def zeros(r,c): return np.full((r,c),F(0),dtype=object)
def rank(M):
    a=[list(row) for row in M];r=0
    for j in range(len(a[0])):
        p=next((i for i in range(r,len(a)) if a[i][j]),None)
        if p is None: continue
        a[r],a[p]=a[p],a[r];v=a[r][j];a[r]=[x/v for x in a[r]]
        for i in range(r+1,len(a)):
            v=a[i][j]
            if v:a[i]=[x-v*y for x,y in zip(a[i],a[r])]
        r+=1
        if r==len(a):break
    return r
def power(a,k):return np.linalg.matrix_power(a,k)
A=zeros(12,12)
for i,j,v in [(0,1,1),(1,7,F('9.81')),(2,3,1),(3,6,-F('9.81')),(4,5,1),(6,9,1),(7,10,1),(8,11,1)]:A[i,j]=F(v)
C=zeros(4,12)
for i,j in enumerate([0,2,4,8]):C[i,j]=F(1)
results={'method':'Independent reconstruction from live-script equations; no MATLAB/Simulink execution. Exact rational matrix ranks.', 'numpy_version':np.__version__, 'phases':{}}
for phase,ix,iz in [('phase-01',F('.0035'),F('.005')),('phase-02',F('.0085532'),F('.01476')),('phase-03',F('.0085532'),F('.01476'))]:
    B=zeros(12,4)
    for i,j,v in [(5,0,1),(9,1,1/ix),(10,2,1/ix),(11,3,1/iz)]:B[i,j]=F(v)
    co=rank(np.hstack([power(A,k)@B for k in range(12)]))
    ob=rank(np.vstack([C@power(A,k) for k in range(12)]))
    item={'state_order':12,'controllability_rank_exact':co,'observability_rank_exact':ob,'A_power_4_is_zero':bool(np.all(power(A,4)==0)),'A_power_3_rank_exact':rank(power(A,3))}
    coeffs=[C@power(A,k)@B for k in range(4)]
    item['transfer_matrix']=[[' + '.join(f'({coeffs[k][i,j]})/s^{k+1}' for k in range(4) if coeffs[k][i,j]) or '0' for j in range(4)] for i in range(4)]
    assert co==ob==12 and item['A_power_4_is_zero'] and item['A_power_3_rank_exact']==2
    if phase!='phase-01':
        Ai=np.block([[A,zeros(12,4)],[-C,zeros(4,4)]]);Bi=np.vstack([B,zeros(4,4)])
        item['integral_augmented_controllability_rank_exact']=rank(np.hstack([power(Ai,k)@Bi for k in range(16)]))
        assert item['integral_augmented_controllability_rank_exact']==16
    results['phases'][phase]=item
target=Path(__file__).with_name('hover-check-results.json')
target.write_text(json.dumps(results,indent=2)+'\n',encoding='utf-8')
print(json.dumps(results,indent=2))
