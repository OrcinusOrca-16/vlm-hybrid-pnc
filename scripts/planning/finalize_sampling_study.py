"""Targeted final ablations and selected-path dense verification."""
import json
from dataclasses import replace
from itertools import product
from collections import Counter
from scripts.planning.run_sampling_development_study import OUT, scenarios, summarize, measure


def main():
    ss=scenarios()
    pools={s.name:json.loads((OUT/(s.name+'.json')).read_text()) for s in ss}
    lat_profiles={
        'd05':tuple(i/2 for i in range(-4,5)),
        'nonuniform':(-2.,-1.5,-1.,-.8,-.4,0.,.4,.8,1.,1.5,2.),
        'nonuniform12':(-2.,-1.5,-1.2,-1.,-.8,-.4,0.,.4,.8,1.,1.2,1.5,2.),
        'small':(-2.,-1.5,-1.,-.8,0.,.8,1.,1.5,2.),
    }
    length_profiles={'base':(6.,10.,14.,18.,22.),'pruned':(6.,10.,14.,18.),
                     'tiny':(6.,14.,18.),'short':(6.,10.,14.)}
    slope_profiles={'strict':(0.,),'s005':(-.05,0.,.05),'s0075':(-.075,0.,.075),
                    's01':(-.1,0.,.1),'s015':(-.15,-.1,0.,.1,.15),
                    's00510':(-.1,-.05,0.,.05,.1)}
    results=[]
    fine_cache={}
    for (ln,lat),(bn,lens),(jn,sl) in product(lat_profiles.items(),length_profiles.items(),slope_profiles.items()):
        # Filter once; retained rows still include rejects for pruning counts.
        sub={n:[r for r in rows if r['params'][0] in lat and r['params'][1] in lens
                and r['params'][2] in sl and r['params'][3]==0.] for n,rows in pools.items()}
        for w,rw in product((3.,3.5,4.,4.5,5.,6.,7.5,10.),(0.,.25,.5,1.)):
            r=summarize(ss,sub,f'{ln}/{bn}/{jn}/w{w}/r{rw}',lat,lens,sl,w,robust_weight=rw)
            fine={}
            for s in ss:
                if s.name not in r['selected']:continue
                params=tuple(r['selected'][s.name]['params'])
                key=(s.name,params)
                if key not in fine_cache:
                    dense=replace(s,resolution_m=.05)
                    object.__setattr__(dense,'_study_initial',s._study_initial)
                    fine_cache[key]=measure(dense,params)
                fine[s.name]=fine_cache[key]
            r['dense_success']=sum(all(v['flags']) for v in fine.values())
            r['dense_clearance']=min(v['clearance'] for v in fine.values())
            r['dense_min_speed']=min((v.get('speed',float('inf')) for v in fine.values()),default=0.)
            r['dense_failures']=[n for n,v in fine.items() if not all(v['flags'])]
            r['dense_mean_speed']=sum(v['speed'] for v in fine.values())/len(fine) if all(all(v['flags']) for v in fine.values()) else None
            results.append(r)
    results.sort(key=lambda r:(-r['dense_success'],-r['success'],r['deficit'],r['geo'],r['count']))
    (OUT/'finalists.json').write_text(json.dumps(results))
    for budget in (45,81,108,132,156,250):
        options=[r for r in results if r['count']<=budget and r['dense_success']==r['success']]
        if options:
            r=options[0]
            print('budget',budget,{k:v for k,v in r.items() if k not in ('selected','pruning')},flush=True)
    # Preserve full dense rows for reproducibility without duplicating them in
    # each summary configuration.
    (OUT/'finalist_dense_rows.json').write_text(json.dumps([dict(name=k[0],params=k[1],row=v) for k,v in fine_cache.items()]))


if __name__=='__main__':main()
