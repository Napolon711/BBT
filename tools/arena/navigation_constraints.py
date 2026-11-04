"""Sentry routing restrictions, applied only to the navigation substitute.

The physical Arena map is immutable. A routing copy blocks road bumps, and a
separate footprint check rejects commands aimed through that excluded terrain.
"""
import copy
import hashlib
import json
import math


def point_segment_distance(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    length=dx*dx+dy*dy
    u=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/length)) if length else 0.
    return math.hypot(p[0]-a[0]-u*dx,p[1]-a[1]-u*dy)


def _inside(p,polygon):
    inside=False
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        if point_segment_distance(p,a,b)<1e-9:return True
        if (a[1]>p[1])!=(b[1]>p[1]) and p[0]<(b[0]-a[0])*(p[1]-a[1])/(b[1]-a[1])+a[0]:inside=not inside
    return inside


def _intersect(a,b,c,d):
    def cross(p,q,r):return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
    return cross(a,b,c)*cross(a,b,d)<0 and cross(c,d,a)*cross(c,d,b)<0


def bumps_clearance(arena,start,end=None,radius=0.):
    """Minimum swept bounding-circle clearance; <=0 means footprint contact."""
    end=start if end is None else end
    result=math.inf
    for t in arena.get('terrain',[]):
        if t['kind']!='bumps':continue
        polygon=t.get('polygon') or [[t['x'],t['y']],[t['x']+t['w'],t['y']],
            [t['x']+t['w'],t['y']+t['h']],[t['x'],t['y']+t['h']]]
        distance=0. if _inside(start,polygon) or _inside(end,polygon) else math.inf
        for a,b in zip(polygon,polygon[1:]+polygon[:1]):
            if _intersect(start,end,a,b):distance=0.
            distance=min(distance,point_segment_distance(start,a,b),point_segment_distance(end,a,b),
                         point_segment_distance(a,start,end),point_segment_distance(b,start,end))
        result=min(result,distance-radius)
    return result


def sentry_routing_map(arena):
    """Block bumps in SDK planning without changing physics, rules or render data."""
    blocked=[t for t in arena.get('terrain',[]) if t['kind']=='bumps']
    result=copy.deepcopy(arena)
    # SDK adds a fixed .36 m planning margin. Expand this routing-only copy by
    # .12 m so its .48 m exclusion also covers the decision's .42 m bounding
    # circle plus .05 m safety margin. The physical terrain remains unchanged.
    padding=.12
    result.setdefault('obstacles',[]).extend(dict(id='sentry_no_bumps_'+t['id'],
        kind='barrier',x=t['x']-padding,y=t['y']-padding,
        w=t['w']+2*padding,h=t['h']+2*padding,
        rotation=0.,bottom=0.,height=10.) for t in blocked)
    # SDK caches by layout_revision/team. A separate key prevents accidentally
    # reusing a graph prepared for infantry that is allowed to cross bumps.
    digest=hashlib.sha256(json.dumps(blocked,sort_keys=True).encode()).hexdigest()[:12]
    result['layout_revision']=str(arena.get('layout_revision',''))+'-sentry-no-bumps-v2-'+digest
    return result
