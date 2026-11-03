"""Compute obstacle routes inside the Arena navigation substitute.

The SDK exposes its graph but normally returns only the first movement vector.
Here Dijkstra predecessors retain every graph vertex. Each returned segment also
passes the sentry's mandatory swept-circle exclusion for road bumps.
"""
import heapq
import math

try:
    from .navigation_constraints import bumps_clearance
except ImportError:  # Direct execution alongside the Arena bridge.
    from navigation_constraints import bumps_clearance


_COST_CACHE={}


def remaining_route(sdk,robot,goal,routing_arena,clearance=.47):
    """Return [current position, graph vertices..., goal], or [] if unreachable.

    Coordinates remain in the Arena frame. No physics, map geometry, robot pose,
    or navigation destination is changed. Missing SDK graph support fails closed.
    """
    prepare=getattr(sdk,'_prepare_chapter4',None)
    if prepare is None or not math.isfinite(clearance) or clearance<.47:
        return []
    start=(float(robot['x']),float(robot['y']));goal=tuple(map(float,goal))
    if len(goal)!=2 or not all(math.isfinite(x) for x in (*start,*goal)):
        return []
    graph=prepare(routing_arena,robot['team'],robot.get('transform_progress',0.)>=.99)
    nodes,edges,sdk_clear,inside=graph

    def bump_clear(a,b):
        return bumps_clearance(routing_arena,a,b,clearance)>1e-6

    def clear(a,b,ignore=(),strict=True):
        return bump_clear(a,b) and sdk_clear(a,b,ignore,strict)

    if not bump_clear(start,start) or not bump_clear(goal,goal):
        return []
    near_goal=inside(goal);guarded=inside(start,True)
    for strict in (True,False):
        variants=(guarded,set(guarded)) if guarded else (guarded,)
        for near_start in variants:
            ignore=({**near_start,**dict.fromkeys(near_goal)} if isinstance(near_start,dict)
                    else near_start|near_goal)
            if clear(start,goal,ignore,strict):
                return [start,goal]
            key=(id(graph),sdk_clear,goal,frozenset(near_goal),strict,clearance)
            cached=_COST_CACHE.get(key)
            if cached is None:
                costs={i:math.dist(p,goal) for i,p in enumerate(nodes)
                       if clear(p,goal,near_goal,strict)}
                onward={i:None for i in costs}
                queue=[(cost,i) for i,cost in costs.items()];heapq.heapify(queue)
                while queue:
                    cost,index=heapq.heappop(queue)
                    if cost!=costs[index]:continue
                    for other,distance in edges[index]:
                        # SDK edges have already passed its obstacle/height
                        # checks; the extra mandatory terrain exclusion remains
                        # necessary even when supplied a generic SDK graph.
                        if not bump_clear(nodes[index],nodes[other]):continue
                        candidate=cost+distance
                        if candidate<costs.get(other,math.inf):
                            costs[other]=candidate;onward[other]=index
                            heapq.heappush(queue,(candidate,other))
                cached=(costs,onward)
                if len(_COST_CACHE)>=64:_COST_CACHE.pop(next(iter(_COST_CACHE)))
                _COST_CACHE[key]=cached
            costs,onward=cached
            candidates=sorted((math.dist(start,nodes[i])+cost,i) for i,cost in costs.items())
            # Match the SDK's already-reached waypoint handling, while requiring
            # a new direct visible segment to every skipped waypoint.
            ahead=[item for item in candidates if math.dist(start,nodes[item[1]])>=.3]
            nearby=[item for item in candidates if math.dist(start,nodes[item[1]])<.3]
            # A nearby corner may still be essential for clearing a wall end.
            # If no farther vertex is visible, finish this short leg instead of
            # declaring the whole graph unreachable at the .3 m threshold.
            for _,index in ahead+nearby:
                if not clear(start,nodes[index],near_start,strict):continue
                path=[start];seen=set()
                while index is not None:
                    if index in seen:return []
                    seen.add(index)
                    point=tuple(nodes[index])
                    if math.dist(path[-1],point)>1e-9:path.append(point)
                    index=onward[index]
                if math.dist(path[-1],goal)>1e-9:path.append(goal)
                # Keep the promise local even if a cached graph is malformed.
                if all(bump_clear(a,b) for a,b in zip(path,path[1:])):
                    return path
    return []
