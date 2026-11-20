"""서로 다른 팔 자세에서 관측한 집게·물체 상대 형상의 지지를 계산한다."""
import math


# 동일 카메라 설정에서 약 2픽셀의 윤곽 재검출 및 1도 이하 축 판독 여유.
# 위치는 해당 카메라의 물체 폭·높이 또는 열린 턱 간격으로 정규화한다.
RESOLUTION = {
    "sx": .04, "sy": .012, "nx": .017, "ny": .017,
    "depth": .012, "clearance": .012, "sideObjectAspect": .012,
    "sideObjectTilt": 1., "sideJawAngle": 1., "nearJawAngle": 1.,
    "nearWidth": .017, "nearHeight": .017, "nearLeft": .017,
    "nearRight": .017, "nearTop": .017,
}


def close_pose_margin(angle, observed_lift_travel, upper_limit=208.):
    """같은 종료 조건으로 실측한 상승 명령의 최대 OFF 변위를 예약한다."""
    if not observed_lift_travel or not math.isfinite(angle):
        return False
    if any(not math.isfinite(value) or value<=0 for value in observed_lift_travel):
        return False
    return 194. < angle <= 206. and angle + max(observed_lift_travel) < upper_limit


def compare_grasp_geometry(observed, references):
    """다른 시연에서 얻은 두 종단점이 지지하는 형상에 들어오는지 확인한다."""
    keys=tuple(RESOLUTION)
    if len({r['episode'] for r in references})<2:
        return {'supported':False,'reason':'independent_references_missing'}
    vectors=[[r['features'][k] for k in keys] for r in references]
    q=[observed[k] for k in keys]
    if not all(math.isfinite(v) for row in [q,*vectors] for v in row):
        return {'supported':False,'reason':'nonfinite_geometry'}
    scales=[]
    for j,k in enumerate(keys):
        column=[x[j] for x in vectors]
        low,high=min(column),max(column)
        if q[j]<low-RESOLUTION[k] or q[j]>high+RESOLUTION[k]:
            return {'supported':False,'reason':'outside_measured_'+k}
        scales.append(max(high-low,RESOLUTION[k]))
    def distance(a,b):
        return math.sqrt(sum(((x-y)/scale)**2 for x,y,scale in zip(a,b,scales))/len(keys))
    # Each endpoint has the radius of its nearest independently recorded
    # neighbor. Two regions must overlap; a lone sparse exemplar cannot vote.
    neighbors=[]
    quant=distance([RESOLUTION[k] for k in keys],[0.]*len(keys))
    for i,row in enumerate(vectors):
        ep=references[i]['episode']
        radius=min(distance(row,other) for j,other in enumerate(vectors) if references[j]['episode']!=ep)
        d=distance(q,row)
        if d<=radius+quant:
            neighbors.append((d,ep))
    episodes=[]
    for _,ep in sorted(neighbors):
        if ep not in episodes:episodes.append(ep)
    return {'supported':len(episodes)>=2,'reason':'local_matches' if len(episodes)>=2 else 'insufficient_local_matches',
            'episodes':episodes}
