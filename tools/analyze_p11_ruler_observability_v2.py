#!/usr/bin/env python3
import sys, cv2, math, statistics
from pathlib import Path
import numpy as np

root = Path(sys.argv[1])
stages = ["A1","B1","A2","B2","A3","B3","A4"]
diag = root / "ruler_diag_v2"
diag.mkdir(exist_ok=True)

def angle180(dx, dy):
    a = math.degrees(math.atan2(dy, dx)) % 180.0
    return a

def cd180(a, b):
    d = abs(a-b) % 180.0
    return min(d, 180.0-d)

def unwrap(vals, ref):
    out=[]
    for a in vals:
        x=a
        while x-ref>90: x-=180
        while x-ref<-90: x+=180
        out.append(x)
    return out

def segments(im):
    h,w = im.shape[:2]
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

    # Улучшаем локальный контраст, потому что B-позиции заметно беднее по контрасту.
    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8,8))
    g = clahe.apply(gray)
    g = cv2.GaussianBlur(g, (3,3), 0)

    lsd = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)
    lines, widths, prec, nfa = lsd.detect(g)

    out=[]
    if lines is None:
        return out

    for idx,ln in enumerate(lines[:,0,:]):
        x1,y1,x2,y2 = map(float,ln)
        dx=x2-x1; dy=y2-y1
        length=math.hypot(dx,dy)
        if length < 55:
            continue

        mx=(x1+x2)*0.5; my=(y1+y2)*0.5
        # По пользовательским кадрам рулетка находится в центральной/нижней части.
        # ROI остаётся широким, чтобы не "вшить" конкретную позицию.
        if my < h*0.18:
            continue

        a=angle180(dx,dy)
        if 78 < a < 102:   # подавляем почти вертикальные границы кадра/предметов
            continue

        # Предпочитаем длинные линии в центральных 90% кадра.
        edge_penalty = 0.65 if (mx < w*0.05 or mx > w*0.95) else 1.0
        score = length * edge_penalty
        out.append((score,length,a,(int(x1),int(y1),int(x2),int(y2))))
    return out

def orientation_clusters(segs, tol=3.0):
    if not segs:
        return []

    # Кандидаты кластеризуем вокруг реально наблюдаемых углов.
    candidates=[]
    for _,_,a,_ in segs:
        cluster=[s for s in segs if cd180(s[2],a)<=tol]
        support=sum(s[0] for s in cluster)
        rawlen=sum(s[1] for s in cluster)
        angles=[s[2] for s in cluster]
        uu=unwrap(angles,a)
        med=statistics.median(uu) % 180.0
        candidates.append({
            "angle":med,
            "support":support,
            "rawlen":rawlen,
            "n":len(cluster),
            "segments":cluster,
        })

    # Merge near-duplicate cluster centers.
    candidates.sort(key=lambda x:x["support"], reverse=True)
    uniq=[]
    for c in candidates:
        if any(cd180(c["angle"],u["angle"])<4.0 for u in uniq):
            continue
        uniq.append(c)
        if len(uniq)>=5:
            break
    return uniq

stage_candidates={}

print("================ P11 РУЛЕТКА: НАБЛЮДАЕМОСТЬ V2 ================")
print("run:", root)
print("Метод: CLAHE + LSD + ранжирование нескольких устойчивых линейных кандидатов.")
print("Это НЕ оценка полного roll/pitch и НЕ доказательство физического наклона.")

for st in stages:
    imgs=sorted(root.glob(f"{st}_*.png"))
    per_frame=[]
    representative=None
    rep_clusters=[]

    for idx,p in enumerate(imgs):
        im=cv2.imread(str(p))
        if im is None:
            continue
        cs=orientation_clusters(segments(im))
        per_frame.append(cs)
        if idx==len(imgs)//2:
            representative=im.copy()
            rep_clusters=cs

    # Строим stage-level hypotheses: каждый frame top-3 голосует.
    votes=[]
    for cs in per_frame:
        for rank,c in enumerate(cs[:3]):
            votes.append((c["angle"], c["support"]/(rank+1)))

    stage=[]
    if votes:
        for a,_ in votes:
            near=[v for v in votes if cd180(v[0],a)<=3.0]
            support=sum(v[1] for v in near)
            angles=[v[0] for v in near]
            uu=unwrap(angles,a)
            med=statistics.median(uu)%180.0
            frame_hits=0
            frame_angles=[]
            for cs in per_frame:
                hit=[c for c in cs[:3] if cd180(c["angle"],med)<=3.0]
                if hit:
                    frame_hits+=1
                    frame_angles.append(hit[0]["angle"])
            if frame_hits:
                uu2=unwrap(frame_angles,med)
                sd=statistics.pstdev(uu2) if len(uu2)>1 else 0.0
            else:
                sd=float("nan")
            stage.append({
                "angle":med,
                "support":support,
                "hits":frame_hits,
                "std":sd,
            })

        stage.sort(key=lambda x:(x["hits"],x["support"]), reverse=True)
        uniq=[]
        for c in stage:
            if any(cd180(c["angle"],u["angle"])<4.0 for u in uniq):
                continue
            uniq.append(c)
            if len(uniq)>=4:
                break
        stage=uniq

    stage_candidates[st]=stage

    print(f"\n{st}: кадров={len(imgs)}")
    if not stage:
        print("  Кандидатов нет.")
    else:
        for i,c in enumerate(stage,1):
            print(f"  C{i}: angle={c['angle']:+.3f} deg  hits={c['hits']}/{len(imgs)}  std={c['std']:.3f} deg")

    # Overlay: top 4 orientation clusters on representative frame.
    if representative is not None:
        colors=[(0,255,255),(0,255,0),(255,0,255),(255,255,0)]
        for rank,c in enumerate(rep_clusters[:4]):
            col=colors[rank]
            for _,_,_,(x1,y1,x2,y2) in c["segments"]:
                cv2.line(representative,(x1,y1),(x2,y2),col,2,cv2.LINE_AA)
            cv2.putText(representative,
                        f"C{rank+1} {c['angle']:.1f}deg",
                        (12,32+28*rank),
                        cv2.FONT_HERSHEY_SIMPLEX,0.65,col,2,cv2.LINE_AA)
        cv2.putText(representative,st,(520,35),
                    cv2.FONT_HERSHEY_SIMPLEX,0.9,(0,0,255),2,cv2.LINE_AA)
        cv2.imwrite(str(diag/f"{st}_candidates.png"),representative)

# Для каждого B ищем кандидата, наиболее близкого к устойчивому A-ориентиру ~146 deg,
# но НЕ объявляем его рулеткой автоматически.
a_angles=[]
for st in ["A1","A2","A3","A4"]:
    if stage_candidates.get(st):
        a_angles.append(stage_candidates[st][0]["angle"])

print("\n================ A-REFERENCE / B-CANDIDATES ================")
if a_angles:
    ref=statistics.median(unwrap(a_angles,a_angles[0])) % 180.0
    print(f"Устойчивый A reference candidate: {ref:+.3f} deg")
    for st in ["B1","B2","B3"]:
        cs=stage_candidates.get(st,[])
        if not cs:
            print(f"{st}: нет кандидатов")
            continue
        best=min(cs,key=lambda c:cd180(c["angle"],ref))
        print(f"{st}: ближайший к A candidate = {best['angle']:+.3f} deg, "
              f"distance={cd180(best['angle'],ref):.3f} deg, "
              f"hits={best['hits']}/20, std={best['std']:.3f}")
else:
    print("A reference не определён.")

# Contact sheet.
panels=[]
for st in stages:
    p=diag/f"{st}_candidates.png"
    im=cv2.imread(str(p))
    if im is not None:
        panels.append(cv2.resize(im,(320,240)))
if panels:
    rows=[]
    for i in range(0,len(panels),4):
        row=panels[i:i+4]
        while len(row)<4:
            row.append(np.zeros_like(panels[0]))
        rows.append(cv2.hconcat(row))
    sheet=cv2.vconcat(rows)
    cv2.imwrite(str(diag/"contact_sheet_candidates.png"),sheet)
    print(f"\nРазметка кандидатов: {diag/'contact_sheet_candidates.png'}")

print("\nИНТЕРПРЕТАЦИЯ:")
print("- Сначала визуально проверьте overlay: C1/C2/... должны лежать именно на кромках рулетки.")
print("- Высокие hits и малый std означают только стабильный image-space line candidate.")
print("- Совпадение B-кандидата с A по углу НЕ доказывает одинаковую физическую ориентацию.")
print("- Если в B ни один стабильный кандидат не лежит на рулетке, ruler-route закрываем для этого dataset.")
