#!/usr/bin/env python3
import csv, math, statistics

H="/home/vio"
LEGS=H+"/jtzero_500mm_v18_legs.csv"
BACK=H+"/jtzero_500mm_v18_backend.csv"
OUT=H+"/jtzero_v18_bias_by_leg.csv"

def read(p):
    with open(p,newline="") as f:return list(csv.DictReader(f))
def iv(r,k):return int(float(r[k]))
def fv(r,k):return float(r[k])

legs=read(LEGS)
be=read(BACK)

need={"bgx","bgy","bgz","bax","bay","baz"}
if not be or not need.issubset(be[0].keys()):
    raise SystemExit("Backend CSV has no bias columns. Rebuild/re-run V18 with current repository.")

def vec(r,p):
    return [fv(r,p+"x"),fv(r,p+"y"),fv(r,p+"z")]

def norm(v): return math.sqrt(sum(x*x for x in v))
def sub(a,b): return [x-y for x,y in zip(a,b)]

rows=[]
print("================ V18 BIAS BY LEG ================")
for L in legs:
    leg=iv(L,"leg")
    ks=iv(L,"start_settled_kf")
    ke=iv(L,"end_press_kf")
    rr=[r for r in be if ks<=iv(r,"keyframe")<=ke]
    if not rr: continue

    a0=vec(rr[0],"ba"); a1=vec(rr[-1],"ba")
    g0=vec(rr[0],"bg"); g1=vec(rr[-1],"bg")
    da=sub(a1,a0); dg=sub(g1,g0)

    am=[statistics.mean(fv(r,k) for r in rr) for k in ("bax","bay","baz")]
    gm=[statistics.mean(fv(r,k) for r in rr) for k in ("bgx","bgy","bgz")]

    xy=fv(L,"horizontal_m")*1000
    row=dict(
      leg=leg,direction=L["direction"],xy_mm=xy,scale=xy/500.0,
      start_bax=a0[0],start_bay=a0[1],start_baz=a0[2],
      end_bax=a1[0],end_bay=a1[1],end_baz=a1[2],
      delta_bax=da[0],delta_bay=da[1],delta_baz=da[2],delta_ba_norm=norm(da),
      mean_bax=am[0],mean_bay=am[1],mean_baz=am[2],mean_ba_norm=norm(am),
      start_bgx=g0[0],start_bgy=g0[1],start_bgz=g0[2],
      end_bgx=g1[0],end_bgy=g1[1],end_bgz=g1[2],
      delta_bgx=dg[0],delta_bgy=dg[1],delta_bgz=dg[2],delta_bg_norm=norm(dg),
      mean_bgx=gm[0],mean_bgy=gm[1],mean_bgz=gm[2],mean_bg_norm=norm(gm),
    )
    rows.append(row)
    print(f'LEG {leg} {L["direction"]}: XY={xy:.2f} scale={xy/500:.3f}')
    print(f'  BA start=[{a0[0]:+.5f},{a0[1]:+.5f},{a0[2]:+.5f}] '
          f'end=[{a1[0]:+.5f},{a1[1]:+.5f},{a1[2]:+.5f}] '
          f'deltaNorm={norm(da):.5f}')
    print(f'  BG start=[{g0[0]:+.6f},{g0[1]:+.6f},{g0[2]:+.6f}] '
          f'end=[{g1[0]:+.6f},{g1[1]:+.6f},{g1[2]:+.6f}] '
          f'deltaNorm={norm(dg):.6f}')

with open(OUT,"w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)

def corr(xs,ys):
    mx=sum(xs)/len(xs); my=sum(ys)/len(ys)
    num=sum((x-mx)*(y-my) for x,y in zip(xs,ys))
    dx=math.sqrt(sum((x-mx)**2 for x in xs))
    dy=math.sqrt(sum((y-my)**2 for y in ys))
    return num/(dx*dy) if dx and dy else float("nan")

print("\n================ BIAS CORRELATIONS WITH SCALE ================")
ys=[r["scale"] for r in rows]
for k in ["start_bax","start_bay","start_baz","end_bax","end_bay","end_baz",
          "delta_bax","delta_bay","delta_baz","delta_ba_norm","mean_ba_norm",
          "start_bgx","start_bgy","start_bgz","end_bgx","end_bgy","end_bgz",
          "delta_bg_norm","mean_bg_norm"]:
    print(f'Pearson({k}, scale) = {corr([r[k] for r in rows],ys):+.4f}')

print("\n================ SESSION DRIFT ================")
first=rows[0]; last=rows[-1]
print(f'scale LEG1 -> LEG6: {first["scale"]:.3f} -> {last["scale"]:.3f}')
print(f'mean BA norm LEG1 -> LEG6: {first["mean_ba_norm"]:.5f} -> {last["mean_ba_norm"]:.5f}')
print(f'mean BG norm LEG1 -> LEG6: {first["mean_bg_norm"]:.6f} -> {last["mean_bg_norm"]:.6f}')
print("Saved:",OUT)
