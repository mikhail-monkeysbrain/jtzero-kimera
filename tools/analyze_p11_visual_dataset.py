#!/usr/bin/env python3
import sys, cv2, math, statistics
from pathlib import Path

root=Path(sys.argv[1])
stages=["A1","B1","A2","B2","A3","B3","A4"]

print("================ P11 VISUAL DATASET CHECK ================")
print("run:",root)

all_imgs={}
for st in stages:
    imgs=sorted(root.glob(f"{st}_*.png"))
    all_imgs[st]=imgs
    print(f"{st}: files={len(imgs)}")
    if not imgs:
        continue
    sharp=[]; mean=[]; std=[]; orb_counts=[]
    orb=cv2.ORB_create(nfeatures=1200)
    for p in imgs:
        im=cv2.imread(str(p),cv2.IMREAD_GRAYSCALE)
        if im is None:
            continue
        sharp.append(cv2.Laplacian(im,cv2.CV_64F).var())
        mean.append(float(im.mean()))
        std.append(float(im.std()))
        kp,_=orb.detectAndCompute(im,None)
        orb_counts.append(len(kp) if kp is not None else 0)
    if sharp:
        print(f"  sharp median={statistics.median(sharp):.1f} min={min(sharp):.1f}")
        print(f"  intensity mean median={statistics.median(mean):.1f} contrast std median={statistics.median(std):.1f}")
        print(f"  ORB features median={statistics.median(orb_counts):.0f} min={min(orb_counts)}")

print("\n================ ADJACENT A/B HOMOGRAPHY OBSERVABILITY ================")
orb=cv2.ORB_create(nfeatures=1800)
bf=cv2.BFMatcher(cv2.NORM_HAMMING)
pairs=[("A1","B1"),("A2","B2"),("A3","B3")]

for a,b in pairs:
    if not all_imgs[a] or not all_imgs[b]:
        print(f"{a}->{b}: НЕТ ДАННЫХ")
        continue
    ia=cv2.imread(str(all_imgs[a][len(all_imgs[a])//2]),cv2.IMREAD_GRAYSCALE)
    ib=cv2.imread(str(all_imgs[b][len(all_imgs[b])//2]),cv2.IMREAD_GRAYSCALE)
    k1,d1=orb.detectAndCompute(ia,None)
    k2,d2=orb.detectAndCompute(ib,None)
    if d1 is None or d2 is None:
        print(f"{a}->{b}: недостаточно признаков")
        continue
    knn=bf.knnMatch(d1,d2,k=2)
    good=[m for m,n in knn if m.distance<0.75*n.distance]
    if len(good)<8:
        print(f"{a}->{b}: good_matches={len(good)} — недостаточно для устойчивой homography")
        continue
    import numpy as np
    p1=np.float32([k1[m.queryIdx].pt for m in good])
    p2=np.float32([k2[m.trainIdx].pt for m in good])
    H,mask=cv2.findHomography(p1,p2,cv2.RANSAC,3.0)
    nin=int(mask.sum()) if mask is not None else 0
    ratio=nin/len(good) if good else 0
    if H is None:
        print(f"{a}->{b}: good_matches={len(good)} H=FAIL")
    else:
        h=H/H[2,2]
        persp=math.hypot(float(h[2,0]),float(h[2,1]))
        print(f"{a}->{b}: keypoints={len(k1)}/{len(k2)} good={len(good)} inliers={nin} ({ratio*100:.1f}%)")
        print(f"  H perspective-term norm={persp:.6e}")
        print("  NOTE: это только проверка наблюдаемости/стабильности, не физический угол.")

print("\nINTERPRETATION:")
print("- 20 файлов на каждый stage => logger protocol complete.")
print("- Высокий feature count + высокий homography inlier ratio => monocular planar relation можно анализировать дальше.")
print("- Низкий match/inlier ratio => текущая текстура недостаточна; нельзя извлекать orientation и делать причинный вывод.")
print("- Даже хорошая homography сама по себе не даёт независимый физический roll/pitch без дополнительных геометрических ограничений.")
