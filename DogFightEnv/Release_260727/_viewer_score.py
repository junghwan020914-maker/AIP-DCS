"""실서버(BattleViewer) 궤적 CSV 두 개를 규정대로 채점한다.

08-13 실서버 대전에서 나온 필요. 뷰어는 `Log/<날짜>_0.csv`(plane0)와 `_1.csv`(plane1)에
Time/Lat/Lon/Alt/Roll/Pitch/Yaw만 남기고 **HP도 승패도 안 남긴다**. 로컬 평가와 같은
기준으로 비교하려면 같은 데미지 공식을 궤적에 다시 적용해야 한다.

규정(제5조 WEZ):
  d_wez = (3000ft - r) / 2500ft,  152.4m <= r <= 914.4m 구간에서 유효
  Phase1 t>=0s   ATA<1도  152.4~914.4m   계수 1.0
  Phase2 t>=100s ATA<2도  152.4~1066.8m  계수 0.3
  Phase3 t>=150s ATA<3도  152.4~1219.2m  계수 0.1
제6조1항 추락(고도<=300m)은 즉시 패배. 제6조2항 200초 시점 HP 우위가 승리.

두 CSV는 행수가 다를 수 있어(로그 주기 흔들림) 시간축으로 보간해 맞춘다.

사용:
    python _viewer_score.py --log-dir <뷰어Log경로> --stem 8_13_18_44
    python _viewer_score.py --p0 a_0.csv --p1 a_1.csv --name0 mine --name1 ryujan
Not part of the student template; safe to delete after use.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

FT = 0.3048
R_MIN = 500.0 * FT          # 152.4 m
DEG = np.pi / 180.0
R_EARTH = 6378137.0

# (발동시각s, ATA한계deg, 최대사거리m, 계수)
PHASES = [
    (0.0, 1.0, 3000.0 * FT, 1.0),
    (100.0, 2.0, 3500.0 * FT, 0.3),
    (150.0, 3.0, 4000.0 * FT, 0.1),
]


def read_csv(path: Path):
    a = np.genfromtxt(path, delimiter=",", skip_header=1, invalid_raise=False)
    a = a[np.isfinite(a).all(axis=1)]
    # Time,Lat,Lon,Alt,Roll,Pitch,Yaw
    return a[:, 0], a[:, 1], a[:, 2], a[:, 3], a[:, 4], a[:, 5], a[:, 6]


def to_local_xyz(lat, lon, alt, lat0, lon0):
    """기준점 접평면 ENU 근사. 교전 반경 수 km라 오차 무시 가능."""
    x = (lat - lat0) * DEG * R_EARTH                      # 북(N)
    y = (lon - lon0) * DEG * R_EARTH * np.cos(lat0 * DEG)  # 동(E)
    return np.stack([x, y, alt], axis=-1)


def forward_vec(roll, pitch, yaw):
    """기수 단위벡터를 (N, E, Up) 좌표로. yaw=0이 북, 시계방향 증가."""
    p, y = pitch * DEG, yaw * DEG
    return np.stack([np.cos(p) * np.cos(y), np.cos(p) * np.sin(y), np.sin(p)], axis=-1)


def resample(t_src, arr, t_dst):
    out = np.empty((len(t_dst),) + arr.shape[1:], dtype=float)
    if arr.ndim == 1:
        return np.interp(t_dst, t_src, arr)
    for k in range(arr.shape[1]):
        out[:, k] = np.interp(t_dst, t_src, arr[:, k])
    return out


def score(t, pos_a, fwd_a, pos_b):
    """a가 b에게 주는 누적 피해와 진단용 시계열."""
    los = pos_b - pos_a
    r = np.linalg.norm(los, axis=1)
    r_safe = np.where(r < 1e-6, 1e-6, r)
    cos = np.einsum("ij,ij->i", fwd_a, los) / r_safe
    ata = np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))

    dt = np.diff(t, prepend=t[0])
    dmg = np.zeros_like(t)
    for t_on, ata_max, r_max, coef in PHASES:
        ok = (t >= t_on) & (ata < ata_max) & (r >= R_MIN) & (r <= r_max)
        d_wez = (3000.0 * FT - r) / (2500.0 * FT)
        dmg += np.where(ok, np.clip(d_wez, 0.0, None) * coef, 0.0) * dt
    return dmg.cumsum(), r, ata


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log-dir")
    ap.add_argument("--stem", help="예: 8_13_18_44 (뒤에 _0/_1 붙음)")
    ap.add_argument("--p0")
    ap.add_argument("--p1")
    ap.add_argument("--name0", default="plane0")
    ap.add_argument("--name1", default="plane1")
    args = ap.parse_args()

    if args.stem:
        d = Path(args.log_dir)
        p0, p1 = d / f"{args.stem}_0.csv", d / f"{args.stem}_1.csv"
    else:
        p0, p1 = Path(args.p0), Path(args.p1)

    t0, la0, lo0, al0, r0, pi0, ya0 = read_csv(p0)
    t1, la1, lo1, al1, r1, pi1, ya1 = read_csv(p1)

    # 공통 시간축(둘 다 커버하는 구간, 촘촘한 쪽 기준)
    lo_t, hi_t = max(t0[0], t1[0]), min(t0[-1], t1[-1])
    n = max(len(t0), len(t1))
    t = np.linspace(lo_t, hi_t, n)

    lat_ref, lon_ref = la0[0], lo0[0]
    A = resample(t0, to_local_xyz(la0, lo0, al0, lat_ref, lon_ref), t)
    B = resample(t1, to_local_xyz(la1, lo1, al1, lat_ref, lon_ref), t)
    FA = resample(t0, forward_vec(r0, pi0, ya0), t)
    FB = resample(t1, forward_vec(r1, pi1, ya1), t)
    FA /= np.linalg.norm(FA, axis=1, keepdims=True)
    FB /= np.linalg.norm(FB, axis=1, keepdims=True)

    hp_a, dist, ata_a = score(t, A, FA, B)
    hp_b, _, ata_b = score(t, B, FB, A)

    alt_a, alt_b = A[:, 2], B[:, 2]
    crash_a = np.argmax(alt_a <= 300.0) if (alt_a <= 300.0).any() else -1
    crash_b = np.argmax(alt_b <= 300.0) if (alt_b <= 300.0).any() else -1

    print("=" * 96)
    print(f"[실서버 채점] {p0.name} / {p1.name}   {t[0]:.0f}~{t[-1]:.0f}s  {len(t)}샘플")
    print(f"  최종 피해량  {args.name0} -> {args.name1}: {hp_a[-1]:.4f}")
    print(f"              {args.name1} -> {args.name0}: {hp_b[-1]:.4f}")
    print(f"  거리   중앙 {np.median(dist):6.0f}m  최소 {dist.min():5.0f}m  "
          f"밴드(152~914m) 체류 {np.mean((dist >= R_MIN) & (dist <= 914.4)) * 100:.1f}%")
    print(f"  {args.name0} ATA 최소 {ata_a.min():5.1f}도  중앙 {np.median(ata_a):5.1f}도  "
          f"<1도 {np.mean(ata_a < 1.0) * 100:.2f}%")
    print(f"  {args.name1} ATA 최소 {ata_b.min():5.1f}도  중앙 {np.median(ata_b):5.1f}도  "
          f"<1도 {np.mean(ata_b < 1.0) * 100:.2f}%")
    print(f"  고도   {args.name0} 최저 {alt_a.min():5.0f}m   {args.name1} 최저 {alt_b.min():5.0f}m")
    if crash_a >= 0:
        print(f"  🔴 {args.name0} 추락(고도<=300m) t={t[crash_a]:.1f}s -> 규정 제6조1항 즉시 패")
    if crash_b >= 0:
        print(f"  🔴 {args.name1} 추락(고도<=300m) t={t[crash_b]:.1f}s -> 규정 제6조1항 즉시 패")

    if crash_a >= 0 or crash_b >= 0:
        res = "패배" if crash_a >= 0 and (crash_b < 0 or crash_a < crash_b) else "승리"
    elif hp_a[-1] > hp_b[-1]:
        res = "승리"
    elif hp_a[-1] < hp_b[-1]:
        res = "패배"
    else:
        res = "무승부"
    print(f"  ➜ {args.name0} 기준 결과: **{res}**  (HP {100 - hp_b[-1]:.2f} vs {100 - hp_a[-1]:.2f})")

    # 국면 흐름 — 20초 간격으로 거리/양쪽 ATA
    print("\n  t     거리   {:>7}ATA {:>7}ATA   누적피해(나/상대)".format(args.name0, args.name1))
    for tt in range(0, int(t[-1]) + 1, 20):
        i = int(np.searchsorted(t, tt))
        if i >= len(t):
            break
        print(f"  {t[i]:5.0f} {dist[i]:6.0f}   {ata_a[i]:9.1f} {ata_b[i]:9.1f}   "
              f"{hp_a[i]:.4f} / {hp_b[i]:.4f}")
    print("=" * 96)


if __name__ == "__main__":
    main()
