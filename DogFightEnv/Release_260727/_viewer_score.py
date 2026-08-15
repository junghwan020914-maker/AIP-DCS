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

## 🔴 08-15 결함 3건 발견 및 수정 — 이 채점기로 내린 판정을 철회했다

이 파일로 "뷰어는 Phase 2/3을 주지 않는다"고 판정했었다(08-13). **그 판정은 근거가
없다.** 아래 셋이 겹쳐 있었다.

**① 시간축을 환산하지 않았다.** `Time` 열은 **경기 시계의 2.07배**로 흐른다.
   격추 사건으로 역교정해 확정했다 — `8_13_22_47`(arcE 격추판)은 행 1390, `Time` 0~139인데
   화면 실제 격추는 **67초**였다. 139/67 = 2.07, 행 간격 0.048초(약 20.7Hz).
   교차검증: 그 dt로 Phase1 피해를 적분하면 1.107이 나온다(격추 기준 1.0, 오차 11%).
   → 옛 코드는 `t>=100`/`t>=150` 게이트를 이 열에 직접 걸었으므로 **Phase 2가 경기시계
     48초에, Phase 3이 72초에 열렸다.** `dt`도 같은 축이라 **모든 피해가 2.07배** 부풀었다.

**② 위상을 합산했다.** `dmg +=` 루프라 조건이 겹치면 계수가 더해졌다(1.0+0.3+0.1=1.4배).
   규정은 "Phase3이 켜진 상태에서 적기가 Phase1 범위 안에 있다면 Phase1의 대미지가
   적용됩니다" — **활성 phase 중 안쪽 우선, 합산 아님.**
   (로컬 채점기 `_aim_time_probe.score_rate`는 처음부터 올바랐다. 이 파일만 틀렸다.)

**③ 그리고 애초에 이 자료로 1도 판정을 할 수 없다.** 궤적은 20Hz로 기록되지만
   **`Time` 도장은 1초 정수**다. 그 축 위에서 보간하면 실질 해상도가 1Hz로 떨어진다.
   원본 대 보간본 ATA 차이: **중앙 0.42~0.73도, p99 5.2~8.0도, 최대 10~18도.**
   기체는 초당 10~18도 돈다. **1도 창(1.78초면 격추)을 이 해상도로 판별할 수 없다** —
   놓칠 수도, 없는 것을 만들 수도 있다.

➡️ **결론: 이 채점기는 기하(거리·고도·대략적 ATA) 확인용이지 데미지 모델 판정용이 아니다.**
   대회 서버가 어떤 Phase를 쓰는지는 **규정 Q&A가 유일한 근거**이고, 규정은 Phase 1/2/3을
   모두 명시한다. 공식 공지·답변에 뷰어가 다르다는 언급은 없다.
   → **규정 모델(P1+P2+P3)을 기본으로 삼는다.**

⚠️ `--time-scale`은 기본 2.07이지만 **판마다 다를 수 있다.** 화면에서 읽은 실제 경기
   길이를 알면 `--match-seconds`로 주는 쪽이 정확하다(그러면 배율을 자료에서 유도한다).

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
    """a가 b에게 주는 누적 피해와 진단용 시계열.

    `t`는 **경기 시계(초)**여야 한다 — CSV `Time` 열이 아니다(위 결함 ① 참고).
    """
    los = pos_b - pos_a
    r = np.linalg.norm(los, axis=1)
    r_safe = np.where(r < 1e-6, 1e-6, r)
    cos = np.einsum("ij,ij->i", fwd_a, los) / r_safe
    ata = np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))

    dt = np.diff(t, prepend=t[0])
    d_wez = np.clip((3000.0 * FT - r) / (2500.0 * FT), 0.0, None)
    # 활성 phase 중 **안쪽(계수 큰 쪽) 하나만** 적용한다. 합산이 아니다.
    # PHASES는 안쪽부터 정렬돼 있으므로, 아직 계수가 정해지지 않은 곳만 채워 나간다.
    coef_now = np.zeros_like(t)
    filled = np.zeros_like(t, dtype=bool)
    for t_on, ata_max, r_max, coef in PHASES:
        ok = (t >= t_on) & (ata < ata_max) & (r >= R_MIN) & (r <= r_max) & ~filled
        coef_now = np.where(ok, coef, coef_now)
        filled |= ok
    return (d_wez * coef_now * dt).cumsum(), r, ata


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log-dir")
    ap.add_argument("--stem", help="예: 8_13_18_44 (뒤에 _0/_1 붙음)")
    ap.add_argument("--p0")
    ap.add_argument("--p1")
    ap.add_argument("--name0", default="plane0")
    ap.add_argument("--name1", default="plane1")
    ap.add_argument("--time-scale", type=float, default=2.07,
                    help="CSV Time 열 / 경기 시계. 격추 사건 역교정으로 얻은 기본값 2.07")
    ap.add_argument("--match-seconds", type=float,
                    help="화면에서 읽은 실제 경기 길이(초). 주면 배율을 자료에서 유도한다")
    args = ap.parse_args()

    if args.stem:
        d = Path(args.log_dir)
        p0, p1 = d / f"{args.stem}_0.csv", d / f"{args.stem}_1.csv"
    else:
        p0, p1 = Path(args.p0), Path(args.p1)

    t0, la0, lo0, al0, r0, pi0, ya0 = read_csv(p0)
    t1, la1, lo1, al1, r1, pi1, ya1 = read_csv(p1)

    # 공통 시간축(둘 다 커버하는 구간, 촘촘한 쪽 기준). **CSV Time 단위**다 —
    # 보간은 t0/t1과 같은 축에서 해야 하므로 여기서는 환산하지 않는다.
    lo_t, hi_t = max(t0[0], t1[0]), min(t0[-1], t1[-1])
    n = max(len(t0), len(t1))
    t_csv = np.linspace(lo_t, hi_t, n)

    # 경기 시계로 환산한다. 이 축만 phase 게이트와 피해 적분에 쓴다(결함 ①).
    if args.match_seconds:
        scale = (hi_t - lo_t) / args.match_seconds
        src = f"--match-seconds {args.match_seconds:.0f} 에서 유도"
    else:
        scale = args.time_scale
        src = "기본값(격추 사건 역교정)"
    t = (t_csv - lo_t) / scale

    lat_ref, lon_ref = la0[0], lo0[0]
    A = resample(t0, to_local_xyz(la0, lo0, al0, lat_ref, lon_ref), t_csv)
    B = resample(t1, to_local_xyz(la1, lo1, al1, lat_ref, lon_ref), t_csv)
    FA = resample(t0, forward_vec(r0, pi0, ya0), t_csv)
    FB = resample(t1, forward_vec(r1, pi1, ya1), t_csv)
    FA /= np.linalg.norm(FA, axis=1, keepdims=True)
    FB /= np.linalg.norm(FB, axis=1, keepdims=True)

    hp_a, dist, ata_a = score(t, A, FA, B)
    hp_b, _, ata_b = score(t, B, FB, A)

    alt_a, alt_b = A[:, 2], B[:, 2]
    crash_a = np.argmax(alt_a <= 300.0) if (alt_a <= 300.0).any() else -1
    crash_b = np.argmax(alt_b <= 300.0) if (alt_b <= 300.0).any() else -1

    print("=" * 96)
    print(f"[실서버 채점] {p0.name} / {p1.name}   경기시계 {t[0]:.0f}~{t[-1]:.0f}s  {len(t)}샘플")
    print(f"  시간축 환산 x{scale:.2f} ({src})   CSV Time 열 {lo_t:.0f}~{hi_t:.0f}")
    print(f"  [주의] CSV Time 도장은 1초 정수라 실질 해상도가 1Hz다. 보간 오차 중앙 0.4~0.7도"
          f"(p99 5~8도)로 1도 판정은 신뢰할 수 없다 - 기하 확인용으로만 볼 것.")
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
