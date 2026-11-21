/* Host tests for app_program/vc_follow.c (line following v4). */
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "../../app_program/vc_follow.h"
static int fails;
#define CHECK(c,m) do{ if(!(c)){printf("FAIL: %s\n",m);fails++;} else printf("ok: %s\n",m);}while(0)

/* Normalised reading of a sensor d mm from the centre of an 18 mm tape
 * (4 mm sensing spot). */
static uint32_t tape_read(double d)
{
  double o = fmin(d + 2.0, 9.0) - fmax(d - 2.0, -9.0);
  return o <= 0.0 ? 0u : (uint32_t)(o / 4.0 * 1000.0);
}

/* Model car (97 mm track, sensors 60 mm ahead of the axle, 15 mm either
 * side) on a circle (positive r left, negative r right), with wheel lag.
 * Reports how often the steering changed side and how often it fell
 * back to ~straight in the second half of the lap, and the worst offset. */
static void circle_run(double r, int *side_changes, int *straight_ticks,
                       double *max_off, bool *lost)
{
  vc4_track_t t; double x = 0, y = 0, h = 0, vl = 0, vr = 0;
  double radius=fabs(r), direction=r>0?1.0:-1.0;
  uint32_t um = 0; int last = 0;
  vc4_reset(&t);
  *side_changes = 0; *straight_ticks = 0; *max_off = 0; *lost = false;
  for (int k = 0; k < 1500; k++) {
    double sx = x + 60 * cos(h), sy = y + 60 * sin(h);
    /* + = left of the line; circle centre (0, r) */
    double dl = direction*(radius - hypot(sx - 15 * sin(h), sy + 15 * cos(h) - r));
    double dr = direction*(radius - hypot(sx + 15 * sin(h), sy - 15 * cos(h) - r));
    double dc = direction*(radius - hypot(sx, sy - r));
    int32_t s = vc4_steer(&t, tape_read(dl), tape_read(dr), 20, um), sp, st;
    vc4_wheels(s, 120, 100, &sp, &st);
    if (!vc4_on_line(&t)) { *lost = true; return; }
    double v = sp * 0.084;
    vl += (v * (1 + st / 1000.0) - vl) * 0.15;
    vr += (v * (1 - st / 1000.0) - vr) * 0.15;
    x += (vl + vr) / 2 * cos(h) * 0.02; y += (vl + vr) / 2 * sin(h) * 0.02;
    h += (vr - vl) / 97.0 * 0.02;
    um += (uint32_t)((vl + vr) / 2 * 20);
    if (k >= 750) {
      int sg = s > 150 ? 1 : (s < -150 ? -1 : 0);
      if (!sg) (*straight_ticks)++;
      if (sg && last && sg != last) (*side_changes)++;
      if (sg) last = sg;
      if (fabs(dc) > *max_off) *max_off = fabs(dc);
    }
  }
}
int main(void){
  vc4_track_t t; uint32_t um=0; int32_t s; int32_t sp, st;
  {
    vc4_track_t left, right;
    int32_t ls=0, rs=0, lsp, lst, rsp, rst;
    vc4_reset(&left); vc4_reset(&right);
    for(int i=0;i<4;i++){
      ls=vc4_steer(&left,600,100,20,(uint32_t)i*2000);
      rs=vc4_steer(&right,100,600,20,(uint32_t)i*2000);
    }
    CHECK(rs==-589, "right curve entry retains previous steering response");
    CHECK(ls==-rs && ls<=VC4_STEER_MAX,
          "left and right curve entry retain the same filter response");
    vc4_wheels(500,120,100,&lsp,&lst);
    vc4_wheels(-500,120,100,&rsp,&rst);
    CHECK(lsp==rsp && lst==-rst,
          "left adjustment leaves wheel split and speed mapping unchanged");
    for(int i=0;i<20;i++) ls=vc4_steer(&left,100,100,20,8000+(uint32_t)i*2000);
    CHECK(vc4_on_line(&left) && ls<200,
          "partial left correction still releases toward straight");
  }
  vc4_wheels(1000,60,100,&sp,&st);
  CHECK(sp*(1000+(st<0?st:-st))/1000>=150,
        "slow tight curve keeps inner wheel above 150 cps");
  vc4_reset(&t);
  vc4_junction(&t,700,700,0);
  for(uint32_t pos=2000;pos<18000;pos+=2000)
    vc4_junction(&t,550,600,pos);
  vc4_junction(&t,100,100,18000);
  CHECK(vc4_junction(&t,100,100,18000+VC4_JCT_CONFIRM_UM)==VC4_JCT_CROSS,
        "junction survives fading sensor contrast across the tape");
  vc4_reset(&t);
  for(int i=0;i<10;i++) s=vc4_steer(&t,1000,350,20,(uint32_t)i*2400);
  CHECK(!t.cross_hold && s>300, "curve plus grey floor cannot force crossing straight hold");
  /* centred straight: both white forever -> never lost, steer 0 */
  vc4_reset(&t);
  for(int i=0;i<500;i++){ s=vc4_steer(&t,100,100,20,um); um+=2400; }
  CHECK(vc4_on_line(&t) && s==0, "centred both-white never lost");
  /* line drifts under left sensor -> steer + (left), wheels turn left */
  for(int i=0;i<10;i++){ s=vc4_steer(&t,900,50,20,um); um+=2400; }
  CHECK(s>300, "line under left sensor -> steer left");
  vc4_wheels(s,120,100,&sp,&st);
  CHECK(st<0, "B2 steering negative (turn left)");
  printf("   s=%d speed_cps=%d steer_permille=%d\n",s,sp,st);
  vc4_wheels(1000,120,100,&sp,&st);
  printf("   full: speed_cps=%d steer=%d\n",sp,st);
  CHECK(sp>0 && st>-1000, "full steer keeps inner wheel moving");
  /* line leaves past left sensor: coast then arc left, lost after 200ms+60mm */
  int lost_at=-1;
  for(int i=0;i<100;i++){ s=vc4_steer(&t,50,50,20,um); um+=2400; if(!vc4_on_line(&t)&&lost_at<0) lost_at=i; }
  CHECK(lost_at>=9 && lost_at<=30, "lost after black exit + 200ms/60mm");
  printf("   lost at tick %d, recovery steer %d\n",lost_at,s);
  CHECK(s>0, "recovery arc toward exit side (left)");
  /* found again */
  s=vc4_steer(&t,800,100,20,um);
  CHECK(vc4_on_line(&t), "found again on black");
  /* grey graze then white: not lost */
  vc4_reset(&t);
  for(int i=0;i<5;i++) vc4_steer(&t,500,100,20,um);
  for(int i=0;i<100;i++){ vc4_steer(&t,100,100,20,um); um+=2400; }
  CHECK(vc4_on_line(&t), "grey graze then white = centred, not lost");
  /* junction: both black for 18 mm then 30 mm -> CROSS */
  vc4_reset(&t); um=0; vc4_jct_t j=0, got=0;
  for(int i=0;i<200;i++){ uint32_t pos=um/1000; uint32_t nl=100,nr=100;
    if(pos>=100 && pos<118){nl=1000;nr=1000;}
    j=vc4_junction(&t,nl,nr,um); if(j) got=j; um+=2400; }
  CHECK(got==VC4_JCT_CROSS, "18 mm crossing tape -> CROSS");
  /* narrow (5mm) -> nothing */
  vc4_reset(&t); um=0; got=0;
  for(int i=0;i<200;i++){ uint32_t pos=um/1000; uint32_t nl=100,nr=100;
    if(pos>=100 && pos<105){nl=1000;nr=1000;}
    j=vc4_junction(&t,nl,nr,um); if(j) got=j; um+=1000; }
  CHECK(got==VC4_JCT_NONE, "5 mm stripe not a junction");
  /* one sensor only (curve) -> never junction */
  vc4_reset(&t); um=0; got=0;
  for(int i=0;i<200;i++){ j=vc4_junction(&t,1000,200,um); if(j) got=j; um+=2400; }
  CHECK(got==VC4_JCT_NONE, "one sensor black (curve) not a junction");
  /* unmarked cross met at an angle: left sensor hits the arm 6 mm first
   * and leaves 6 mm later -> car must go straight, never lost */
  { vc4_reset(&t); um=0; int32_t maxs=0; bool lostx=false;
    for(int i=0;i<300;i++){ uint32_t pos=um/1000; uint32_t nl=100,nr=100;
      if(pos>=100 && pos<124) nl=1000;
      if(pos>=106 && pos<130) nr=1000;
      s=vc4_steer(&t,nl,nr,20,um);
      if(pos>=130 && (s>maxs||-s>maxs)) maxs=s<0?-s:s;
      if(!vc4_on_line(&t)) lostx=true;
      um+=2400; }
    printf("   angled cross: max steer after tape %d\n",maxs);
    CHECK(!lostx && maxs<100, "angled unmarked cross: straight on, not lost"); }
  /* real exit still works after a crossing */
  for(int i=0;i<10;i++){ s=vc4_steer(&t,900,50,20,um); um+=2400; }
  { int lx=-1; for(int i=0;i<100;i++){ vc4_steer(&t,50,50,20,um); um+=2400; if(!vc4_on_line(&t)&&lx<0) lx=i; }
    CHECK(lx>=9 && lx<=30, "after a cross, a real exit is still lost/searched"); }
  /* curve: line drifts onto the left sensor; steering must be strong
   * while the sensor is still only part-way on the tape */
  { vc4_reset(&t); um=0; int32_t s2=0;
    for(int i=0;i<4;i++){ s2=vc4_steer(&t,600,100,20,um); um+=2400; }
    printf("   half on tape for 80 ms: steer %d\n",s2);
    CHECK(s2>350, "curve: part-way onto the sensor already steers hard");
    for(int i=0;i<2;i++){ s2=vc4_steer(&t,950,100,20,um); um+=2400; }
    for(int i=0;i<3;i++){ s2=vc4_steer(&t,100,100,20,um); um+=2400; }
    printf("   60 ms after the line left the left sensor: steer %d\n",s2);
    CHECK(s2>300, "curve exit: recovery arc starts at once, not after a coast"); }
  /* Steady curve: hold one smooth turn (model video), not turn /
   * straight / turn, and no weaving across the line. */
  { int sc, stt; double mo; bool lo;
    circle_run(200.0, &sc, &stt, &mo, &lo);
    printf("   circle r=200: lost %d, side changes %d, straight ticks %d/750, max offset %.1f mm\n",
           (int)lo, sc, stt, mo);
    CHECK(!lo && sc == 0 && stt < 40 && mo < 9.0,
          "steady curve: one smooth turn, never back to straight");
    circle_run(150.0, &sc, &stt, &mo, &lo);
    printf("   circle r=150: lost %d, side changes %d, straight ticks %d/750, max offset %.1f mm\n",
           (int)lo, sc, stt, mo);
    CHECK(!lo && sc == 0 && stt < 40 && mo < 9.0,
          "tight curve: one smooth turn, never back to straight");
    circle_run(-150.0, &sc, &stt, &mo, &lo);
    CHECK(!lo && sc == 0 && stt < 40 && mo < 9.0,
          "right curve remains smooth with baseline filter"); }
  printf("cps(120)=%d cps(60)=%d\n", vc4_mm_s_to_cps(120), vc4_mm_s_to_cps(60));
  printf(fails? "FAILED %d\n":"ALL PASS\n",fails); return fails;
}
