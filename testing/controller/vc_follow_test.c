/* Host tests for app_program/vc_follow.c (line following v4). */
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "../../app_program/vc_follow.h"
static int fails;
/* Bare mat: B3 floor tracking makes it read ~0 (below VC4_WHITE). */
#define FLOOR 30u
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
  /* A crossbar must cancel an existing curve immediately. */
  for(int side=-1;side<=1;side+=2){
    vc4_reset(&t);
    t.filt=side*900; t.bias=side*700; t.exit_side=(int8_t)side;
    t.prev_exit=(int8_t)side; t.lost=true;
    s=vc4_steer(&t,900,650,20,0);
    CHECK(s==0 && !t.lost && t.exit_side==0 && t.bias==0,
          "junction cancels stale curve and loss immediately");
  }
  /* Low-contrast alternating sensor noise on an otherwise straight line. */
  vc4_reset(&t);
  { int peak=0;
    for(int i=0;i<100;i++){
      s=vc4_steer(&t,i%2?100:FLOOR,i%2?FLOOR:100,20,(uint32_t)i*2400);
      if(s>peak) peak=s;
      if(-s>peak) peak=-s;
    }
    CHECK(peak<50, "small alternating edge readings produce gentle steering");
  }
  {
    vc4_track_t left, right;
    int32_t ls=0, rs=0, lsp, lst, rsp, rst;
    vc4_reset(&left); vc4_reset(&right);
    for(int i=0;i<4;i++){
      ls=vc4_steer(&left,600,FLOOR,20,(uint32_t)i*2000);
      rs=vc4_steer(&right,FLOOR,600,20,(uint32_t)i*2000);
    }
    CHECK(rs<=-589, "right curve entry steers at least as hard as before");
    CHECK(ls==-rs && ls<=VC4_STEER_MAX,
          "left and right curve entry retain the same filter response");
    vc4_wheels(500,120,100,&lsp,&lst);
    vc4_wheels(-500,120,100,&rsp,&rst);
    CHECK(lsp==rsp && lst==-rst,
          "left adjustment leaves wheel split and speed mapping unchanged");
    for(int i=0;i<20;i++) ls=vc4_steer(&left,FLOOR,FLOOR,20,8000+(uint32_t)i*2000);
    CHECK(vc4_on_line(&left) && ls<200,
          "partial left correction still releases toward straight");
  }
  vc4_wheels(1000,60,100,&sp,&st);
  /* A sustained left bend need not reach the full-black exit threshold. */
  vc4_reset(&t); um=0;
  for(int i=0;i<12;i++){
    vc4_steer(&t,650,FLOOR,20,um); um+=2400;
  }
  CHECK(t.exit_side==1, "sustained partial left tape arms curve recovery");
  for(int i=0;i<8;i++){
    s=vc4_steer(&t,FLOOR,FLOOR,20,um); um+=2400;
  }
  CHECK(s>400 && vc4_on_line(&t), "left bend keeps turning through floor gap");
  vc4_steer(&t,FLOOR,900,20,um);
  CHECK(t.exit_side==-1, "opposite tape replaces left recovery");
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
  for(int i=0;i<500;i++){ s=vc4_steer(&t,FLOOR,FLOOR,20,um); um+=2400; }
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
  CHECK(st<=-790 && sp*(1000+st)/1000>=150,
        "tight bend permits 80 percent split with usable inner-wheel speed");
  /* line leaves past left sensor: coast then arc left, lost after 200ms+60mm */
  int lost_at=-1;
  for(int i=0;i<100;i++){ s=vc4_steer(&t,50,50,20,um); um+=2400; if(!vc4_on_line(&t)&&lost_at<0) lost_at=i; }
  CHECK(lost_at>=9 && lost_at<=30, "lost after black exit + 200ms/60mm");
  printf("   lost at tick %d, recovery steer %d\n",lost_at,s);
  CHECK(s>0, "recovery arc toward exit side (left)");
  /* found again */
  s=vc4_steer(&t,800,FLOOR,20,um);
  CHECK(vc4_on_line(&t), "found again on black");
  /* The left exit in the robot log faded through the sensor for about
   * 250 ms before both sensors went white. Keep only left-side memory over
   * that interval; leave the known-good right-side freshness unchanged. */
  vc4_reset(&t); um=0;
  vc4_steer(&t,884,FLOOR,20,um); um+=2400;
  for(int i=0;i<12;i++){
    vc4_steer(&t,500,FLOOR,20,um); um+=2400;
  }
  s=vc4_steer(&t,FLOOR,FLOOR,20,um);
  CHECK(t.exit_side==1, "left exit survives a 240 ms partial fade");
  for(int i=0;i<5;i++){
    s=vc4_steer(&t,FLOOR,FLOOR,20,um); um+=2400;
  }
  CHECK(s>200, "left steering is retained after the logged fade");

  vc4_reset(&t); um=0;
  vc4_steer(&t,FLOOR,884,20,um); um+=2400;
  for(int i=0;i<12;i++){
    vc4_steer(&t,FLOOR,500,20,um); um+=2400;
  }
  vc4_steer(&t,FLOOR,FLOOR,20,um);
  CHECK(t.exit_side==0, "right exit retains the original 200 ms freshness");

  vc4_reset(&t); um=0;
  vc4_steer(&t,884,FLOOR,20,um); um+=2400;
  for(int i=0;i<25;i++){
    vc4_steer(&t,500,FLOOR,20,um); um+=2400;
  }
  vc4_steer(&t,FLOOR,FLOOR,20,um);
  CHECK(t.exit_side==0, "left exit memory still expires after a long fade");
  /* grey graze then white: not lost */
  vc4_reset(&t);
  for(int i=0;i<5;i++) vc4_steer(&t,500,FLOOR,20,um);
  for(int i=0;i<100;i++){ vc4_steer(&t,FLOOR,FLOOR,20,um); um+=2400; }
  CHECK(vc4_on_line(&t), "grey graze then white = centred, not lost");
  /* junction: both black for 18 mm then 30 mm -> CROSS */
  vc4_reset(&t); um=0; vc4_jct_t j=0, got=0;
  for(int i=0;i<200;i++){ uint32_t pos=um/1000; uint32_t nl=FLOOR,nr=FLOOR;
    if(pos>=100 && pos<118){nl=1000;nr=1000;}
    j=vc4_junction(&t,nl,nr,um); if(j) got=j; um+=2400; }
  CHECK(got==VC4_JCT_CROSS, "18 mm crossing tape -> CROSS");
  /* Angled crossing: one sensor only partly covers the tape. */
  vc4_reset(&t); um=0; got=0;
  for(int i=0;i<200;i++){ uint32_t pos=um/1000; uint32_t nl=FLOOR,nr=FLOOR;
    if(pos>=100 && pos<118){nl=450;nr=650;}
    j=vc4_junction(&t,nl,nr,um); if(j) got=j; um+=2000; }
  CHECK(got==VC4_JCT_CROSS, "partial-coverage crossing -> CROSS");
  /* narrow (5mm) -> nothing */
  vc4_reset(&t); um=0; got=0;
  for(int i=0;i<200;i++){ uint32_t pos=um/1000; uint32_t nl=FLOOR,nr=FLOOR;
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
    for(int i=0;i<300;i++){ uint32_t pos=um/1000; uint32_t nl=FLOOR,nr=FLOOR;
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
    for(int i=0;i<4;i++){ s2=vc4_steer(&t,600,FLOOR,20,um); um+=2400; }
    printf("   half on tape for 80 ms: steer %d\n",s2);
    CHECK(s2>350, "curve: part-way onto the sensor already steers hard");
    for(int i=0;i<2;i++){ s2=vc4_steer(&t,950,FLOOR,20,um); um+=2400; }
    for(int i=0;i<3;i++){ s2=vc4_steer(&t,FLOOR,FLOOR,20,um); um+=2400; }
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
  for(int dir=-1;dir<=1;dir+=2){
    int sc, stt; double mo; bool lo;
    circle_run(dir*85.0, &sc, &stt, &mo, &lo);
    printf("   circle r=%d: lost %d, side changes %d, straight %d, offset %.1f mm\n",
           dir*85, (int)lo, sc, stt, mo);
    CHECK(!lo && sc==0 && stt<40 && mo<9.0, "85 mm circle stays on line");
  }
  printf("cps(120)=%d cps(60)=%d\n", vc4_mm_s_to_cps(120), vc4_mm_s_to_cps(60));
  printf(fails? "FAILED %d\n":"ALL PASS\n",fails); return fails;
}
