/* Host tests for app_program/vc_follow.c (line following v4). */
#include <stdio.h>
#include <stdbool.h>
#include "../../app_program/vc_follow.h"
static int fails;
#define CHECK(c,m) do{ if(!(c)){printf("FAIL: %s\n",m);fails++;} else printf("ok: %s\n",m);}while(0)
int main(void){
  vc4_track_t t; uint32_t um=0; int32_t s; int32_t sp, st;
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
  printf("cps(120)=%d cps(60)=%d\n", vc4_mm_s_to_cps(120), vc4_mm_s_to_cps(60));
  printf(fails? "FAILED %d\n":"ALL PASS\n",fails); return fails;
}
