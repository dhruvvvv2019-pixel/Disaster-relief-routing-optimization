#include "solver.h"
#include <vector>
#include <string>
#include <array>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <chrono>

using namespace std;

// --- Your Existing Structs and Helper Functions ---

struct AD_Base { double x=0,y=0; int id=0; };
struct AD_Settlement { double x=0,y=0; int id=0; int people=0; int otherNeed=0; int mealNeed=0; };
struct AD_Mission {
    vector<int> path;
    vector<array<int,3>> cargo;
    double distance=0, weight=0, value=0;
    int load_d=0, load_p=0, load_o=0;
};
struct AD_Aircraft {
    double capWeight=0, capDist=0, fixedCost=0, kmCost=0;
    int id=0, origin=0;
    double traveled=0;
    vector<AD_Mission> missions;
};

double limitTime=0, globalDist=0;
double wt_d=1,val_d=0,wt_p=1,val_p=0,wt_o=1,val_o=0;
int numBases=0,numSettle=0,numAir=0;

vector<AD_Base> baseList;
vector<AD_Settlement> settleList;
vector<AD_Aircraft> fleet;
vector<vector<double>> baseToSettle, settleToSettle;

double AD_dist(double a,double b,double c,double d){
    double dx=a-c, dy=b-d;
    double r=dx*dx+dy*dy;
    return sqrt(r);
}
double AD_val(char t){ return t=='d'?val_d:(t=='p'?val_p:val_o); }
double AD_wt(char t){ return t=='d'?wt_d:(t=='p'?wt_p:wt_o); }

double AD_routeDist(const AD_Aircraft &ac,const vector<int> &path){
    if(path.empty()) return 0;
    double s=0;
    s += baseToSettle[ac.origin][path[0]];
    for(int i=1;i<(int)path.size();i++){
        s += settleToSettle[path[i-1]][path[i]];
    }
    s += baseToSettle[ac.origin][path.back()];
    return s;
}

struct AD_Load {
    vector<array<int,3>> cargo;
    double weight=0,value=0;
    int d=0,p=0,o=0;
    bool feasible=true;
};

AD_Load AD_packMission(const vector<int> &path,const vector<int> &nm,const vector<int> &no,double cap){
    AD_Load out;
    out.cargo.assign(path.size(),{0,0,0});
    auto ratio=[&](char t){
        double w=AD_wt(t), v=AD_val(t);
        if(w<=0) return numeric_limits<double>::infinity();
        return v/w;
    };
    bool preferP = ratio('p')>=ratio('d');
    double freeC=cap;
    vector<int> meals, others;
    for(int v:path){ meals.push_back(nm[v]); others.push_back(no[v]); }
    auto take=[&](char t){
        double w=AD_wt(t), v=AD_val(t);
        for(int i=0;i<(int)path.size();i++){
            if(freeC+1e-12<w && w>0) continue;
            if(t=='o'){
                int av=others[i];
                if(av<=0) continue;
                int use=(w>0?min(av,(int)(freeC/w)):av);
                if(use<=0) continue;
                out.cargo[i][2]+=use; out.o+=use; others[i]-=use;
                if(w>0){ freeC-=use*w; out.weight+=use*w; }
                out.value+=use*v;
            }else{
                int av=meals[i];
                if(av<=0) continue;
                int use=(w>0?min(av,(int)(freeC/w)):av);
                if(use<=0) continue;
                if(t=='p') out.cargo[i][1]+=use, out.p+=use;
                else out.cargo[i][0]+=use, out.d+=use;
                meals[i]-=use;
                if(w>0){ freeC-=use*w; out.weight+=use*w; }
                out.value+=use*v;
            }
        }
    };
    if(preferP){ take('p'); take('d'); } else { take('d'); take('p'); }
    take('o');
    if(out.weight>cap+1e-9) out.feasible=false;
    return out;
}

double AD_score(const vector<AD_Aircraft> &fs){
    double val=0,cost=0;
    for(auto &ac:fs){
        for(auto &m:ac.missions){
            for(int i=0;i<(int)m.path.size();i++){
                val += m.cargo[i][0]*val_d + m.cargo[i][1]*val_p + m.cargo[i][2]*val_o;
            }
            if (!m.path.empty()) {
                double tmp = ac.fixedCost + ac.kmCost * m.distance;
                cost += tmp;
            }
        }
    }
    return val-cost;
}

void AD_buildInit(vector<AD_Aircraft> &fs,vector<int> &nm,vector<int> &no){
    for(auto &ac:fs){
        ac.missions.clear();
        ac.traveled=0;
        while(true){
            int pick=-1; double best=1e18;
            for(int v=0; v<numSettle; v++){
                if(nm[v]<=0 && no[v]<=0) continue;
                double d=baseToSettle[ac.origin][v]*2;
                if(d > ac.capDist + 1e-9 || ac.traveled + d > globalDist + 1e-9) continue;
                if(d<best){ best=d; pick=v; }
            }
            if(pick<0) break;
            AD_Mission m; m.path.push_back(pick);
            bool grow=true;
            while(grow){
                grow=false;
                double dist=AD_routeDist(ac,m.path);
                if(dist>ac.capDist+1e-9){ m.path.pop_back(); break; }
                AD_Load ld=AD_packMission(m.path,nm,no,ac.capWeight);
                if(!ld.feasible){ m.path.pop_back(); break; }
                if(ac.traveled+dist>globalDist+1e-9){ m.path.pop_back(); break; }
                int last=m.path.back(), cand=-1; double sc=1e100;
                for(int c=0;c<numSettle;c++){
                    if((nm[c]+no[c])<=0) continue;
                    if(find(m.path.begin(),m.path.end(),c)!=m.path.end()) continue;
                    double d=settleToSettle[last][c];
                    double val=d/(1.0+nm[c]+no[c]);
                    if(val<sc){ sc=val; cand=c; }
                }
                if(cand!=-1){
                    m.path.push_back(cand);
                    double nd=AD_routeDist(ac,m.path);
                    if(nd>ac.capDist+1e-9){ m.path.pop_back(); break; }
                    AD_Load l2=AD_packMission(m.path,nm,no,ac.capWeight);
                    if(!l2.feasible){ m.path.pop_back(); break; }
                    grow=true;
                }
            }
            if(m.path.empty()) break;
            AD_Load f=AD_packMission(m.path,nm,no,ac.capWeight);
            if(!f.feasible) break;
            m.cargo=f.cargo; m.load_d=f.d; m.load_p=f.p; m.load_o=f.o;
            m.weight=f.weight; m.distance=AD_routeDist(ac,m.path);
            for(int i=0;i<(int)m.path.size();i++){
                nm[m.path[i]]-=m.cargo[i][0]+m.cargo[i][1];
                no[m.path[i]]-=m.cargo[i][2];
                if(nm[m.path[i]]<0) nm[m.path[i]]=0;
                if(no[m.path[i]]<0) no[m.path[i]]=0;
            }
            ac.missions.push_back(m);
            ac.traveled+=m.distance;
            if(ac.traveled>=globalDist-1e-9) break;
        }
    }
}

bool AD_neighbor(vector<AD_Aircraft> &fs,vector<int> &nm,vector<int> &no,double &bestObj,chrono::steady_clock::time_point end){
    bool ok=false; double bestDelta=1e-9;
    struct Move{int hi,ti,pos,nv; double nd; AD_Load load;};
    Move bm; bool found=false;
    for(int hi=0;hi<(int)fs.size();hi++){
        auto &ac=fs[hi];
        if(chrono::steady_clock::now()>end) break;
        for(int ti=0;ti<(int)ac.missions.size();ti++){
            if(chrono::steady_clock::now()>end) break;
            auto &m=ac.missions[ti];
            for(int pos=0;pos<(int)m.path.size();pos++){
                int old=m.path[pos];
                auto tm=nm,to=no;
                for(int k=0;k<(int)m.path.size();k++){
                    tm[m.path[k]]+=m.cargo[k][0]+m.cargo[k][1];
                    to[m.path[k]]+=m.cargo[k][2];
                }
                int cand=-1; double sc=1e100;
                for(int c=0;c<numSettle;c++){
                    if(c==old) continue;
                    if((tm[c]+to[c])<=0) continue;
                    if(find(m.path.begin(),m.path.end(),c)!=m.path.end()) continue;
                    double d=settleToSettle[old][c];
                    double val=d/(1.0+tm[c]+to[c]);
                    if(val<sc){ sc=val; cand=c; }
                }
                if(cand==-1) continue;
                auto newP=m.path; newP[pos]=cand;
                double nd=AD_routeDist(ac,newP);
                if(nd>ac.capDist+1e-9) continue;
                double nt=ac.traveled-m.distance+nd;
                if(nt>globalDist+1e-9) continue;
                AD_Load l=AD_packMission(newP,tm,to,ac.capWeight);
                if(!l.feasible) continue;
                double oldVal=0;
                for(int k=0;k<(int)m.path.size();k++)
                    oldVal+=m.cargo[k][0]*val_d+m.cargo[k][1]*val_p+m.cargo[k][2]*val_o;
                double delta=l.value-oldVal-ac.kmCost*(nd-m.distance);
                if(delta>bestDelta){ bestDelta=delta; bm={hi,ti,pos,cand,nd,l}; found=true; }
            }
        }
    }
    if(found){
        auto &ac=fs[bm.hi]; auto &m=ac.missions[bm.ti];
        for(int k=0;k<(int)m.path.size();k++){
            nm[m.path[k]]+=m.cargo[k][0]+m.cargo[k][1];
            no[m.path[k]]+=m.cargo[k][2];
        }
        m.path[bm.pos]=bm.nv;
        m.cargo=bm.load.cargo;
        m.load_d=bm.load.d; m.load_p=bm.load.p; m.load_o=bm.load.o;
        m.weight=bm.load.weight;
        ac.traveled+=bm.nd-m.distance;
        m.distance=bm.nd;
        for(int k=0;k<(int)m.path.size();k++){
            nm[m.path[k]]-=m.cargo[k][0]+m.cargo[k][1];
            no[m.path[k]]-=m.cargo[k][2];
            if(nm[m.path[k]]<0) nm[m.path[k]]=0;
            if(no[m.path[k]]<0) no[m.path[k]]=0;
        }
        ok=true; bestObj=AD_score(fs);
    }
    return ok;
}

Solution solve(const ProblemData& problem) {
    // --- Step 1: Initialize solver data from the 'problem' object ---
    limitTime = problem.time_limit_minutes;
    globalDist = problem.d_max;

    wt_d = problem.packages[0].weight; val_d = problem.packages[0].value;
    wt_p = problem.packages[1].weight; val_p = problem.packages[1].value;
    wt_o = problem.packages[2].weight; val_o = problem.packages[2].value;

    numBases = problem.cities.size();
    baseList.assign(numBases, {});
    for(int i = 0; i < numBases; ++i) {
        baseList[i].x = problem.cities[i].x;
        baseList[i].y = problem.cities[i].y;
        baseList[i].id = i;
    }

    numSettle = problem.villages.size();
    settleList.assign(numSettle, {});
    for(int i = 0; i < numSettle; ++i) {
        settleList[i].x = problem.villages[i].coords.x;
        settleList[i].y = problem.villages[i].coords.y;
        settleList[i].id = problem.villages[i].id - 1;
        settleList[i].people = problem.villages[i].population;
        settleList[i].mealNeed = 9 * problem.villages[i].population;
        settleList[i].otherNeed = 1 * problem.villages[i].population;
    }

    numAir = problem.helicopters.size();
    fleet.assign(numAir, {});
    for(int i = 0; i < numAir; ++i) {
        const auto& heli = problem.helicopters[i];
        fleet[i].id = heli.id - 1;
        fleet[i].origin = heli.home_city_id - 1;
        fleet[i].capWeight = heli.weight_capacity;
        fleet[i].capDist = heli.distance_capacity;
        fleet[i].fixedCost = heli.fixed_cost;
        fleet[i].kmCost = heli.alpha;
    }

    // --- Step 2: Run the core solving logic (Unchanged) ---
    baseToSettle.assign(numBases, vector<double>(numSettle, 0));
    for(int i = 0; i < numBases; i++)
        for(int j = 0; j < numSettle; j++)
            baseToSettle[i][j] = AD_dist(baseList[i].x, baseList[i].y, settleList[j].x, settleList[j].y);

    settleToSettle.assign(numSettle, vector<double>(numSettle, 0));
    for(int i = 0; i < numSettle; i++)
        for(int j = 0; j < numSettle; j++)
            settleToSettle[i][j] = AD_dist(settleList[i].x, settleList[i].y, settleList[j].x, settleList[j].y);

    vector<int> nm(numSettle), no(numSettle);
    for(int i = 0; i < numSettle; i++) {
        nm[i] = settleList[i].mealNeed;
        no[i] = settleList[i].otherNeed;
    }

    AD_buildInit(fleet, nm, no);
    
    auto end = chrono::steady_clock::now() + chrono::milliseconds(long(max(5000.0, limitTime * 60 * 1000 - 1000.0)));
    double obj = AD_score(fleet);

    while(chrono::steady_clock::now() < end && AD_neighbor(fleet, nm, no, obj, end));

    // --- Step 3: Format the output into a Solution object ---
    Solution final_solution;
    for (const auto& ac : fleet) {
        HelicopterPlan plan;
        plan.helicopter_id = ac.id + 1;

        for (const auto& m : ac.missions) {
            Trip trip;
            trip.dry_food_pickup = m.load_d;
            trip.perishable_food_pickup = m.load_p;
            trip.other_supplies_pickup = m.load_o;

            for (size_t k = 0; k < m.path.size(); ++k) {
                Drop drop;
                drop.village_id = m.path[k] + 1;
                drop.dry_food = m.cargo[k][0];
                drop.perishable_food = m.cargo[k][1];
                drop.other_supplies = m.cargo[k][2];
                trip.drops.push_back(drop);
            }
            plan.trips.push_back(trip);
        }
        final_solution.push_back(plan);
    }

    return final_solution;
}