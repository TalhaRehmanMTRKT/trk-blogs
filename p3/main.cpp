// =============================================================================
//  Post: p3 — Adding Electric Vehicles as Flexible Storage
//  File: main.cpp
//
//  Snippet map (referenced from trk-blogs-docs/p3.tex):
//    snippet#1 lines  47-112 : EV fleet input data (arrival, departure, capacity)
//    snippet#2 lines 113-184 : 2-D decision variables for EV charge/discharge/SoC
//    snippet#3 lines 185-227 : EV scheduling constraints (per vehicle, per hour)
//    snippet#4 lines 228-269 : Power balance updated with the EV fleet term
//    snippet#5 lines 270-315 : CSV output extended with per-vehicle columns
//
//  Author : Talha Rehman <https://github.com/TalhaRehmanMTRKT>
//  Refactored with Claude Opus 4.7
// =============================================================================

#include <ilcplex/ilocplex.h>
#include <chrono>
#include <iostream>
#include <fstream>
ILOSTLBEGIN

typedef IloArray<IloNumVarArray>  NumVar2D;
typedef IloArray<IloBoolVarArray> BoolVar2D;

int main(int, char**)
{
    auto start = chrono::high_resolution_clock::now();
    IloEnv env;
    IloModel model(env);

    // ===== Input data =====
    int T     = 24;
    int Cdg1  = 135;
    int Cdg2  = 140;
    int Chob  = 80;
    int Cchp1 = 150;
    int Cchp2 = 145;

    float socini = 0.2f;
    int   Pbmax  = 100;
    float effin  = 0.95f;
    int   Hssmax = 50;
    float Heffin = 0.95f;
    float k1 = 1.2f;
    float k2 = 0.8f;

    // -------- snippet#1 lines 47-112 : EV fleet input data --------
    int   numEvs   = 5;                        // Total number of EVs
    float Eveffin  = 0.90f;                    // EV charge/discharge efficiency

    // Per-vehicle arrival hour, departure hour, initial SoC and battery capacity.
    // EV n is grid-connected for t in [ta[n], td[n]]; outside that window
    // its charge and discharge powers are clamped to zero.
    int*   ta       = new int[numEvs]   { 2, 4, 6, 9, 9 };
    int*   td       = new int[numEvs]   { 6, 7, 9, 12, 16 };
    float* evsocini = new float[numEvs] { 0.30f, 0.20f, 0.10f, 0.70f, 0.40f };
    float* evcap    = new float[numEvs] { 38.3f, 47.5f, 28.9f, 56.0f, 52.0f };

    int* Pload = new int[T] {
        169, 175, 179, 171, 181, 190, 270, 264,
        273, 281, 300, 320, 280, 260, 250, 200,
        180, 190, 240, 280, 325, 350, 300, 250 };

    int* Hload = new int[T] {
        130, 125, 120, 120, 125, 135, 150, 160,
        175, 190, 195, 200, 195, 195, 180, 170,
        185, 190, 195, 200, 195, 190, 180, 175 };

    int* CGbuy  = new int[T] {
        138, 139, 143, 149, 150, 152, 155, 158,
        160, 154, 153, 153, 152, 150, 149, 149,
        154, 156, 163, 164, 164, 160, 150, 148 };

    int* CGsell = new int[T] {
        128, 129, 133, 139, 140, 142, 145, 148,
        150, 144, 143, 143, 142, 140, 139, 139,
        144, 146, 153, 154, 154, 150, 140, 138 };

    int* CHbuy  = new int[T] {
        77,77,77,77,77,77,77,77,
        83,83,83,83,83,83,83,83,
        83,83,78,78,78,78,78,78 };

    int* CHsell = new int[T] {
        75,75,75,75,75,75,75,75,
        80,80,80,80,80,80,80,80,
        80,80,78,78,78,78,78,78 };

    float* Rdg1 = new float[T] {
        0,0,0,0,0,0,0,10,15,20,23,28,
        33,35,34,31,28,10,0,0,0,0,0,0 };

    // ===== Microgrid decision variables (electric + heat side, same as p2) =====
    IloNumVarArray PGbuy  (env, T, 0, IloInfinity);
    IloNumVarArray PGsell (env, T, 0, IloInfinity);
    IloNumVarArray statoc (env, T, 0, 1);
    IloNumVarArray Bchg   (env, T, 0, 100);
    IloNumVarArray Bdischg(env, T, 0, 100);
    IloNumVarArray Pdg1   (env, T, 0, 100, ILOINT);
    IloNumVarArray Pdg2   (env, T, 0,  80, ILOINT);
    IloNumVarArray Pchp1  (env, T, 30, 60, ILOINT);
    IloNumVarArray Pchp2  (env, T, 50,100, ILOINT);

    IloNumVarArray HGbuy  (env, T, 0, IloInfinity, ILOINT);
    IloNumVarArray HGsell (env, T, 0, IloInfinity, ILOINT);
    IloNumVarArray HSSsoc (env, T, 0, 1);
    IloNumVarArray Hchg   (env, T, 0, 50, ILOINT);
    IloNumVarArray Hdischg(env, T, 0, 50, ILOINT);
    IloNumVarArray Hhob   (env, T, 0, 80, ILOINT);
    IloNumVarArray Hchp1  (env, T, 0, IloInfinity);
    IloNumVarArray Hchp2  (env, T, 0, IloInfinity);

    // -------- snippet#2 lines 113-184 : 2-D EV decision variables --------
    // For each EV n we allocate three length-T variable arrays. The charge and
    // discharge power are upper-bounded by the vehicle's own capacity; the SoC
    // is dimensionless in [0,1] (fraction of capacity). A binary uev per
    // vehicle enforces charge XOR discharge via Big-M.
    IloBoolVarArray ubat(env, T);   // battery charge/discharge selector
    IloBoolVarArray uhst(env, T);   // heat-storage charge/discharge selector
    NumVar2D  Pevchg   (env, numEvs);
    NumVar2D  Pevdischg(env, numEvs);
    NumVar2D  evsoc    (env, numEvs);
    BoolVar2D uev      (env, numEvs);
    for (int n = 0; n < numEvs; n++)
    {
        Pevchg[n]    = IloNumVarArray (env, T, 0, evcap[n], ILOFLOAT);
        Pevdischg[n] = IloNumVarArray (env, T, 0, evcap[n], ILOFLOAT);
        evsoc[n]     = IloNumVarArray (env, T, 0, 1,        ILOFLOAT);
        uev[n]       = IloBoolVarArray(env, T);
    }

    // ===== Objective (same cost terms as p2; EV charge/discharge has no own price) =====
    IloExpr objective(env);
    for (int t = 0; t < T; t++)
    {
        objective += Cdg1  * Pdg1[t]  + Cdg2  * Pdg2[t]
                   + Cchp1 * Pchp1[t] + Cchp2 * Pchp2[t]
                   + Chob  * Hhob[t]
                   + CGbuy[t]  * PGbuy[t]  - CGsell[t] * PGsell[t]
                   + CHbuy[t]  * HGbuy[t]  - CHsell[t] * HGsell[t];
    }
    model.add(IloMinimize(env, objective));

    // ===== Microgrid constraints (same as p2) =====
    for (int t = 0; t < T; t++)
    {
        model.add(Pdg1[t]  <= 100);
        model.add(Pdg2[t]  <=  80);
        model.add(Hhob[t]  <=  80);
        model.add(Pchp1[t] >=  30); model.add(Pchp1[t] <=  60);
        model.add(Pchp2[t] >=  50); model.add(Pchp2[t] <= 100);

        model.add(Hchp1[t] == k1 * Pchp1[t]);
        model.add(Hchp2[t] == k2 * Pchp2[t]);

        // Big-M mutual exclusion: charge XOR discharge for each storage unit
        model.add(Bchg[t]    <= 50 *      ubat[t]);
        model.add(Bdischg[t] <= 50 * (1 - ubat[t]));
        model.add(Hchg[t]    <= 50 *      uhst[t]);
        model.add(Hdischg[t] <= 50 * (1 - uhst[t]));

        if (t == 0) {
            model.add(statoc[t] == socini
                      + ((effin * Bchg[t] - Bdischg[t] / effin) / Pbmax));
            model.add(Bchg[t]    <= (Pbmax * (1 - socini) / effin));
            model.add(Bdischg[t] <= (Pbmax * socini * effin));

            model.add(HSSsoc[t] == 0.5
                      + ((Heffin * Hchg[t] - Hdischg[t] / Heffin) / Hssmax));
            model.add(Hchg[t]    <= (Hssmax * 0.5 / Heffin));
            model.add(Hdischg[t] <= (Hssmax * 0.5 * Heffin));
        } else {
            model.add(statoc[t] == statoc[t - 1]
                      + ((effin * Bchg[t] - Bdischg[t] / effin) / Pbmax));
            model.add(Bchg[t]    <= (Pbmax * (1 - statoc[t - 1])) / effin);
            model.add(Bdischg[t] <= Pbmax * statoc[t - 1] * effin);

            model.add(HSSsoc[t] == HSSsoc[t - 1]
                      + ((Heffin * Hchg[t] - Hdischg[t] / Heffin) / Hssmax));
            model.add(Hchg[t]    <= (Hssmax * (1 - HSSsoc[t - 1]) / Heffin));
            model.add(Hdischg[t] <= Hssmax * HSSsoc[t - 1] * Heffin);
        }
    }

    // -------- snippet#3 lines 185-227 : EV scheduling constraints --------
    // For every vehicle n and every hour t we have three regimes:
    //   1. t == ta[n]            : connection hour, SoC starts from evsocini[n].
    //   2. ta[n] <  t <= td[n]   : connected, SoC follows charge/discharge.
    //   3. otherwise (disconnected): charge and discharge are forced to zero.
    // We additionally require evsoc[n][td[n]] >= 0.5 so the vehicle leaves with
    // at least half a battery — the price the microgrid pays for using it as
    // flexibility is delivering it back charged.
    for (int t = 0; t < T; t++)
    {
        for (int n = 0; n < numEvs; n++)
        {
            if (t == ta[n])
            {
                model.add(evsoc[n][t] == evsocini[n]
                          + (Eveffin * Pevchg[n][t] - Pevdischg[n][t] / Eveffin) / evcap[n]);
                model.add(Pevchg[n][t]    <= evcap[n] * (1 - evsocini[n]) / Eveffin);
                model.add(Pevdischg[n][t] <= evcap[n] * evsocini[n] * Eveffin);
                // Big-M mutex: per-vehicle charge XOR discharge
                model.add(Pevchg[n][t]    <= evcap[n] *      uev[n][t]);
                model.add(Pevdischg[n][t] <= evcap[n] * (1 - uev[n][t]));
            }
            else if (t > ta[n] && t <= td[n])
            {
                model.add(evsoc[n][t] == evsoc[n][t - 1]
                          + (Eveffin * Pevchg[n][t] - Pevdischg[n][t] / Eveffin) / evcap[n]);
                model.add(Pevchg[n][t]    <= evcap[n] * (1 - evsoc[n][t - 1]) / Eveffin);
                model.add(Pevdischg[n][t] <= evcap[n] * evsoc[n][t - 1] * Eveffin);
                // Big-M mutex: per-vehicle charge XOR discharge
                model.add(Pevchg[n][t]    <= evcap[n] *      uev[n][t]);
                model.add(Pevdischg[n][t] <= evcap[n] * (1 - uev[n][t]));

                if (t == td[n])
                    model.add(evsoc[n][t] >= 0.5);
            }
            else
            {
                model.add(Pevchg[n][t]    == 0);
                model.add(Pevdischg[n][t] == 0);
            }
        }
    }

    // -------- snippet#4 lines 228-269 : Power balance with EV fleet term --------
    for (int t = 0; t < T; t++)
    {
        IloExpr evCharge   (env);
        IloExpr evDischarge(env);
        for (int n = 0; n < numEvs; n++) {
            evCharge    += Pevchg[n][t];
            evDischarge += Pevdischg[n][t];
        }

        // Electric balance: EV charging is a load, EV discharging is a source.
        model.add(Pdg1[t] + Pdg2[t] + Rdg1[t]
                  + Pchp1[t] + Pchp2[t]
                  + Bdischg[t] - Bchg[t]
                  + evDischarge - evCharge
                  + PGbuy[t] - PGsell[t] == Pload[t]);

        // Heat balance is unchanged with respect to p2.
        model.add(Hhob[t] + Hchp1[t] + Hchp2[t]
                  + HGbuy[t] - HGsell[t]
                  - Hchg[t]  + Hdischg[t] == Hload[t]);
    }

    // ===== Solve =====
    IloCplex cplex(env);
    cplex.extract(model);
    cplex.exportModel("ModelLP.lp");
    cplex.setOut(env.getNullStream());

    if (!cplex.solve()) {
        env.error() << "Failed" << endl;
        throw(-1);
    }

    double obj = cplex.getObjValue();
    auto end   = chrono::high_resolution_clock::now();
    auto ms    = chrono::duration_cast<chrono::milliseconds>(end - start);

    cout << "\n\tElapsed time (ms): " << ms.count() << endl;
    cout << "Solution status     : " << cplex.getStatus() << endl;
    cout << "Minimized objective : " << obj << endl;

    // -------- snippet#5 lines 270-315 : CSV output extended with per-vehicle columns --------
    std::ofstream outputFile("output.csv");
    if (outputFile.is_open()) {
        outputFile << "Time,Pload,Hload,CGbuy,CGsell,CHsell,CHbuy,Rdg1,"
                      "PGbuy,PGsell,statoc,Bchg,Bdischg,Pdg1,Pdg2,"
                      "Pchp1,Pchp2,Hhob,Hchp1,Hchp2,HGbuy,HGsell,Hchg,Hdischg";
        for (int n = 0; n < numEvs; n++)
            outputFile << ",Pevchg" << n + 1 << ",Pevdischg" << n + 1;
        outputFile << std::endl;

        for (int i = 0; i < T; i++) {
            outputFile << i + 1 << "," << Pload[i] << "," << Hload[i] << ","
                       << CGbuy[i]  << "," << CGsell[i] << ","
                       << CHsell[i] << "," << CHbuy[i]  << "," << Rdg1[i] << ","
                       <<  cplex.getValue(PGbuy[i])    << ","
                       << -cplex.getValue(PGsell[i])   << ","
                       <<  cplex.getValue(statoc[i])   << ","
                       << -cplex.getValue(Bchg[i])     << ","
                       <<  cplex.getValue(Bdischg[i])  << ","
                       <<  cplex.getValue(Pdg1[i])     << ","
                       <<  cplex.getValue(Pdg2[i])     << ","
                       <<  cplex.getValue(Pchp1[i])    << ","
                       <<  cplex.getValue(Pchp2[i])    << ","
                       <<  cplex.getValue(Hhob[i])     << ","
                       <<  cplex.getValue(Hchp1[i])    << ","
                       <<  cplex.getValue(Hchp2[i])    << ","
                       <<  cplex.getValue(HGbuy[i])    << ","
                       << -cplex.getValue(HGsell[i])   << ","
                       << -cplex.getValue(Hchg[i])     << ","
                       <<  cplex.getValue(Hdischg[i]);
            for (int n = 0; n < numEvs; n++) {
                outputFile << "," << -cplex.getValue(Pevchg[n][i])
                           << "," <<  cplex.getValue(Pevdischg[n][i]);
            }
            outputFile << std::endl;
        }
        outputFile.close();
        std::cout << "Data saved to output.csv" << std::endl;
    }
    else {
        std::cerr << "Failed to open output.csv for writing." << std::endl;
    }

    env.end();
    return 0;
}
