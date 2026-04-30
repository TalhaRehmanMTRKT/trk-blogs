// =============================================================================
//  Post: p2 — Adding the Heat Sector (CHPs, HOB, heat storage, heat trading)
//  File: main.cpp
//
//  Snippet map (referenced from trk-blogs-docs/p2.tex):
//    snippet#1 lines  30-87  : Microgrid input data (electric + heat)
//    snippet#2 lines  88-111 : Decision variables (electric + heat side)
//    snippet#3 lines 112-123 : Objective function
//    snippet#4 lines 124-144 : Generator, CHP and HOB bounds + CHP heat coupling
//    snippet#5 lines 145-181 : Battery and heat-storage dynamics
//    snippet#6 lines 182-196 : Power and heat balance equations
//    snippet#7 lines 197-253 : Solve, report, write CSV
//
//  Author : Talha Rehman <https://github.com/TalhaRehmanMTRKT>
//  Refactored with Claude Opus 4.7
// =============================================================================

#include <ilcplex/ilocplex.h>
#include <chrono>
#include <iostream>
#include <fstream>
ILOSTLBEGIN

int main(int, char**)
{
    auto start = chrono::high_resolution_clock::now();
    IloEnv env;
    IloModel model(env);

    // -------- snippet#1 lines 30-87 : Microgrid input data --------
    int T     = 24;   // One day (hours)

    // Generation costs (per kW)
    int Cdg1  = 135;
    int Cdg2  = 140;
    int Chob  = 80;
    int Cchp1 = 150;
    int Cchp2 = 145;

    // Storage parameters
    float socini = 0.2f;   // Initial battery SoC
    int   Pbmax  = 100;    // Battery capacity (kWh)
    float effin  = 0.95f;  // Battery efficiency
    int   Hssmax = 50;     // Heat-storage capacity (kWh-th)
    float Heffin = 0.95f;  // Heat-storage efficiency

    // CHP heat-to-electric ratios
    float k1 = 1.2f;
    float k2 = 0.8f;

    // Demands
    int* Pload = new int[T] {
        169, 175, 179, 171, 181, 190, 270, 264,
        273, 281, 300, 320, 280, 260, 250, 200,
        180, 190, 240, 280, 325, 350, 300, 250 };

    int* Hload = new int[T] {
        130, 125, 120, 120, 125, 135, 150, 160,
        175, 190, 195, 200, 195, 195, 180, 170,
        185, 190, 195, 200, 195, 190, 180, 175 };

    // Trading prices (electricity and heat)
    int* CGbuy  = new int[T] {
        138, 139, 143, 149, 150, 152, 155, 158,
        160, 154, 153, 153, 152, 150, 149, 149,
        154, 156, 163, 164, 164, 160, 150, 148 };

    int* CGsell = new int[T] {
        128, 129, 133, 139, 140, 142, 145, 148,
        150, 144, 143, 143, 142, 140, 139, 139,
        144, 146, 153, 154, 154, 150, 140, 138 };

    int* CHbuy  = new int[T] {
        77, 77, 77, 77, 77, 77, 77, 77,
        83, 83, 83, 83, 83, 83, 83, 83,
        83, 83, 78, 78, 78, 78, 78, 78 };

    int* CHsell = new int[T] {
        75, 75, 75, 75, 75, 75, 75, 75,
        80, 80, 80, 80, 80, 80, 80, 80,
        80, 80, 78, 78, 78, 78, 78, 78 };

    // PV forecast
    float* Rdg1 = new float[T] {
        0,0,0,0,0,0,0,10,15,20,23,28,
        33,35,34,31,28,10,0,0,0,0,0,0 };

    // -------- snippet#2 lines 88-111 : Decision variables --------
    // Electric side
    IloNumVarArray  PGbuy  (env, T, 0, IloInfinity);
    IloNumVarArray  PGsell (env, T, 0, IloInfinity);
    IloNumVarArray  statoc (env, T, 0, 1);
    IloNumVarArray  Bchg   (env, T, 0, 50);
    IloNumVarArray  Bdischg(env, T, 0, 50);
    IloBoolVarArray ubat   (env, T);              // 1 = charge, 0 = discharge
    IloNumVarArray  Pdg1   (env, T, 0, 100, ILOINT);
    IloNumVarArray  Pdg2   (env, T, 0,  80, ILOINT);
    IloNumVarArray  Pchp1  (env, T, 30, 60, ILOINT);
    IloNumVarArray  Pchp2  (env, T, 50,100, ILOINT);

    // Heat side
    IloNumVarArray  HGbuy  (env, T, 0, IloInfinity, ILOINT);
    IloNumVarArray  HGsell (env, T, 0, IloInfinity, ILOINT);
    IloNumVarArray  HSSsoc (env, T, 0, 1);
    IloNumVarArray  Hchg   (env, T, 0, 50, ILOINT);
    IloNumVarArray  Hdischg(env, T, 0, 50, ILOINT);
    IloBoolVarArray uhst   (env, T);              // 1 = charge, 0 = discharge
    IloNumVarArray  Hhob   (env, T, 0, 80, ILOINT);
    IloNumVarArray  Hchp1  (env, T, 0, IloInfinity);
    IloNumVarArray  Hchp2  (env, T, 0, IloInfinity);

    // -------- snippet#3 lines 112-123 : Objective function --------
    IloExpr objective(env);
    for (int t = 0; t < T; t++)
    {
        objective += Cdg1  * Pdg1[t]  + Cdg2  * Pdg2[t]
                   + Cchp1 * Pchp1[t] + Cchp2 * Pchp2[t]
                   + Chob  * Hhob[t]
                   + CGbuy[t] * PGbuy[t] - CGsell[t] * PGsell[t]
                   + CHbuy[t] * HGbuy[t] - CHsell[t] * HGsell[t];
    }
    model.add(IloMinimize(env, objective));

    // -------- snippet#4 lines 124-144 : Generator / CHP / HOB bounds and heat coupling --------
    for (int t = 0; t < T; t++)
    {
        // Dispatchable-generator output bounds
        model.add(Pdg1[t] <= 100);
        model.add(Pdg2[t] <=  80);

        // Heat-only boiler bound
        model.add(Hhob[t] <= 80);

        // CHP electric output bands (must be on, between min and max)
        model.add(Pchp1[t] >= 30);
        model.add(Pchp1[t] <= 60);
        model.add(Pchp2[t] >= 50);
        model.add(Pchp2[t] <= 100);

        // CHP heat output is locked to its electric output through a fixed ratio
        model.add(Hchp1[t] == k1 * Pchp1[t]);
        model.add(Hchp2[t] == k2 * Pchp2[t]);
    }

    // -------- snippet#5 lines 145-181 : Battery and heat-storage dynamics --------
    for (int t = 0; t < T; t++)
    {
        // Big-M mutual exclusion: charge XOR discharge for each storage unit
        model.add(Bchg[t]    <= 1e6 *      ubat[t]);
        model.add(Bdischg[t] <= 1e6 * (1 - ubat[t]));
        model.add(Hchg[t]    <= 1e6 *      uhst[t]);
        model.add(Hdischg[t] <= 1e6 * (1 - uhst[t]));

        if (t == 0)
        {
            // Battery (electric)
            model.add(statoc[t] == socini
                      + ((effin * Bchg[t] - Bdischg[t] / effin) / Pbmax));
            model.add(Bchg[t]    <= (Pbmax * (1 - socini) / effin));
            model.add(Bdischg[t] <= (Pbmax * socini * effin));

            // Heat storage (assumed half-full at t=0)
            model.add(HSSsoc[t] == 0.5
                      + ((Heffin * Hchg[t] - Hdischg[t] / Heffin) / Hssmax));
            model.add(Hchg[t]    <= (Hssmax * 0.5 / Heffin));
            model.add(Hdischg[t] <= (Hssmax * 0.5 * Heffin));
        }
        else
        {
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

    // -------- snippet#6 lines 182-196 : Power and heat balance equations --------
    for (int t = 0; t < T; t++)
    {
        // Electric balance
        model.add(Pdg1[t] + Pdg2[t] + Rdg1[t]
                  + Pchp1[t] + Pchp2[t]
                  + Bdischg[t] - Bchg[t]
                  + PGbuy[t]   - PGsell[t] == Pload[t]);

        // Heat balance
        model.add(Hhob[t] + Hchp1[t] + Hchp2[t]
                  + HGbuy[t] - HGsell[t]
                  - Hchg[t]  + Hdischg[t] == Hload[t]);
    }

    // -------- snippet#7 lines 197-253 : Solve, report, write CSV --------
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

    std::ofstream outputFile("output.csv");
    if (outputFile.is_open()) {
        outputFile << "Time,Pload,Hload,CGbuy,CGsell,CHsell,CHbuy,Rdg1,"
                      "PGbuy,PGsell,statoc,Bchg,Bdischg,Pdg1,Pdg2,"
                      "Pchp1,Pchp2,Hhob,Hchp1,Hchp2,HGbuy,HGsell,Hchg,Hdischg"
                   << std::endl;

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
                       <<  cplex.getValue(Hdischg[i])  << std::endl;
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
