// =============================================================================
//  Post: p1 — Introduction to Optimization with C++/CPLEX
//  File: main.cpp
//
//  Snippet map (referenced from trk-blogs-docs/p1.tex):
//    snippet#1 lines  29-59  : Microgrid input data
//    snippet#2 lines  61-71  : Decision variables
//    snippet#3 lines  73-82  : Objective function
//    snippet#4 lines  84-113 : Constraints (DG bounds, battery, power balance)
//    snippet#5 lines 115-132 : Solve and report
//    snippet#6 lines 134-159 : Write results to CSV
//
//  When you reference a snippet in the .tex, use \codesnippet{N}{...}{nnn}{nnn}
//  with the line numbers shown above. Keep them in sync if you edit this file.
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

    // -------- snippet#1 lines 29-59 : Microgrid input data --------
    int T     = 24;   // One day (hours)
    int Cdg1  = 80;   // Cost per kW from DG-1
    int Cdg2  = 90;   // Cost per kW from DG-2

    int* Pload = new int[T] {
        169, 175, 179, 171, 181, 172, 270, 264,
        273, 281, 193, 158, 161, 162, 250, 260,
        267, 271, 284, 167, 128, 134, 144, 150 };

    int* CGbuy = new int[T] {
         90,  90,  90,  90,  90,  90, 110, 110,
        110, 110, 110, 125, 125, 125, 125, 125,
        125, 125, 110, 110, 110, 110, 110, 110 };

    int* CGsell = new int[T] {
         70,  70,  70,  70,  70,  70,  90,  90,
         90,  90,  90, 105, 105, 105, 105, 105,
        105, 105,  90,  90,  90,  90,  90,  90 };

    float* Rdg1 = new float[T] {
        0,0,0,0,0,0,0,10,15,20,23,28,
        33,35,34,31,28,10,0,0,0,0,0,0 };

    float* Rdg2 = new float[T] {
        0,0,0,0,0,0,0,10,15,20,23,28,
        33,35,34,31,28,10,0,0,0,0,0,0 };

    float socini = 0.2f;   // Initial battery SoC
    int   Pbmax  = 200;    // Battery capacity (kWh)
    float effin  = 0.95f;  // Battery efficiency

    // -------- snippet#2 lines 61-71 : Decision variables --------
    IloNumVarArray PGbuy  (env, T, 0, IloInfinity); // Grid power bought
    IloNumVarArray PGsell (env, T, 0, IloInfinity); // Grid power sold

    IloNumVarArray statoc (env, T, 0, 1);   // Battery state of charge

    IloNumVarArray Bchg   (env, T, 0, 100); // Battery charging
    IloNumVarArray Bdischg(env, T, 0, 100); // Battery discharging

    IloNumVarArray Pdg1   (env, T, 0, 80);  // DG-1 output
    IloNumVarArray Pdg2   (env, T, 0, 100); // DG-2 output

    // -------- snippet#3 lines 73-82 : Objective function --------
    IloExpr objective(env);
    for (int t = 0; t < T; t++)
    {
        objective += Cdg1 * Pdg1[t]
                   + Cdg2 * Pdg2[t]
                   + CGbuy[t]  * PGbuy[t]
                   - CGsell[t] * PGsell[t];
    }
    model.add(IloMinimize(env, objective));

    // -------- snippet#4 lines 84-113 : Constraints --------
    for (int t = 0; t < T; t++)
    {
        // DG output bounds
        model.add(0 <= Pdg1[t]);  model.add(Pdg1[t] <= 80);
        model.add(0 <= Pdg2[t]);  model.add(Pdg2[t] <= 90);

        // Battery state of charge bounds
        model.add(0 <= statoc[t]); model.add(statoc[t] <= 1);

        if (t == 0)
        {
            model.add(statoc[t] == socini
                      + ((effin * Bchg[t] - (Bdischg[t] / effin)) / Pbmax));
            model.add(Bchg[t]    <= (Pbmax * (1 - socini) / effin));
            model.add(Bdischg[t] <= (Pbmax * socini * effin));
        }
        else
        {
            model.add(statoc[t] == statoc[t - 1]
                      + ((effin * Bchg[t] - (Bdischg[t] / effin)) / Pbmax));
            model.add(Bchg[t]    <= (Pbmax * (1 - statoc[t - 1])) / effin);
            model.add(Bdischg[t] <= Pbmax * statoc[t - 1] * effin);
        }

        // Power balance
        model.add(Pdg1[t] + Pdg2[t] + Rdg1[t] + Rdg2[t]
                  + Bdischg[t] - Bchg[t]
                  + PGbuy[t]   - PGsell[t] == Pload[t]);
    }

    // -------- snippet#5 lines 115-132 : Solve and report --------
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

    // -------- snippet#6 lines 134-159 : Write results to CSV --------
    std::ofstream outputFile("output.csv");
    if (outputFile.is_open()) {
        outputFile << "Time,Pload,CGbuy,CGsell,Rdg1,Rdg2,"
                      "PGbuy,PGsell,statoc,Bchg,Bdischg,Pdg1,Pdg2" << std::endl;

        for (int i = 0; i < T; i++) {
            outputFile << i + 1                 << ","
                       << Pload[i]              << ","
                       << CGbuy[i]              << ","
                       << CGsell[i]             << ","
                       << Rdg1[i]               << ","
                       << Rdg2[i]               << ","
                       <<  cplex.getValue(PGbuy[i])   << ","
                       << -cplex.getValue(PGsell[i])  << ","
                       <<  cplex.getValue(statoc[i])  << ","
                       << -cplex.getValue(Bchg[i])    << ","
                       <<  cplex.getValue(Bdischg[i]) << ","
                       <<  cplex.getValue(Pdg1[i])    << ","
                       <<  cplex.getValue(Pdg2[i])    << std::endl;
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
