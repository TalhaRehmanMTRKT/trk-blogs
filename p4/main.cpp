// =============================================================================
//  Post: p4 — Adding the Cooling Sector (Absorption + Electric Chillers)
//  File: main.cpp
//
//  Snippet map (referenced from trk-blogs-docs/p4.tex):
//    snippet#1 lines  63-117 : Cooling demand and chiller-related data
//    snippet#2 lines 118-224 : New decision variables: Hac (AC heat) and Pec
//    snippet#3 lines 225-273 : Coupled balances — electricity, heat, cooling
//    snippet#4 lines 275-302 : CSV output with every variable in its own column
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

    // ===== Input data inherited from p3 =====
    int T     = 24;
    int Cdg1  = 135;
    int Cdg2  = 140;
    int Chob  = 80;
    int Cchp1 = 150;
    int Cchp2 = 145;

    float socini = 0.2f, socmin = 0.1f, socmax = 0.9f;  // Battery / heat-storage / EV SoC: initial, min, max
    int   Pbmax  = 150;
    float effin  = 0.95f;
    int   Hssmax = 50;
    float Heffin = 0.95f;
    float k1 = 1.2f;
    float k2 = 0.8f;

    int   numEvs   = 5;
    float Eveffin  = 0.90f;
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

    // -------- snippet#1 lines 63-117 : Cooling demand and chiller efficiencies --------
    // The cooling load is served by two devices:
    //   - an absorption chiller (AC) that consumes heat with COP_ac = 0.85
    //   - an electric chiller   (EC) that consumes electricity with COP_ec = 0.95
    // Their decision variables (Hac and Pec) are introduced in snippet#2 below;
    // the COPs are hard-coded in the cooling balance in snippet#3.
    int* Cload  = new int[T] {
        100, 100,  80, 100, 120, 135, 150, 135,
        125, 130, 140, 150, 150, 130, 120, 110,
         90,  80, 135, 150, 135, 140, 110, 125 };

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

    // ===== Microgrid decision variables (electric + heat sides, same as p3) =====
    IloNumVarArray PGbuy  (env, T, 0, IloInfinity);
    IloNumVarArray PGsell (env, T, 0, IloInfinity);
    IloNumVarArray statoc (env, T, socmin, socmax);
    IloNumVarArray Bchg   (env, T, 0, 100);
    IloNumVarArray Bdischg(env, T, 0, 100);
    IloNumVarArray Pdg1   (env, T, 0, 100, ILOINT);
    IloNumVarArray Pdg2   (env, T, 0,  80, ILOINT);
    IloNumVarArray Pchp1  (env, T, 30, 60, ILOINT);
    IloNumVarArray Pchp2  (env, T, 50,100, ILOINT);

    IloNumVarArray HGbuy  (env, T, 0, IloInfinity, ILOINT);
    IloNumVarArray HGsell (env, T, 0, IloInfinity, ILOINT);
    IloNumVarArray HSSsoc (env, T, socmin, socmax);
    IloNumVarArray Hchg   (env, T, 0, 50, ILOINT);
    IloNumVarArray Hdischg(env, T, 0, 50, ILOINT);
    IloNumVarArray Hhob   (env, T, 0, 80, ILOINT);
    IloNumVarArray Hchp1  (env, T, 0, IloInfinity);
    IloNumVarArray Hchp2  (env, T, 0, IloInfinity);

    // -------- snippet#2 lines 118-224 : Cooling-sector decision variables --------
    // Hac : heat drawn from the heat bus by the absorption chiller (kWh-th)
    // Pec : electricity drawn from the electric bus by the electric chiller (kW)
    // Both are continuous and non-negative; their cooling contribution is
    // computed downstream by multiplying by the chiller's COP.
    IloNumVarArray Hac (env, T, 0, IloInfinity);
    IloNumVarArray Pec (env, T, 0, IloInfinity);

    // Big-M mutex selectors (battery, heat storage; per-EV uev declared below)
    IloBoolVarArray ubat(env, T);
    IloBoolVarArray uhst(env, T);

    // EV decision variables (same as p3) plus per-vehicle Big-M selector uev
    NumVar2D  Pevchg   (env, numEvs);
    NumVar2D  Pevdischg(env, numEvs);
    NumVar2D  evsoc    (env, numEvs);
    BoolVar2D uev      (env, numEvs);
    for (int n = 0; n < numEvs; n++) {
        Pevchg[n]    = IloNumVarArray (env, T, 0, evcap[n], ILOFLOAT);
        Pevdischg[n] = IloNumVarArray (env, T, 0, evcap[n], ILOFLOAT);
        evsoc[n]     = IloNumVarArray (env, T, socmin, socmax, ILOFLOAT);
        uev[n]       = IloBoolVarArray(env, T);
    }

    // ===== Objective (operating costs of generation + grid trading) =====
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

    // ===== Generator / CHP / HOB / Storage / EV constraints (same as p3) =====
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
        model.add(Bchg[t]    <= 1e6 *      ubat[t]);
        model.add(Bdischg[t] <= 1e6 * (1 - ubat[t]));
        model.add(Hchg[t]    <= 1e6 *      uhst[t]);
        model.add(Hdischg[t] <= 1e6 * (1 - uhst[t]));

        if (t == 0) {
            model.add(statoc[t] == socini
                      + ((effin * Bchg[t] - Bdischg[t] / effin) / Pbmax));
            model.add(Bchg[t]    <= (Pbmax * (socmax - socini) / effin));
            model.add(Bdischg[t] <= (Pbmax * (socini - socmin) * effin));

            model.add(HSSsoc[t] == 0.5
                      + ((Heffin * Hchg[t] - Hdischg[t] / Heffin) / Hssmax));
            model.add(Hchg[t]    <= (Hssmax * (socmax - 0.5) / Heffin));
            model.add(Hdischg[t] <= (Hssmax * (0.5 - socmin) * Heffin));
        } else {
            model.add(statoc[t] == statoc[t - 1]
                      + ((effin * Bchg[t] - Bdischg[t] / effin) / Pbmax));
            model.add(Bchg[t]    <= (Pbmax * (socmax - statoc[t - 1])) / effin);
            model.add(Bdischg[t] <= Pbmax * (statoc[t - 1] - socmin) * effin);

            model.add(HSSsoc[t] == HSSsoc[t - 1]
                      + ((Heffin * Hchg[t] - Hdischg[t] / Heffin) / Hssmax));
            model.add(Hchg[t]    <= (Hssmax * (socmax - HSSsoc[t - 1]) / Heffin));
            model.add(Hdischg[t] <= Hssmax * (HSSsoc[t - 1] - socmin) * Heffin);
        }

        for (int n = 0; n < numEvs; n++)
        {
            if (t == ta[n])
            {
                model.add(evsoc[n][t] == evsocini[n]
                          + (Eveffin * Pevchg[n][t] - Pevdischg[n][t] / Eveffin) / evcap[n]);
                model.add(Pevchg[n][t]    <= evcap[n] * (socmax - evsocini[n]) / Eveffin);
                model.add(Pevdischg[n][t] <= evcap[n] * (evsocini[n] - socmin) * Eveffin);
                // Big-M mutex: per-vehicle charge XOR discharge
                model.add(Pevchg[n][t]    <= 1e6 *      uev[n][t]);
                model.add(Pevdischg[n][t] <= 1e6 * (1 - uev[n][t]));
            }
            else if (t > ta[n] && t <= td[n])
            {
                model.add(evsoc[n][t] == evsoc[n][t - 1]
                          + (Eveffin * Pevchg[n][t] - Pevdischg[n][t] / Eveffin) / evcap[n]);
                model.add(Pevchg[n][t]    <= evcap[n] * (socmax - evsoc[n][t - 1]) / Eveffin);
                model.add(Pevdischg[n][t] <= evcap[n] * (evsoc[n][t - 1] - socmin) * Eveffin);
                // Big-M mutex: per-vehicle charge XOR discharge
                model.add(Pevchg[n][t]    <= 1e6 *      uev[n][t]);
                model.add(Pevdischg[n][t] <= 1e6 * (1 - uev[n][t]));
                if (t == td[n]) model.add(evsoc[n][t] >= 0.5);
            }
            else
            {
                model.add(Pevchg[n][t]    == 0);
                model.add(Pevdischg[n][t] == 0);
            }
        }
    }

    // -------- snippet#3 lines 225-273 : Coupled balances — electricity, heat, cooling --------
    // The electric chiller appears as an extra electric load (-Pec); the
    // absorption chiller appears as an extra heat load (-Hac); together they
    // produce cooling power that must equal the cooling demand:
    //     COP_ac * Hac + COP_ec * Pec == Cload.
    for (int t = 0; t < T; t++)
    {
        IloExpr evCharge   (env);
        IloExpr evDischarge(env);
        for (int n = 0; n < numEvs; n++) {
            evCharge    += Pevchg[n][t];
            evDischarge += Pevdischg[n][t];
        }

        // Electric balance — Pec is a new load.
        model.add(Pdg1[t] - Pec[t] + Pdg2[t] + Rdg1[t]
                  + Pchp1[t] + Pchp2[t]
                  + Bdischg[t] - Bchg[t]
                  + evDischarge - evCharge
                  + PGbuy[t] - PGsell[t] == Pload[t]);

        // Heat balance — Hac is a new load on the heat bus.
        model.add(Hhob[t] - Hac[t] + Hchp1[t] + Hchp2[t]
                  + HGbuy[t] - HGsell[t]
                  - Hchg[t]  + Hdischg[t] == Hload[t]);

        // Cooling balance — combined chiller output meets the cooling demand.
        model.add(0.85 * Hac[t] + 0.95 * Pec[t] == Cload[t]);
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

    // -------- snippet#4 lines 275-302 : CSV output with every variable in its own column --------
    // Each decision variable is written to a separate column so the
    // electric, heat, and cooling sectors can be inspected directly:
    //   - Pload, Hload, Cload         : raw demands on each carrier
    //   - PGbuy/PGsell, HGbuy/HGsell  : grid trading on electric/heat buses
    //   - Bchg/Bdischg, Hchg/Hdischg  : storage flows (and SoC trajectories)
    //   - Pdg1, Pdg2, Pchp1, Pchp2    : dispatchable electric units
    //   - Hhob, Hchp1, Hchp2          : heat producers
    //   - Hac, Pec                    : energy *consumed* by the AC and EC
    //   - Cac = COP_ac*Hac, Cec       : cooling *produced* by the AC and EC
    //   - Pevchg/Pevdischg/evsoc      : per-vehicle EV trajectories
    std::ofstream outputFile("output.csv");
    if (outputFile.is_open()) {
        outputFile << "Time,Pload,Hload,Cload,"
                      "CGbuy,CGsell,CHbuy,CHsell,Rdg1,"
                      "PGbuy,PGsell,statoc,Bchg,Bdischg,"
                      "Pdg1,Pdg2,Pchp1,Pchp2,"
                      "Hhob,Hchp1,Hchp2,HGbuy,HGsell,"
                      "HSSsoc,Hchg,Hdischg,"
                      "Hac,Pec,Cac,Cec";
        for (int n = 0; n < numEvs; n++) outputFile << ",Pevchg"    << n + 1;
        for (int n = 0; n < numEvs; n++) outputFile << ",Pevdischg" << n + 1;
        for (int n = 0; n < numEvs; n++) outputFile << ",evsoc"     << n + 1;
        outputFile << std::endl;

        for (int i = 0; i < T; i++) {
            outputFile << i + 1 << ","
                       << Pload[i]  << "," << Hload[i] << "," << Cload[i] << ","
                       << CGbuy[i]  << "," << CGsell[i] << ","
                       << CHbuy[i]  << "," << CHsell[i] << "," << Rdg1[i] << ","
                       << cplex.getValue(PGbuy[i])   << "," << cplex.getValue(PGsell[i])  << ","
                       << cplex.getValue(statoc[i])  << ","
                       << cplex.getValue(Bchg[i])    << "," << cplex.getValue(Bdischg[i]) << ","
                       << cplex.getValue(Pdg1[i])    << "," << cplex.getValue(Pdg2[i])    << ","
                       << cplex.getValue(Pchp1[i])   << "," << cplex.getValue(Pchp2[i])   << ","
                       << cplex.getValue(Hhob[i])    << ","
                       << cplex.getValue(Hchp1[i])   << "," << cplex.getValue(Hchp2[i])   << ","
                       << cplex.getValue(HGbuy[i])   << "," << cplex.getValue(HGsell[i])  << ","
                       << cplex.getValue(HSSsoc[i])  << ","
                       << cplex.getValue(Hchg[i])    << "," << cplex.getValue(Hdischg[i]) << ","
                       << cplex.getValue(Hac[i])     << "," << cplex.getValue(Pec[i])     << ","
                       << 0.85 * cplex.getValue(Hac[i]) << ","
                       << 0.95 * cplex.getValue(Pec[i]);
            for (int n = 0; n < numEvs; n++) {
                bool connected = (i >= ta[n] && i <= td[n]);
                outputFile << "," << (connected ? cplex.getValue(Pevchg[n][i]) : 0.0);
            }
            for (int n = 0; n < numEvs; n++) {
                bool connected = (i >= ta[n] && i <= td[n]);
                outputFile << "," << (connected ? cplex.getValue(Pevdischg[n][i]) : 0.0);
            }
            for (int n = 0; n < numEvs; n++) {
                bool connected = (i >= ta[n] && i <= td[n]);
                outputFile << "," << (connected ? cplex.getValue(evsoc[n][i]) : 0.0);
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
