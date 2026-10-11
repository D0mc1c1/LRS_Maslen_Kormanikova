#pragma once
#include <cstddef>
#include <optional>

#include "lrs_planning/types.hpp"

namespace lrs_planning
{

// Vysledok planovania jedneho segmentu misie.
struct PlanResult
{
  Path path;                   // zjednodusena cesta v map frame, VRATANE start aj goal
  double time_ms{0.0};         // cas planovania
  std::size_t raw_points{0};   // pocet bodov pred zjednodusenim (pre dokumentaciu)
  std::size_t expanded{0};     // pocet expandovanych uzlov A* (0 ak sa nepouziva)
};

// KONTRAKT medzi A1.1 (lrs_planning) a A1.2 (lrs_control).
//
//  - start a goal su v map frame (ENU, metre), rovnako ako CSV misie
//  - vratena cesta zacina v `start` a konci v `goal` (presne tieto body)
//  - medzi nimi su len body po zjednoduseni, nie kazdy voxel
//  - std::nullopt = cesta neexistuje (alebo start/goal je v prekazke)
//  - plan() moze trvat dlhsie (sekundy), vola sa pred vzletom
class Planner
{
public:
  virtual ~Planner() = default;
  virtual std::optional<PlanResult> plan(const Vec3 & start, const Vec3 & goal) = 0;
};

// DOCASNY planovac pre vyvoj A1.2, kym nie je hotovy A*.
// Vrati priamku start -> goal a NEKONTROLUJE kolizie.
// Pouzivat len s misiou v otvorenom priestore (missions/test_open.csv).
class StraightLinePlanner : public Planner
{
public:
  std::optional<PlanResult> plan(const Vec3 & start, const Vec3 & goal) override
  {
    PlanResult r;
    r.path = {start, goal};
    r.raw_points = 2;
    return r;
  }
};

}  // namespace lrs_planning
