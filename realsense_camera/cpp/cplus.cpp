#include <iostream>   // for std::cout (printing)

// A named namespace — a "box" around names (like a Python module).
namespace demo
{
  constexpr float cWEIGHT_LIVING = 3.0f;   // compile-time constant

  int RiskLevel(float distance_m)
  {
    if (distance_m < 0.5f) return 4;
    if (distance_m < 1.0f) return 3;
    if (distance_m < 2.0f) return 1;
    return 0;
  }
}

// An anonymous namespace — private to this file only.
namespace
{
  const char* Label(int level)
  {
    return level >= 3 ? "HIGH" : (level >= 1 ? "LOW" : "NONE");
  }
}

int main()
{
  float d = 0.8f;
  int level = demo::RiskLevel(d);          // :: reaches into namespace demo
  std::cout << "distance=" << d << "m  weight=" << demo::cWEIGHT_LIVING
            << "  risk=" << level << " (" << Label(level) << ")\n";
  return 0;
}
