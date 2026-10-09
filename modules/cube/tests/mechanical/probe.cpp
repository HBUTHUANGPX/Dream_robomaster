#include "rm/mechanical.hpp"
#include "rm/sim.hpp"
int main(int argc, char** argv) {
  return rm::cube::mechanical_main(rm::repo_root(argc, argv), argc, argv);
}
