#include <cassert>
#include <iostream>
#include <stdexcept>
#include <string>

#include "motionbrain_hardware_interface/proposal_command_ids.hpp"

using motionbrain_hardware_interface::ProposalCommandIds;

int main(int argc, char ** argv)
{
  if (argc == 2 && std::string(argv[1]) == "random") {
    ProposalCommandIds first;
    ProposalCommandIds second;
    if (!first.initialize() || !second.initialize()) {
      std::cerr << "Entropy unavailable: producer correctly refuses initialization\n";
      return 2;
    }
    std::string a;
    std::string b;
    assert(first.next(a));
    assert(second.next(b));
    assert(a != b);
    std::cout << a << '\n' << b << '\n';
    return 0;
  }

  ProposalCommandIds first;
  std::string id = "must-be-cleared";
  assert(!first.next(id));
  assert(id.empty());
  assert(first.initialize_with([]() {return ProposalCommandIds::SessionWords{1, 2, 3, 4};}));
  assert(first.next(id));
  assert(id == "ros2-control-00000001000000020000000300000004-1");
  assert(first.next(id));
  assert(id == "ros2-control-00000001000000020000000300000004-2");

  ProposalCommandIds second;
  assert(second.initialize_with([]() {return ProposalCommandIds::SessionWords{1, 2, 3, 5};}));
  std::string second_id;
  assert(second.next(second_id));
  assert(second_id == "ros2-control-00000001000000020000000300000005-1");
  assert(id != second_id);

  // Reconfiguration creates a new identity. A failed reconfiguration must not
  // retain a previously usable session or silently fall back to sequence-only.
  assert(first.initialize_with([]() {return ProposalCommandIds::SessionWords{5, 6, 7, 8};}));
  assert(first.next(id));
  assert(id == "ros2-control-00000005000000060000000700000008-1");
  assert(!first.initialize_with([]() -> ProposalCommandIds::SessionWords {
    throw std::runtime_error("injected entropy failure");
  }));
  assert(!first.next(id));
  assert(id.empty());
  assert(!first.initialize_with([]() {return ProposalCommandIds::SessionWords{};}));
  assert(!first.next(id));
  return 0;
}
