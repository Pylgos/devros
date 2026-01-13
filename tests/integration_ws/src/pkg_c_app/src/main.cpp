#include <iostream>
#include "pkg_a_lib/pkg_a_lib.hpp"
#include "pkg_b_lib/pkg_b_lib.hpp"

int main()
{
  std::cout << "Messages from libraries:" << std::endl;
  std::cout << "  A: " << pkg_a_lib::get_message_a() << std::endl;
  std::cout << "  B: " << pkg_b_lib::get_message_b() << std::endl;
  return 0;
}
