#include <iostream>
#include <algorithm>

int main()
{
    auto add = [](int a, int b) -> int { return a + b; };
    int sum = add(3, 5);
    auto print_sum = [sum]()->void { std::cout << "3 + 5 = " << sum << std::endl; };
    print_sum();
    return 0;
}
