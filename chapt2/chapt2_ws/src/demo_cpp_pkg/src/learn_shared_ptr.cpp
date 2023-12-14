#include <iostream>
#include <memory>

int main()
{
    auto p1 = std::make_shared<std::string>("This is a str.");
    std::cout << "p1 的引用计数为：" << p1.use_count() << "，指向内存的地址为：" << p1.get() << std::endl;

    auto p2 = p1;
    std::cout << "p1 的引用计数为：" << p1.use_count() << "，指向内存的地址为：" << p1.get() << std::endl;
    std::cout << "p2 的引用计数为：" << p2.use_count() << "，指向内存的地址为：" << p2.get() << std::endl;

    p1.reset(); 
    std::cout << "p1 的引用计数为：" << p1.use_count() << "，指向内存的地址为：" << p1.get() << std::endl;
    std::cout << "p2 的引用计数为：" << p2.use_count() << "，指向内存的地址为：" << p2.get() << std::endl;
    std::cout << "p2 指向资源的内容为：" << p2->c_str() << std::endl;
    return 0;
}
