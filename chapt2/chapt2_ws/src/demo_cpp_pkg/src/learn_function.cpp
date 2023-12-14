#include <iostream>
#include <functional>

void save_with_free_fun(const std::string &file_name)
{
    std::cout << "调用了自由函数，保存:" << file_name << std::endl;
}


class FileSave
{
public:
    void save_with_member_fun(const std::string &file_name)
    {
        std::cout << "调用了成员方法，保存:" << file_name << std::endl;
    };
};


int main()
{
    FileSave file_save;
  	auto save_with_lambda_fun = [](const std::string &file_name) -> void
    {
        std::cout << "调用了Lambda 函数，保存:" << file_name << std::endl;
    };
    // 将自由函数放进function对象中
    std::function<void(const std::string &)> save1 = save_with_free_fun; 
    // 将Lambda函数放入function对象中
    std::function<void(const std::string &)> save2 = save_with_lambda_fun;
    // 将成员方法放入function对象中
    std::function<void(const std::string &)> save3 = std::bind(&FileSave::save_with_member_fun, &file_save, std::placeholders::_1);
    // 无论哪种函数都可以使用统一的调用方式
    save1("file.txt");
    save2("file.txt");
    save3("file.txt");
    return 0;
}
