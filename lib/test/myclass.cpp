#include "myclass.h"
#include <string>

MyClass::MyClass(int number) : myNumber(number){};

void MyClass::sayHello()
{
    std::cout << "3rd Test:"
              << myNumber << std::endl;
}

Xarm::Xarm(
    const std::string &port,
    int number,
    bool test) : myPort(port), myNumber(number), myBool(test){};

int Xarm::run(int *num)
{
    *num = 5 + *num;
    return 1;
}

int *array()
{
    int res[2] = {1, 2};
    return res;
}
