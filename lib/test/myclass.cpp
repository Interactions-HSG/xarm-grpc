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
