#include "myclass.h"

MyClass::MyClass(int number) : myNumber(number){};

void MyClass::sayHello()
{
    std::cout << "3rd Test:"
              << myNumber << std::endl;
}
