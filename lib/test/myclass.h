#ifndef MYCLASS_H
#define MYCLASS_H

#include <iostream>
#include <string>

class MyClass
{
        int myNumber;

public:
        MyClass(int number);
        void sayHello();
};

class Xarm
{
public:
        Xarm(const std::string &port = "PORT-8000",
             int number = 5,
             bool test = true);

        std::string myPort;
        int myNumber;
        bool myBool;

        int run(int *num);
};

int *array();

#endif
