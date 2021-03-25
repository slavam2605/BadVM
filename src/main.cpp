#include <iostream>
#include "vm/vm.h"

using namespace std;

int main() {
    vm jvm;
    jvm.start("Sample");
    return 0;
}
