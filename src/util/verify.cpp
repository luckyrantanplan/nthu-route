#include <string>
#include "verifier.h"

using namespace std;

int mainVerifier(const int argc, const char* argv[]) {
    string inputFileName = "input.txt";
    string outputFileName = "output.txt";

    switch (argc) {
    case 3:
        outputFileName = argv[2];
        break;
    case 2:
        inputFileName = argv[1];
        break;
    case 1:
    default:
        break;
    }

    Verifier verifier(inputFileName.c_str(), outputFileName.c_str());
    verifier.verify();

    return 0;
}
