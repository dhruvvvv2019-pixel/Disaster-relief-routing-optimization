#include <iostream>
#include <string>
#include <chrono>
#include "structures.h"
#include "io_handler.h"
#include "solver.h" // CORRECT: Includes the header, NOT the .cpp file

using namespace std;

// This is the one and only 'main' function for your program.
int main(int argc, char* argv[]) {
    if (argc != 3) {
        cerr << "Usage: " << argv[0] << " <input_filename> <output_filename>" << endl;
        return 1;
    }

    string input_filename = argv[1];
    string output_filename = argv[2];

    try {
        // Reads the input data using the function from io_handler.cpp
        ProblemData problem = readInputData(input_filename);
        cout << "Successfully read input file: " << input_filename << endl;

        auto start_time = chrono::steady_clock::now();
        
        // Calls the 'solve' function from your refactored solver.cpp
        Solution solution = solve(problem);
        
        auto end_time = chrono::steady_clock::now();
        auto elapsed = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);
        
        cout << "Solver completed in " << elapsed.count() / 1000.0 << " seconds." << endl;
        
        if (elapsed.count() / 60000.0 > problem.time_limit_minutes) {
             cerr << "TimeLimitExceeded: Solver exceeded the time limit of " << problem.time_limit_minutes << " minutes." << endl;
             cout << "This instance will receive a score of 0." << endl;
        }

        // Writes the solution to a file using the function from io_handler.cpp
        writeOutputData(output_filename, solution);
        cout << "Successfully wrote solution to output file: " << output_filename << endl;

    } catch (const runtime_error& e) {
        cerr << "An error occurred: " << e.what() << endl;
        return 1;
    }

    return 0;
}