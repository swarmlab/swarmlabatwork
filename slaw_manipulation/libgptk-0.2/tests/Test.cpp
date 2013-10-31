#include "Test.h"

Test::Test()
{
	header = "";
}

Test::~Test()
{
}

void Test::printHeader() {
	cout << "*************************************************" << endl;
	cout << header << endl;
	cout << "*************************************************" << endl;
	
}

/**
 * A simple output header function
 */
void Test::printTestDescription(string desc) {
	cout << desc << " ";
}

/**
 * Test wrapper
 * This simply runs the tests and outputs a failed/passed message
 */
void Test::runTest( bool testFunction()  ) {
	if (testFunction()) 
		cout << "[Passed]" << endl << endl;
	else 
		cout << "[Failed]" << endl << endl;
}

/**
 * Add a test function to the list of tests and the corresponding
 * header to the list of headers
 */
void Test::addTest(bool function(), string description) {
	tests.push_back(function);
	descriptions.push_back(description);
}

void Test::run() {
	assert(tests.size() == descriptions.size());
	
	// Display header
	printHeader();
	
	// Run all tests
	for (int i=0; i<tests.size(); i++) {
		printTestDescription(descriptions[i]);
		runTest(tests[i]);
	}
	
	cout << "End of test." << endl;
}

