//
// Created by bontius on 11/07/16.
//

// http://www.ibm.com/developerworks/aix/library/au-googletestingframework.html

// http://askubuntu.com/questions/145887/why-no-library-files-installed-for-google-test
// cd /usr/src/gtest
// sudo cmake .
// sudo make
// sudo mv libg* /usr/lib/

#include "gtest/gtest.h"


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}