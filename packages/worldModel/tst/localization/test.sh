# dev wrapper to show tracing in case a test case fails
# to run all tests: 
#      ./test.sh
# to run a specific test, for example #5: 
#      ./test.sh --gtest_filter=LocalizationFileIOTest/LocalizationFileIOTest.testFileIO/5


#roscd worldModel
../../bin/testLocalizationIO --gtest_color=yes $* | tee /var/tmp/testOutput.txt
success=${PIPESTATUS[0]}

if [ $success != "0" ]; then
    # find first failing test case
    failId=`grep FAILED /var/tmp/testOutput.txt | head -1 | awk '{print $4}' | tr -d , | tr -cd "[:print:]\n" | sed 's/^..\(.*\)/\1/'`
    # rerun
    echo
    echo "re-running the test and opening the log ..."
    echo
    ../../bin/testLocalizationIO --gtest_color=yes --gtest_filter=$failId | tee /var/tmp/testOutput.txt
    # open logging
    tracefile=`head -1 /var/tmp/testOutput.txt | awk '{print $NF}'`
    nedit $tracefile
fi

