#ifndef TIMEMACROS_H
#define TIMEMACROS_H
// Record the execution time of some code, in milliseconds.
#define DECLARE_TIMING(s)  int64 timeStart_##s; double timeDiff_##s; double timeTally_##s = 0; int countTally_##s = 0
#define START_TIMING(s)    timeStart_##s = cvGetTickCount()
#define STOP_TIMING(s) 	   timeDiff_##s = (double)(cvGetTickCount() - timeStart_##s); timeTally_##s += timeDiff_##s; countTally_##s++
#define GET_TIMING(s) 	   (double)(timeDiff_##s / (cvGetTickFrequency()*1000.0))
#define GET_AVERAGE_TIMING(s)   (double)(countTally_##s ? timeTally_##s/ ((double)countTally_##s * cvGetTickFrequency()*1000.0) : 0)
#define CLEAR_AVERAGE_TIMING(s) timeTally_##s = 0; countTally_##s = 0

#endif /*End TIMEMACROS_H*/

/*
// Example:
DECLARE_TIMING(myTimer);
while (something)
{
    START_TIMING(myTimer);
    printf("Hello!\n");
    STOP_TIMING(myTimer);
}
printf("Execution time: %f ms.\n", GET_TIMING(myTimer) );
printf("Average time: %f ms per iteration.\n", GET_AVERAGE_TIMING(myTimer) );

*/