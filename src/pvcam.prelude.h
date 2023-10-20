#ifndef H_ACQUIRE_PVCAM_PRELUDE_V0
#define H_ACQUIRE_PVCAM_PRELUDE_V0

#include "master.h" // must come before pvcam.h
#include "pvcam.h"

#define L (aq_logger)
#define LOG(...) L(0, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)
#define ERR(...) L(1, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)

// #define TRACE(...) LOG(__VA_ARGS__)
#define TRACE(...)

#define PVISFAIL(status) (PV_FAIL == (status))

#define EXPECT_INNER(eval, test, action, logger, ...)                          \
    do {                                                                       \
        eval;                                                                  \
        if (!(test)) {                                                         \
            logger(__VA_ARGS__);                                               \
            action;                                                            \
        }                                                                      \
    } while (0)
#define EXPECT(e, ...) EXPECT_INNER(, e, goto Error, ERR, __VA_ARGS__)
#define CHECK(e) EXPECT(e, "Expression was false:\n\t%s\n", #e)
#define WARN(e) EXPECT_INNER(, e, , LOG, "Expression was false:\n\t%s\n", #e)
//#define PVCAM_INNER(e, logger, action)                                          \
//    EXPECT_INNER(!PVISFAIL((e)),                                                \
//                 action,                                                        \
//                 logger,                                                        \
//                 "Expression failed:\n\t%s\n\t%s\n",                            \
//                 #e,                                                            \
//                 pl_error_message(result_))
//
//#define PVCAM(e) PVCAM_INNER(e, ERR, goto Error)
//#define PVWARN(e) PVCAM_INNER(e, LOG, )

#endif // H_ACQUIRE_PVCAM_PRELUDE_V0
