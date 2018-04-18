
# Libs required for the tests
SET(LIBS ${LIBS} ArmarXCore DSRTController)
 
armarx_add_test(DSRTControllerTest DSRTControllerTest.cpp "${LIBS}")