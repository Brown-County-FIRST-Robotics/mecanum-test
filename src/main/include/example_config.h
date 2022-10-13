//#define logitech_dual_action
//#define logitech_gamepad_f310

#ifdef logitech_dual_action
    #define leftX 0
    #define leftY 1
    #define rightX 2
    #define rightY 3
    #define A_button 2
    #define triggerL 7
    #define triggerR 8
#endif

#ifdef logitech_gamepad_f310
    #define leftX 0
    #define leftY 1
    #define rightX 3
    #define rightY 4
    #define analogTrigger
    #define A_button 1
    #define triggerL 2
    #define triggerR 5
#endif
