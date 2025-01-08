Disclaimer: these were developped for NimBLE v1.0.0 in 2018, so rather old !

nimble.mk is i


# NimBLE modifications


I had to modify the NimBLE sources a slight bit to make them compatible for C++.

All modifications are marked with //SEB or /*SEB

Files concerned are:
* ble_uuid.h
* modlog.h
* nimble_hci.cpp
* syscfg.h


# NimBLE OS abstraction

To make Nimble compatible with Circle, you need(ed?) to implement an abstraction layer, i.e.
* nimble_nlp_os.h
* nimble_nlp_os.cpp

This is a very minial implementation that worked for me, but it probably needs enhanced...

In particular, LinkedList.h is not included here and is my self-baked implementation of a linked list. You'll probably want to replace that :)

Also, the new versions of circle now have mutex/semaphores (I think), so you should use that instead.




# Usage from circle

The file bluetooth.cpp is the main code starting nimble and interacting with it.

